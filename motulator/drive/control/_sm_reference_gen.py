"""Feedforward reference generation methods for synchronous machine drives."""

from cmath import exp, phase
from functools import partial
from math import inf, pi, sqrt

import numpy as np
from scipy.optimize import root_scalar

from motulator.common.utils._utils import clip, sign, wrap
from motulator.drive.utils._parameters import (
    BaseSynchronousMachineControlPars,
    aux_current,
    aux_flux,
    mat_vec,
)
from motulator.drive.utils._sm_control_loci import ControlLoci

EPS = 1e-4


# %%
class ReferenceGeneratorBase:
    """
    Base class for the reference generators of synchronous machines.

    The base class holds the flux limits and the current reference, which is a state
    driven toward the flux and torque references by a tracking law [#Sar2026]_. The
    current reference is needed only for current-vector control. The parameters are
    described in `ReferenceGenerator`.

    """

    def __init__(
        self,
        par: BaseSynchronousMachineControlPars,
        psi_s_min: float | None,
        psi_s_max: float,
        k_u: float,
        k_mtpv: float,
        alpha_ref: float,
    ) -> None:
        self.par = par
        self.k_u = k_u
        self.k_mtpv = k_mtpv
        self.alpha = alpha_ref

        psi_s_min = par.psi_f if psi_s_min is None else psi_s_min
        if par.psi_f == 0 and psi_s_min <= 0:
            raise ValueError("psi_s_min must be positive for machines without PMs.")
        self.psi_s_limits = (psi_s_min, psi_s_max)

        self.i_s_ref = 0j  # Current-reference state, initialized by the subclasses
        self.tau_M_ref = 0.0
        self.psi_s_ref = psi_s_min

    def _evaluate(self, i_s: complex) -> tuple[complex, float]:
        """Flux linkage and torque produced by the given current."""
        psi_s = complex(self.par.psi_s_dq(i_s))
        tau_M = 1.5 * self.par.n_p * (i_s * psi_s.conjugate()).imag
        return psi_s, tau_M

    def _get_max_flux(self, w_m: float, u_dc: float) -> float:
        """Get the maximum available flux linkage."""
        u_s_max = self.k_u * u_dc / sqrt(3)
        return u_s_max / abs(w_m) if w_m != 0 else inf

    def compute_current_ref(self, tau_M_ref: float) -> complex:
        """
        Get the current reference, needed only for current-vector control.

        Parameters
        ----------
        tau_M_ref : float
            Limited torque reference (Nm), whose sign gives the sign of the q-axis
            current.

        Returns
        -------
        complex
            Stator current reference (A) in rotor coordinates.

        """
        return self.i_s_ref if tau_M_ref >= 0 else self.i_s_ref.conjugate()

    def update(self, T_s: float) -> None:
        """
        Update the current-reference state.

        Parameters
        ----------
        T_s : float
            Sampling period (s).

        """
        psi_s, tau_M = self._evaluate(self.i_s_ref)
        psi_s_abs = abs(psi_s)
        if psi_s_abs > 0:
            L_s = self.par.incr_ind_mat(self.i_s_ref)
            psi_a = aux_flux(self.i_s_ref, psi_s, L_s)
            ell = mat_vec(L_s, psi_s) / psi_s_abs
            den = (psi_a * ell.conjugate()).real
            if den != 0:
                err_tau = (abs(self.tau_M_ref) - tau_M) / (1.5 * self.par.n_p)
                err_psi = self.psi_s_ref - psi_s_abs
                err = (1j * ell * err_tau + psi_a * err_psi) / den
                self.i_s_ref += T_s * self.alpha * err


# %%
class ReferenceGenerator(ReferenceGeneratorBase):
    """
    Optimal feedforward reference generator for synchronous machines.

    This class computes the optimal flux, limited torque, and current references from a
    given torque reference. The MTPA locus as well as the current, voltage, and MTPV
    limits are taken into account. This class can be used also for a saturated machine
    model. The flux and torque references are computed using precomputed lookup tables
    [#Mey2006]_, [#Awa2018]_. The current reference is generated dynamically using a
    tracking law [#Sar2026]_, needed only for current-vector control.

    Parameters
    ----------
    par : BaseSynchronousMachineControlPars
        Machine model parameters.
    i_s_max : float
        Maximum stator current (A).
    psi_s_min : float | None
        Minimum stator flux (Vs). If None, `par.psi_f` is used. It must be positive for
        machines without PMs.
    psi_s_max : float
        Maximum stator flux (Vs).
    k_u : float
        Voltage utilization factor.
    k_mtpv : float
        MTPV margin.
    alpha_ref : float
        Bandwidth of the current-reference tracking (rad/s), needed only for
        current-vector control. It should be well below the sampling frequency to
        maintain a numerical margin.

    References
    ----------
    .. [#Mey2006] Meyer, Böcker, "Optimum control for interior permanent magnet
       synchronous motors (IPMSM) in constant torque and flux weakening range," Proc.
       EPE-PEMC, 2006, https://doi.org/10.1109/EPEPEMC.2006.4778413

    .. [#Awa2018] Awan, Song, Saarakkala, Hinkkanen, "Optimal torque control of
       saturated synchronous motors: Plug-and-play method," IEEE Trans. Ind. Appl.,
       2018, https://doi.org/10.1109/TIA.2018.2862410

    .. [#Sar2026] Sarén, Hartikainen, Piippo, Hinkkanen, "Decoupled online feedforward
       generation of optimal references for saturated synchronous machine drives," 2026,
       https://arxiv.org/abs/2607.08528

    """

    def __init__(
        self,
        par: BaseSynchronousMachineControlPars,
        i_s_max: float,
        psi_s_min: float | None,
        psi_s_max: float,
        k_u: float,
        k_mtpv: float,
        alpha_ref: float,
    ) -> None:
        super().__init__(par, psi_s_min, psi_s_max, k_u, k_mtpv, alpha_ref)
        psi_s_min = self.psi_s_limits[0]

        # Generate LUTs
        loci = ControlLoci(par)

        # MTPA locus
        mtpa = loci.compute_mtpa_locus(i_s_max)
        self.psi_s_mtpa = partial(np.interp, xp=mtpa.tau_M, fp=abs(mtpa.psi_s_dq))

        # MTPV limit at the flux linkages where the MTPV exists, no limit elsewhere
        mtpv = loci.compute_mtpv_locus(abs(mtpa.psi_s_dq[-1]))
        valid = ~np.isnan(mtpv.tau_M)
        psi_mtpv, tau_mtpv = abs(mtpv.psi_s_dq[valid]), mtpv.tau_M[valid]
        if not valid.any():
            psi_mtpv, tau_mtpv = np.zeros(1), np.full(1, inf)
        self.tau_M_mtpv = partial(np.interp, xp=psi_mtpv, fp=tau_mtpv, right=inf)

        # Current limit
        gamma1 = phase(loci.compute_mtpv_current(i_s_max))
        gamma2 = phase(mtpa.i_s_dq[-1])
        cl = loci.compute_const_current_locus(i_s_max, (gamma1, gamma2))
        self.tau_M_cl = partial(np.interp, xp=abs(cl.psi_s_dq), fp=cl.tau_M)

        # Current-reference state, initialized at the zero-torque operating point of the
        # minimum flux
        if par.psi_f == 0:
            self.i_s_ref = complex(psi_s_min / par.incr_ind_mat(0j)[0, 0])
        else:
            self.i_s_ref = complex(np.interp(0.0, mtpa.tau_M, mtpa.i_s_dq))

    def compute_flux_and_torque_refs(
        self, tau_M_ref: float, w_m: float, u_dc: float
    ) -> tuple[float, float]:
        """
        Compute the flux and torque reference signals.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference (Nm).
        w_m : float
            Electrical angular speed (rad/s).
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        tuple[float, float]
            Flux and torque reference signals.

        """
        # MTPA flux
        psi_s_abs_ref = clip(float(self.psi_s_mtpa(abs(tau_M_ref))), *self.psi_s_limits)

        # Maximum flux (field weakening)
        psi_s_abs_ref = min(psi_s_abs_ref, self._get_max_flux(w_m, u_dc))

        # Current limit
        tau_M_cl = float(self.tau_M_cl(psi_s_abs_ref))
        tau_M_ref = min(tau_M_cl, abs(tau_M_ref)) * sign(tau_M_ref)

        # MTPV limit
        tau_M_mtpv = float(self.tau_M_mtpv(psi_s_abs_ref))
        if tau_M_mtpv > 0:
            tau_M_ref = min(self.k_mtpv * tau_M_mtpv, abs(tau_M_ref)) * sign(tau_M_ref)

        self.psi_s_ref = psi_s_abs_ref
        self.tau_M_ref = tau_M_ref
        return psi_s_abs_ref, tau_M_ref


# %%
class ReferenceGeneratorOnline(ReferenceGeneratorBase):
    """
    Optimal feedforward online reference generator for synchronous machines.

    This class tracks online in a feedforward manner the optimal flux, limited torque,
    and current references from a given torque reference [#Sar2026]_. The MTPA locus as
    well as the current, voltage, and MTPV limits are taken into account. This class can
    be used also for a saturated machine model. The current reference is only needed for
    current-vector control.

    Parameters
    ----------
    par : BaseSynchronousMachineControlPars
        Machine model parameters.
    i_s_max : float
        Maximum stator current (A).
    psi_s_min : float | None
        Minimum stator flux (Vs). If None, `par.psi_f` is used. It must be positive for
        machines without PMs.
    psi_s_max : float
        Maximum stator flux (Vs).
    k_u : float
        Voltage utilization factor.
    k_mtpv : float
        MTPV margin.
    alpha_ref : float
        Bandwidth of the reference tracking (rad/s).

    """

    def __init__(
        self,
        par: BaseSynchronousMachineControlPars,
        i_s_max: float,
        psi_s_min: float | None,
        psi_s_max: float,
        k_u: float,
        k_mtpv: float,
        alpha_ref: float,
    ) -> None:
        super().__init__(par, psi_s_min, psi_s_max, k_u, k_mtpv, alpha_ref)
        psi_s_min = self.psi_s_limits[0]

        # Dynamic tracking states
        if self.par.psi_f == 0:  # SyRM
            self.gamma_limits = (0.25 * pi, 0.5 * pi)
            L_d0 = float(self.par.incr_ind_mat(0j)[0, 0])
            i_s_min = max(psi_s_min / L_d0 if L_d0 > 0 else 0.0, EPS * i_s_max)
            self.i_s_limits = (i_s_min, i_s_max)
        else:  # With PMs
            self.gamma_limits = (0.5 * pi, pi)
            self.i_s_limits = (0.0, i_s_max)

        def mtpv_cond(gamma: float) -> float:
            i_s = i_s_max * exp(1j * gamma)
            psi_s, _ = self._evaluate(i_s)
            L_s = self.par.incr_ind_mat(i_s)
            i_a = aux_current(psi_s, i_s, L_s)
            return (i_a * psi_s.conjugate()).real

        f0, f1 = mtpv_cond(self.gamma_limits[0]), mtpv_cond(self.gamma_limits[1])
        self.has_mtpv = f0 * f1 < 0  # MTPV within the current limit
        self.i_s_mtpv = 0j  # MTPV tracker state, used only with the MTPV
        if self.has_mtpv:
            res = root_scalar(mtpv_cond, bracket=self.gamma_limits, method="brentq")
            self.i_s_mtpv = complex(i_s_max * exp(1j * res.root))

        self.i_s_mtpa = self.i_s_limits[0] * exp(1j * self.gamma_limits[0])
        self.gamma_cl = 0.5 * sum(self.gamma_limits)  # Angle on the current limit
        # Zero torque at the minimum flux corresponds to pure d-axis current
        self.i_s_ref = complex(self.i_s_limits[0])

        # Cached from compute_flux_and_torque_refs, reused by the tracker updates
        self._mtpa_state = self._evaluate(self.i_s_mtpa)
        self._mtpv_state = self._evaluate(self.i_s_mtpv)
        self._cl_state = self._evaluate(self._i_s_cl)

    @property
    def _i_s_cl(self) -> complex:
        """Current on the current limit at the tracked angle."""
        return self.i_s_limits[1] * exp(1j * self.gamma_cl)

    def _clip_state(self, i_s: complex, max_angle: float) -> complex:
        """Constrain the current magnitude and angle to the limits."""
        # The angle is measured from the middle of the range, so that an angle beyond
        # the range is clipped to the nearest limit also across the negative real axis
        mid = 0.5 * (self.gamma_limits[0] + max_angle)
        gamma = clip(mid + wrap(phase(i_s) - mid), self.gamma_limits[0], max_angle)
        return max(abs(i_s), self.i_s_limits[0]) * exp(1j * gamma)

    def compute_flux_and_torque_refs(
        self, tau_M_ref: float, w_m: float, u_dc: float
    ) -> tuple[float, float]:
        """
        Compute flux and torque references.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference (Nm).
        w_m : float
            Electrical angular speed (rad/s).
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        tuple[float, float]
            Flux and torque reference signals.

        """
        # MTPA flux, cached for reuse in the MTPA tracker update
        self._mtpa_state = self._evaluate(self.i_s_mtpa)
        psi_s_abs_ref = clip(abs(self._mtpa_state[0]), *self.psi_s_limits)

        # Maximum flux (field weakening)
        psi_s_abs_ref = min(psi_s_abs_ref, self._get_max_flux(w_m, u_dc))

        # Current limit, cached for reuse in the current-limit tracker update
        self._cl_state = self._evaluate(self._i_s_cl)
        tau_M_cl = abs(self._cl_state[1])
        tau_M_ref = min(tau_M_cl, abs(tau_M_ref)) * sign(tau_M_ref)

        # MTPV limit, cached for reuse in the MTPV tracker update
        if self.has_mtpv:
            self._mtpv_state = self._evaluate(self.i_s_mtpv)
            tau_M_mtpv = abs(self._mtpv_state[1])
            if tau_M_mtpv > 0:
                tau_M_ref = min(self.k_mtpv * tau_M_mtpv, abs(tau_M_ref)) * sign(
                    tau_M_ref
                )

        self.psi_s_ref = psi_s_abs_ref
        self.tau_M_ref = tau_M_ref
        return psi_s_abs_ref, tau_M_ref

    def _update_mtpa(self, T_s: float) -> None:
        """Update the MTPA tracker state."""
        psi_s, tau_M = self._mtpa_state
        if abs(psi_s) > 0:
            L_s = self.par.incr_ind_mat(self.i_s_mtpa)
            L_delta = 0.5 * ((L_s[0, 0] - L_s[1, 1]) + 1j * (L_s[1, 0] + L_s[0, 1]))
            psi_a = aux_flux(self.i_s_mtpa, psi_s, L_s)
            phi_a = psi_a + 2 * L_delta * self.i_s_mtpa.conjugate()

            den = (psi_a * phi_a.conjugate()).real
            if den != 0:
                err_tau = (abs(self.tau_M_ref) - tau_M) / (1.5 * self.par.n_p)
                err_mtpa = (psi_a * self.i_s_mtpa.conjugate()).real
                err = (1j * phi_a * err_tau - psi_a * err_mtpa) / den
                self.i_s_mtpa += T_s * self.alpha * err

    def _update_mtpv(self, T_s: float) -> None:
        """Update the MTPV tracker state."""
        # Tracks the current vector satisfying the MTPV condition for a given flux
        psi_s, _ = self._mtpv_state
        psi_s_abs = abs(psi_s)
        if psi_s_abs > 0:
            L_s = self.par.incr_ind_mat(self.i_s_mtpv)
            det_L = L_s[0, 0] * L_s[1, 1] - L_s[0, 1] ** 2
            L_delta = 0.5 * ((L_s[0, 0] - L_s[1, 1]) + 1j * (L_s[1, 0] + L_s[0, 1]))
            L_psi = mat_vec(L_s, psi_s)
            ell = L_psi / psi_s_abs
            i_a = L_psi / det_L - self.i_s_mtpv
            j_a = i_a + 2 * (L_delta / det_L) * psi_s.conjugate()
            phi_e = mat_vec(L_s, j_a)

            den = (ell * phi_e.conjugate()).imag
            if den != 0:
                err_psi = self.psi_s_ref - psi_s_abs
                err_mtpv = (i_a * psi_s.conjugate()).real
                err = 1j * (phi_e * err_psi + ell * err_mtpv) / den
                self.i_s_mtpv += T_s * self.alpha * err

    def _update_lim(self, T_s: float) -> None:
        """Update the current-limit tracker state."""
        # Tracks the angle on the current limit that provides the requested flux, with
        # the Newton step limited to one radian. The flux magnitude decreases with the
        # angle in the tracked range. Its derivative vanishes on the d axis (on the q
        # axis for machines without PMs), so it is bounded away from zero.
        psi_s, _ = self._cl_state
        psi_s_abs = abs(psi_s)
        if psi_s_abs > 0:
            i_s = self._i_s_cl
            L_s = self.par.incr_ind_mat(i_s)
            ell = mat_vec(L_s, psi_s) / psi_s_abs
            den = min((ell * i_s.conjugate()).imag, -EPS * psi_s_abs)
            err = clip((self.psi_s_ref - psi_s_abs) / den, -1.0, 1.0)
            self.gamma_cl += T_s * self.alpha * err

    def update(self, T_s: float) -> None:
        """Update the tracking states."""
        max_angle = self.gamma_limits[1]
        self._update_mtpa(T_s)
        self.i_s_mtpa = self._clip_state(self.i_s_mtpa, max_angle)
        if self.has_mtpv:
            self._update_mtpv(T_s)
            self.i_s_mtpv = self._clip_state(self.i_s_mtpv, max_angle)
            max_angle = phase(self.i_s_mtpv)
        self._update_lim(T_s)
        self.gamma_cl = clip(self.gamma_cl, self.gamma_limits[0], max_angle)
        super().update(T_s)
