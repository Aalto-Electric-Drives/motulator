"""Feedforward reference generation methods for synchronous machine drives."""

from cmath import exp, phase
from math import inf, pi, sqrt

import numpy as np
from scipy.optimize import root_scalar

from motulator.common.utils._utils import clip, sign
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)
from motulator.drive.utils._sm_control_loci import ControlLoci

EPS = 1e-4


# %%
def _mat_vec(L_s: np.ndarray, x: complex) -> complex:
    """Matrix-vector product L_s @ x for symmetric 2x2 matrix."""
    return L_s[0, 0] * x.real + 1j * L_s[1, 1] * x.imag + 1j * L_s[0, 1] * x.conjugate()


def _aux_flux(i_s: complex, psi_s: complex, L_s: np.ndarray) -> complex:
    """Auxiliary flux linkage vector."""
    return (
        psi_s
        - L_s[1, 1] * i_s.real
        - 1j * L_s[0, 0] * i_s.imag
        + 1j * L_s[0, 1] * i_s.conjugate()
    )


def _aux_current(psi_s: complex, i_s: complex, L_s: np.ndarray) -> complex:
    """Auxiliary current vector."""
    det_L = L_s[0, 0] * L_s[1, 1] - L_s[0, 1] ** 2
    return _mat_vec(L_s, psi_s) / det_L - i_s


# %%
class ReferenceGenerator:
    """
    Optimal feedforward reference generator for synchronous machines.

    This class computes the optimal flux, limited torque, and current references from a
    given torque reference. The MTPA locus as well as the current, voltage and MTPV
    limits are taken into account. This class can be used also for a saturated machine
    model. The flux and torque references are computed using pre-computed lookup tables
    [#Mey2006]_, [#Awa2018]_. The current reference is generated dynamically using a
    tracking law [#Sar2026]_, needed only for current-vector control.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    i_s_max : float
        Maximum stator current (A).
    psi_s_min : float, optional
        Minimum stator flux (Vs), defaults to `par.psi_f`.
    psi_s_max : float, optional
        Maximum stator flux (Vs), defaults to `inf`.
    k_u : float, optional
        Voltage utilization factor, defaults to 1.
    k_mtpv : float, optional
        MTPV margin, defaults to 1.
    alpha_ref : float, optional
        Bandwidth of the reference tracking (rad/s), defaults to 2*pi*100. It should be
        well below the sampling frequency to maintain a numerical margin.

    References
    ----------
    .. [#Mey2006] Meyer, Böcker, “Optimum control for interior permanent magnet
       synchronous motors (IPMSM) in constant torque and flux weakening range,” Proc.
       EPE-PEMC, 2006, https://doi.org/10.1109/EPEPEMC.2006.4778413

    .. [#Awa2018] Awan, Song, Saarakkala, Hinkkanen, “Optimal torque control of
       saturated  synchronous motors: Plug-and-play method,” IEEE Trans. Ind. Appl.,
       2018, https://doi.org/10.1109/TIA.2018.2862410

    .. [#Sar2026] Sarén, Hartikainen, Piippo, Hinkkanen, "Decoupled online feedforward
       generation of optimal references for saturated synchronous machine drives,"
       2026, https://arxiv.org/abs/2607.08528

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        i_s_max: float,
        psi_s_min: float | None = None,
        psi_s_max: float = inf,
        k_u: float = 1.0,
        k_mtpv: float = 1.0,
        alpha_ref: float = 2 * pi * 100,
    ) -> None:
        self.par = par
        self.k_u = k_u
        self.k_mtpv = k_mtpv
        self.alpha = alpha_ref

        psi_s_min = par.psi_f if psi_s_min is None else psi_s_min
        self.psi_s_limits = (psi_s_min, psi_s_max)

        # Generate LUTs
        loci = ControlLoci(par)

        # MTPA locus
        mtpa = loci.compute_mtpa_locus(i_s_max)
        self.i_s_mtpa = mtpa.i_s_dq_vs_tau_M
        self.psi_s_mtpa = mtpa.psi_s_abs_vs_tau_M

        # MTPV limit
        mtpv = loci.compute_mtpv_locus(abs(mtpa.psi_s_dq[-1]))
        self.i_s_mtpv = mtpv.i_s_dq_vs_psi_s_abs
        self.tau_M_mtpv = mtpv.tau_M_vs_psi_s_abs

        # Current limit
        gamma1 = phase(loci.compute_mtpv_current(i_s_max))
        gamma2 = phase(mtpa.i_s_dq[-1])
        cl = loci.compute_const_current_locus(i_s_max, (gamma1, gamma2))
        self.i_s_cl = cl.i_s_dq_vs_psi_s_abs
        self.tau_M_cl = cl.tau_M_vs_psi_s_abs

        # Current-reference state, initialized at the zero-torque operating point
        self.i_s_ref = complex(self.i_s_mtpa(0.0))
        if par.psi_f == 0:
            self.i_s_ref = EPS * i_s_max

        self.tau_M_ref = 0.0
        self.psi_s_ref = self.psi_s_limits[0]

    def _evaluate(self, i_s: complex) -> tuple[complex, float]:
        """Flux linkage and torque produced by the given current."""
        psi_s = complex(self.par.psi_s_dq(i_s))
        tau_M = 1.5 * self.par.n_p * (i_s * psi_s.conjugate()).imag
        return psi_s, tau_M

    def _get_max_flux(self, w_m: float, u_dc: float) -> float:
        """Get the maximum available flux linkage."""
        u_s_max = self.k_u * u_dc / sqrt(3)
        return u_s_max / abs(w_m) if w_m != 0 else inf

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
            Mechanical angular speed (rad/s).
        u_dc : float
            DC-link voltage (V).

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

    def compute_current_ref(self, tau_M_ref: float) -> complex:
        """
        Compute the current reference.

        This method is needed only for current-vector control. It returns the current
        reference state. The state is updated in the `update` method.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference (Nm).

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
        # The current reference is a state , driven toward the stored flux and torque
        # references by the tracking law [#Sar2026]_.
        psi_s, tau_M = self._evaluate(self.i_s_ref)
        psi_s_abs = abs(psi_s)

        if psi_s_abs > 0:
            L_s = self.par.incr_ind_mat(self.i_s_ref)
            psi_a = _aux_flux(self.i_s_ref, psi_s, L_s)
            ell = _mat_vec(L_s, psi_s) / psi_s_abs
            den = (psi_a * ell.conjugate()).real
            if den != 0:
                err_tau = (abs(self.tau_M_ref) - tau_M) / (1.5 * self.par.n_p)
                err_psi = self.psi_s_ref - psi_s_abs
                err = (1j * ell * err_tau + psi_a * err_psi) / den
                self.i_s_ref += T_s * self.alpha * err


# %%
class ReferenceGeneratorOnline:
    """
    Optimal feedforward online reference generator for synchronous machines.

    This class tracks online in a feedforward manner the optimal flux, limited torque,
    and current references from a given torque reference [#Sar2026]_. The MTPA locus as
    well as the current, voltage, and MTPV limits are taken into account. This class can
    be used also for a saturated machine model. The current reference is only needed for
    current-vector control.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars
        Machine model parameters.
    i_s_max : float
        Maximum stator current (A).
    psi_s_min : float, optional
        Minimum stator flux (Vs), defaults to `par.psi_f`.
    psi_s_max : float, optional
        Maximum stator flux (Vs), defaults to `inf`.
    k_u : float, optional
        Voltage utilization factor, defaults to 1.
    k_mtpv : float, optional
        MTPV margin, defaults to 1.
    alpha_ref : float, optional
        Bandwidth of the reference tracking (rad/s), defaults to 2*pi*100.

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        i_s_max: float,
        psi_s_min: float | None = None,
        psi_s_max: float = inf,
        k_u: float = 1.0,
        k_mtpv: float = 1.0,
        alpha_ref: float = 2 * pi * 100,
    ) -> None:
        self.par = par
        self.k_u = k_u
        self.k_mtpv = k_mtpv
        self.alpha = alpha_ref

        psi_s_min = par.psi_f if psi_s_min is None else psi_s_min
        self.psi_s_limits = (psi_s_min, psi_s_max)

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
            i_a = _aux_current(psi_s, i_s, L_s)
            return (i_a * psi_s.conjugate()).real

        f0, f1 = mtpv_cond(self.gamma_limits[0]), mtpv_cond(self.gamma_limits[1])
        if f0 * f1 >= 0:
            # No MTPV for this current, fallback
            self.has_mtpv = False
            self.i_s_mtpv = -i_s_max if self.par.psi_f > 0 else EPS * (1 + 1j) * i_s_max
        else:
            self.has_mtpv = True
            res = root_scalar(mtpv_cond, bracket=self.gamma_limits, method="brentq")
            self.i_s_mtpv = complex(i_s_max * exp(1j * res.root))

        # Add small imaginary part to avoid singularity if exactly on d-axis
        if self.i_s_mtpv.imag == 0:
            self.i_s_mtpv += 1j * EPS * i_s_max

        exp_j_gamma_mid = exp(0.5j * sum(self.gamma_limits))
        self.i_s_mtpa = self.i_s_limits[0] * exp(1j * self.gamma_limits[0])
        self.i_s_cl = self.i_s_limits[1] * exp_j_gamma_mid
        # Zero torque at the minimum flux corresponds to pure d-axis current
        self.i_s_ref = complex(self.i_s_limits[0])

        self.tau_M_ref = 0.0
        self.psi_s_ref = self.psi_s_limits[0]

        # Cached from compute_flux_and_torque_refs, reused by the tracker updates
        self._mtpa_state = self._evaluate(self.i_s_mtpa)
        self._mtpv_state = self._evaluate(self.i_s_mtpv)

    def _evaluate(self, i_s: complex) -> tuple[complex, float]:
        """Flux linkage and torque produced by the given current."""
        psi_s = complex(self.par.psi_s_dq(i_s))
        tau_M = 1.5 * self.par.n_p * (i_s * psi_s.conjugate()).imag
        return psi_s, tau_M

    def _get_max_flux(self, w_m: float, u_dc: float) -> float:
        """Get the maximum available flux linkage."""
        u_s_max = self.k_u * u_dc / sqrt(3)
        return u_s_max / abs(w_m) if w_m != 0 else inf

    def _clip_state(self, i_s: complex, max_angle: float | None = None) -> complex:
        """Constrain current magnitude and angle to allowable limits."""
        i_s_mag = clip(abs(i_s), self.i_s_limits[0], inf)
        angle_upper_limit = max_angle if max_angle is not None else self.gamma_limits[1]
        gamma = clip(phase(i_s), self.gamma_limits[0], angle_upper_limit)
        return i_s_mag * exp(1j * gamma)

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
            Mechanical angular speed (rad/s).
        u_dc : float
            DC-link voltage (V).

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

        # Current limit
        tau_M_cl = abs(self._evaluate(self.i_s_cl)[1])
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

    def compute_current_ref(self, tau_M_ref: float) -> complex:
        """Compute current reference."""
        return self.i_s_ref if tau_M_ref >= 0 else self.i_s_ref.conjugate()

    def _update_mtpa(self, T_s: float) -> None:
        """Update the MTPA tracker state."""
        psi_s, tau_M = self._mtpa_state
        if abs(psi_s) > 0:
            L_s = self.par.incr_ind_mat(self.i_s_mtpa)
            L_delta = 0.5 * ((L_s[0, 0] - L_s[1, 1]) + 1j * (L_s[1, 0] + L_s[0, 1]))
            psi_a = _aux_flux(self.i_s_mtpa, psi_s, L_s)
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
            L_psi = _mat_vec(L_s, psi_s)
            ell = L_psi / psi_s_abs
            i_a = L_psi / det_L - self.i_s_mtpv
            j_a = i_a + 2 * (L_delta / det_L) * psi_s.conjugate()
            phi_e = _mat_vec(L_s, j_a)

            den = (ell * phi_e.conjugate()).imag
            if den != 0:
                err_psi = self.psi_s_ref - psi_s_abs
                err_mtpv = (i_a * psi_s.conjugate()).real
                err = 1j * (phi_e * err_psi + ell * err_mtpv) / den
                self.i_s_mtpv += T_s * self.alpha * err

    def _update_lim(self, T_s: float) -> None:
        """Update the current limit tracker state."""
        # Tracks the point on the current limit circle that provides the requested flux
        i_s_cl_abs = abs(self.i_s_cl)
        if i_s_cl_abs > 0:
            self.i_s_cl = self.i_s_cl / i_s_cl_abs * self.i_s_limits[1]
        psi_s, _ = self._evaluate(self.i_s_cl)
        psi_s_abs = abs(psi_s)
        if psi_s_abs > 0:
            L_s = self.par.incr_ind_mat(self.i_s_cl)
            ell = _mat_vec(L_s, psi_s) / psi_s_abs
            den = (ell * self.i_s_cl.conjugate()).imag
            if den != 0:
                err = 1j * self.i_s_cl * (self.psi_s_ref - psi_s_abs) / den
                self.i_s_cl += T_s * self.alpha * err

    def _update_current_tracking(self, T_s: float) -> None:
        """Update the current reference tracker state."""
        # Tracks the current vector that produces the requested torque and flux
        psi_s, tau_M = self._evaluate(self.i_s_ref)
        psi_s_abs = abs(psi_s)
        if psi_s_abs > 0:
            L_s = self.par.incr_ind_mat(self.i_s_ref)
            ell = _mat_vec(L_s, psi_s) / psi_s_abs
            psi_a = _aux_flux(self.i_s_ref, psi_s, L_s)
            den = (psi_a * ell.conjugate()).real
            if den != 0:
                err_tau = (abs(self.tau_M_ref) - tau_M) / (1.5 * self.par.n_p)
                err_psi = self.psi_s_ref - psi_s_abs
                err = (1j * ell * err_tau + psi_a * err_psi) / den
                self.i_s_ref += T_s * self.alpha * err

    def update(self, T_s: float) -> None:
        """
        Update all tracking states.

        Parameters
        ----------
        T_s : float
            Sampling period.

        """
        self._update_mtpa(T_s)
        self.i_s_mtpa = self._clip_state(self.i_s_mtpa)
        if self.has_mtpv:
            self._update_mtpv(T_s)
            self.i_s_mtpv = self._clip_state(self.i_s_mtpv)
            max_angle = phase(self.i_s_mtpv)
        else:
            max_angle = None
        self._update_lim(T_s)
        self.i_s_cl = self._clip_state(self.i_s_cl, max_angle=max_angle)
        self._update_current_tracking(T_s)
