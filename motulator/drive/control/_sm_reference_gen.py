"""Reference generation for synchronous machine drives."""

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
class ReferenceGenerator:
    """
    Optimal reference generator for synchronous machines.

    This class computes the optimal flux, limited torque, and current references from a
    given torque reference. The MTPA locus as well as the current, voltage and MTPV
    limits are taken into account. This class can be used also for a saturated machine
    model. The flux and torque references are computed using pre-computed lookup
    tables [#Mey2006]_, [#Awa2018]_. The current reference is generated dynamically
    using a tracking law [#Sar2026]_, needed only for current-vector control.

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
    alpha_cur : float, optional
        Bandwidth of the current-reference tracking (rad/s), defaults to 2*pi*200. It
        should be well below the sampling frequency to maintain a numerical margin.

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
        alpha_cur: float = 2 * pi * 200,
    ) -> None:
        self.par = par
        self.k_u = k_u
        self.k_mtpv = k_mtpv
        self.alpha_cur = alpha_cur

        # Set limits
        psi_s_min = par.psi_f if psi_s_min is None else psi_s_min
        self.psi_s_limits = (psi_s_min, psi_s_max)

        # Generate LUTs
        loci = ControlLoci(par)

        # MTPA locus
        mtpa = loci.compute_mtpa_locus(i_s_max)
        self.i_s_mtpa = mtpa.i_s_dq_vs_tau_M

        # MTPV limit
        mtpv = loci.compute_mtpv_locus(abs(mtpa.psi_s_dq[-1]))
        self.i_s_mtpv = mtpv.i_s_dq_vs_psi_s_abs

        # Current limit
        gamma1 = phase(loci.compute_mtpv_current(i_s_max))
        gamma2 = phase(mtpa.i_s_dq[-1])
        cl = loci.compute_const_current_locus(i_s_max, (gamma1, gamma2))
        self.i_s_cl = cl.i_s_dq_vs_psi_s_abs

        # Current-reference state, initialized at the zero-torque operating point
        self.i_s_ref = complex(self.i_s_mtpa(0.0))
        if par.psi_f == 0:
            self.i_s_ref = 1e-3 * i_s_max

    def _evaluate(self, i_s: complex) -> tuple[complex, float]:
        """Flux linkage and torque produced by the given current."""
        psi_s = complex(self.par.psi_s_dq(i_s))
        tau_M = 1.5 * self.par.n_p * (i_s * psi_s.conjugate()).imag
        return psi_s, tau_M

    def _get_mtpa_flux(self, tau_M_ref: float) -> float:
        """Get the maximum-torque-per-ampere (MTPA) flux magnitude."""
        psi_s, _ = self._evaluate(self.i_s_mtpa(abs(tau_M_ref)))
        return abs(psi_s)

    def _get_mtpv_torque(self, psi_s_abs_ref: float) -> float:
        """Get the maximum-torque-per-volt (MTPV) torque limit."""
        _, tau_M = self._evaluate(self.i_s_mtpv(psi_s_abs_ref))
        return tau_M

    def _get_current_limit_torque(self, psi_s_abs_ref: float) -> float:
        """Get torque corresponding to the current limit."""
        _, tau_M = self._evaluate(self.i_s_cl(psi_s_abs_ref))
        return tau_M

    def _get_max_flux(self, w_m: float, u_dc: float) -> float:
        """Get the maximum available flux linkage."""
        u_s_max = self.k_u * u_dc / sqrt(3)
        psi_s_max = u_s_max / abs(w_m) if w_m != 0 else inf
        return psi_s_max

    def compute_flux_and_torque_refs(
        self, tau_M_ref: float, w_m: float, u_dc: float
    ) -> tuple[float, float]:
        """Compute the flux and torque reference signals."""
        # MTPA flux
        psi_s_abs_ref = clip(self._get_mtpa_flux(tau_M_ref), *self.psi_s_limits)

        # Maximum flux (field weakening)
        psi_s_abs_ref = min(psi_s_abs_ref, self._get_max_flux(w_m, u_dc))

        # Current limit
        tau_M_cl = self._get_current_limit_torque(psi_s_abs_ref)
        tau_M_ref = min(tau_M_cl, abs(tau_M_ref)) * sign(tau_M_ref)

        # MTPV limit
        tau_M_mtpv = self._get_mtpv_torque(psi_s_abs_ref)
        if tau_M_mtpv > 0:
            tau_M_ref = min(self.k_mtpv * tau_M_mtpv, abs(tau_M_ref)) * sign(tau_M_ref)

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

    def update(self, T_s: float, psi_s_abs_ref: float, tau_M_ref: float) -> None:
        """
        Update the current-reference state.

        The current reference is a state variable, driven toward the given flux and
        torque references by the tracking law with the bandwidth `alpha_cur`
        [#Sar2026]_.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        psi_s_abs_ref : float
            Stator flux reference (Vs).
        tau_M_ref : float
            Torque reference (Nm).

        """
        psi_s, tau_M = self._evaluate(self.i_s_ref)
        psi_s_abs = abs(psi_s)

        if psi_s_abs > 0:
            L_s = self.par.incr_ind_mat(self.i_s_ref)
            ell = complex(*(L_s @ [psi_s.real, psi_s.imag])) / psi_s_abs
            psi_a = complex(self.par.aux_flux(self.i_s_ref))
            den = (psi_a * ell.conjugate()).real
            if den != 0:
                d_i_s = (
                    1j * ell * (abs(tau_M_ref) - tau_M) / (1.5 * self.par.n_p)
                    + psi_a * (psi_s_abs_ref - psi_s_abs)
                ) / den
                self.i_s_ref += T_s * self.alpha_cur * d_i_s


class ReferenceGeneratorOnline:
    """
    Optimal onlinereference generator for synchronous machines.

    This class computes the optimal flux, limited torque, and current references from a
    given torque reference. The MTPA locus as well as the current, voltage and MTPV
    limits are taken into account. This class can be used also for a saturated machine
    model. Optimal references are tracked online using tracking laws [#Sar2026]_.
    The current reference is only needed for current-vector control.

        References
    ----------
    .. [#Mey2006] Meyer, Böcker, “Optimum control for interior permanent magnet
       synchronous motors (IPMSM) in constant torque and flux weakening range,” Proc.
       EPE-PEMC, 2006, https://doi.org/10.1109/EPEPEMC.2006.4778413

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
        current_ref: bool | None = None,
    ) -> None:
        self.par = par
        self.k_u = k_u
        self.k_mtpv = k_mtpv
        self.alpha = alpha_ref
        self.current_ref = current_ref

        psi_s_min = par.psi_f if psi_s_min is None else psi_s_min
        self.psi_s_limits = (psi_s_min, psi_s_max)

        # Dynamic tracking states
        if self.par.psi_f == 0 and psi_s_min is not None:  # SyRM
            self.gamma_limits = (0.25 * pi, 0.5 * pi)
            self.delta_limits = (0.25 * pi, 0.5 * pi)
            self.i_s_limits = (
                abs(complex(self.par.iterate_i_s_dq(psi_s_min))),
                i_s_max,
            )

        else:  # With PMs
            self.gamma_limits = (0.5 * pi, pi)
            self.delta_limits = (0.5 * pi, 0.75 * pi)
            self.i_s_limits = (0, i_s_max)

        def mtpv_cond(gamma: float) -> float:
            i_s_dq = i_s_max * exp(1j * gamma)
            psi_s_dq = complex(self.par.psi_s_dq(i_s_dq))
            i_a_dq = complex(self.par.aux_current(i_s_dq))
            return (i_a_dq * psi_s_dq.conjugate()).real

        if mtpv_cond(self.gamma_limits[0]) * mtpv_cond(self.gamma_limits[1]) >= 0:
            # No MTPV for this current, fallback
            self.i_s_mtpv = -i_s_max + 1e-3j if self.par.psi_f > 0 else 1e-3 + 1e-3j
        else:
            gamma_root = root_scalar(
                mtpv_cond, bracket=self.gamma_limits, method="brentq"
            ).root
            self.i_s_mtpv = complex(i_s_max * exp(1j * gamma_root))

        # Add small imaginary part to avoid singularity if exactly on d-axis
        if self.i_s_mtpv.imag == 0:
            self.i_s_mtpv += 1e-3j

        self.i_s_mtpa = self.i_s_limits[0] * exp(1j * self.gamma_limits[0])
        self.i_s_cl = self.i_s_limits[1] * exp(
            1j * (self.gamma_limits[0] + self.gamma_limits[1]) / 2
        )
        self.i_s_ref = self.i_s_limits[0] * exp(
            1j * (self.gamma_limits[0] + self.gamma_limits[1]) / 2
        )

        self.tau_M_ref = 0.0
        self.psi_s_ref = self.psi_s_limits[0]

    def _evaluate(self, i_s: complex) -> tuple[complex, float]:
        """Flux linkage and torque produced by the given current."""
        psi_s = complex(self.par.psi_s_dq(i_s))
        tau_M = 1.5 * self.par.n_p * (i_s * psi_s.conjugate()).imag
        return psi_s, tau_M

    def _get_mtpa_flux(self, i_s_mtpa: complex) -> float:
        """Get the maximum-torque-per-ampere (MTPA) flux magnitude."""
        psi_s = complex(self.par.psi_s_dq(i_s_mtpa))
        return abs(psi_s)

    def _get_mtpv_torque(self, i_s_mtpv: complex) -> float:
        """Get the maximum-torque-per-volt (MTPV) torque limit."""
        _, tau_M = self._evaluate(i_s_mtpv)
        return abs(tau_M)

    def _get_max_flux(self, w_m: float, u_dc: float) -> float:
        """Get the maximum available flux linkage."""
        u_s_max = self.k_u * u_dc / sqrt(3)
        psi_s_max = u_s_max / abs(w_m) if w_m != 0 else inf
        return abs(psi_s_max)

    def _get_current_limit_torque(self, i_s_cl: complex) -> float:
        """Get torque corresponding to the current limit."""
        _, tau_M = self._evaluate(i_s_cl)
        return abs(tau_M)

    def _clip_state(self, i_s: complex, max_angle: float | None = None) -> complex:
        """Constrain current magnitude and angle to allowable limits."""
        i_s_mag = clip(abs(i_s), self.i_s_limits[0], inf)
        angle_upper_limit = max_angle if max_angle is not None else self.gamma_limits[1]
        gamma = clip(phase(i_s), self.gamma_limits[0], angle_upper_limit)
        return i_s_mag * exp(1j * gamma)

    def compute_flux_and_torque_refs(
        self, tau_M_ref: float, w_m: float, u_dc: float
    ) -> tuple[float, float]:
        """Compute flux and torque references using tracked parameters."""
        # MTPA flux
        psi_s_mtpa = complex(self._get_mtpa_flux(self.i_s_mtpa))
        psi_s_abs_ref = clip(abs(psi_s_mtpa), *self.psi_s_limits)

        # Maximum flux (field weakening)
        psi_s_abs_ref = min(psi_s_abs_ref, self._get_max_flux(w_m, u_dc))

        # Current limit
        tau_M_cl = self._get_current_limit_torque(self.i_s_cl)
        tau_M_ref = min(tau_M_cl, abs(tau_M_ref)) * sign(tau_M_ref)

        # MTPV limit
        tau_M_mtpv = self._get_mtpv_torque(self.i_s_mtpv)
        if tau_M_mtpv > 0:
            tau_M_ref = min(self.k_mtpv * tau_M_mtpv, abs(tau_M_ref)) * sign(tau_M_ref)

        self.psi_s_ref = psi_s_abs_ref
        self.tau_M_ref = tau_M_ref
        return psi_s_abs_ref, tau_M_ref

    def compute_current_ref(self, tau_M_ref: float) -> complex:
        """Compute current reference."""
        return self.i_s_ref if tau_M_ref >= 0 else self.i_s_ref.conjugate()

    def update(self, T_s: float, *_: float) -> None:
        """Update all tracking states."""
        self.update_mtpa(T_s)
        self.i_s_mtpa = self._clip_state(self.i_s_mtpa)
        self.update_mtpv(T_s)
        self.i_s_mtpv = self._clip_state(self.i_s_mtpv)
        self.update_lim(T_s)
        self.i_s_cl = self._clip_state(self.i_s_cl, max_angle=phase(self.i_s_mtpv))
        if self.current_ref:
            self.update_current_tracking(T_s)

    def update_mtpa(self, T_s: float) -> None:
        """Update the MTPA tracker state (Sigma_mtpa) using exact linearization."""

        psi_s, tau_M = self._evaluate(self.i_s_mtpa)
        psi_s_abs = abs(psi_s)
        if psi_s_abs > EPS:
            L_s = self.par.incr_ind_mat(self.i_s_mtpa)  # 2x2 real array
            L_delta = (L_s[0, 0] - L_s[1, 1]) / 2 + 1j * (L_s[1, 0] + L_s[0, 1]) / 2
            psi_a = complex(self.par.aux_flux(self.i_s_mtpa))
            phia = psi_a + 2 * L_delta * self.i_s_mtpa.conjugate()

            dir_tau = (
                1j * phia
            )  # Movement here changes torque, keeps MTPA condition error constant
            dir_mtpa = (
                -1 * psi_a
            )  # Movement here changes MTPA condition error, keeps torque constant

            den = (psi_a * phia.conjugate()).real
            if abs(den) > EPS:
                gamma = (psi_a * self.i_s_mtpa.conjugate()).real  # mtpa condition error
                torque_error_mag = (abs(self.tau_M_ref) - tau_M) / (1.5 * self.par.n_p)
                d_i_mtpa = (torque_error_mag * dir_tau + gamma * dir_mtpa) / den
                self.i_s_mtpa += T_s * self.alpha * d_i_mtpa

    def update_mtpv(self, T_s: float) -> None:
        """
        Update the MTPV tracker state (Sigma_mtpv) using exact linearization.
        Tracks the current vector that satisfies the MTPV condition
        for a given flux reference.
        """
        psi_s, _ = self._evaluate(self.i_s_mtpv)
        psi_s_abs = abs(psi_s)
        if psi_s_abs > EPS:
            L_s = self.par.incr_ind_mat(self.i_s_mtpv)
            L_sigma = (L_s[0, 0] + L_s[1, 1]) / 2
            L_delta = (L_s[0, 0] - L_s[1, 1]) / 2 + 1j * (L_s[1, 0] + L_s[0, 1]) / 2
            G_s = np.linalg.inv(L_s)
            G_delta = (G_s[0, 0] - G_s[1, 1]) / 2 + 1j * (G_s[1, 0] + G_s[0, 1]) / 2

            i_a = complex(self.par.aux_current(self.i_s_mtpv))
            psi_dir = psi_s / psi_s_abs
            ell = L_sigma * psi_dir + L_delta * psi_dir.conjugate()
            j_a = i_a - 2 * G_delta * psi_s.conjugate()
            phi_e = L_sigma * j_a + L_delta * j_a.conjugate()

            dir_flux = (
                1j * phi_e
            )  # Movement here changes flux, keeps MTPV condition error constant
            dir_mtpv = (
                1j * ell
            )  # Movement here changes MTPV condition error, keeps flux constant

            den = (ell * phi_e.conjugate()).imag
            if den > EPS:
                flux_error = self.psi_s_ref - psi_s_abs
                c_delta = (i_a * psi_s.conjugate()).real
                d_i_mtpv = (flux_error * dir_flux + c_delta * dir_mtpv) / den
                self.i_s_mtpv += T_s * self.alpha * d_i_mtpv

    def update_lim(self, T_s: float) -> None:
        """
        Update the Current Limit tracker state (Sigma_lim) using exact linearization.
        Tracks the point on the current limit circle that provides the requested flux.
        """
        # Forcing the current limit to be on the circle, but allowing the angle to
        # change to track the flux reference
        if abs(self.i_s_cl) > 0:
            self.i_s_cl = (self.i_s_cl / abs(self.i_s_cl)) * self.i_s_limits[1]

        psi_s, _ = self._evaluate(self.i_s_cl)
        psi_s_abs = abs(psi_s)

        if psi_s_abs > EPS:
            L_s = self.par.incr_ind_mat(self.i_s_cl)  # 2x2 real array
            L_sigma = (L_s[0, 0] + L_s[1, 1]) / 2
            L_delta = (L_s[0, 0] - L_s[1, 1]) / 2 + 1j * (L_s[1, 0] + L_s[0, 1]) / 2

            psi_dir = psi_s / psi_s_abs
            ell = L_sigma * psi_dir + L_delta * psi_dir.conjugate()

            dir_lim = (
                1j * self.i_s_cl
            )  # Movement here changes the current limit, keeps flux constant

            den = (ell * dir_lim.conjugate()).real
            if abs(den) > EPS:
                d_i_cl = (self.psi_s_ref - psi_s_abs) * dir_lim / den
                self.i_s_cl += T_s * self.alpha * d_i_cl

    def update_current_tracking(self, T_s: float) -> None:
        """
        Update the Current Reference tracker state (Sigma_cur) using
        exact linearization. Tracks the specific current vector i_s that produces
        the requested torque and flux.
        """
        psi_s, tau_M = self._evaluate(self.i_s_ref)
        psi_s_abs = abs(psi_s)

        if psi_s_abs > EPS:
            L_s = self.par.incr_ind_mat(self.i_s_ref)  # 2x2 real array
            L_sigma = (L_s[0, 0] + L_s[1, 1]) / 2
            L_delta = (L_s[0, 0] - L_s[1, 1]) / 2 + 1j * (L_s[1, 0] + L_s[0, 1]) / 2

            psi_a = complex(self.par.aux_flux(self.i_s_ref))
            psi_dir = psi_s / psi_s_abs
            ell = L_sigma * psi_dir + L_delta * psi_dir.conjugate()

            dir_tau = 1j * ell  # Movement here changes torque, keeps flux constant
            dir_flux = -psi_a  # Movement here changes flux, keeps torque constant

            den = (psi_a * ell.conjugate()).real
            if abs(den) > EPS:
                torque_error_mag = (abs(self.tau_M_ref) - tau_M) / (1.5 * self.par.n_p)
                flux_error = self.psi_s_ref - psi_s_abs

                d_i_ref = (torque_error_mag * dir_tau - flux_error * dir_flux) / den

                self.i_s_ref += T_s * self.alpha * d_i_ref
