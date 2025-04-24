"""Reference generation for synchronous machine drives."""

from cmath import exp, phase
from math import inf, sqrt

from scipy.optimize import root_scalar

from motulator.common.utils import clip, sign
from motulator.drive.utils._parameters import (
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)
from motulator.drive.utils._sm_control_loci import ControlLoci


# %%
class ReferenceGenerator:
    """
    Optimal reference generator for synchronous machines.

    This class computes the optimal flux, limited torque, and current references from a
    given torque reference. The MTPA locus as well as the current, voltage and MTPV
    limits are taken into account. This class can be used also for a saturated machine
    model. The flux and torque references are computed using pre-computed lookup
    tables [#Mey2006]_, [#Awa2018]_. The current reference is computed using a
    root-finding algorithm (needed only for current-vector control).

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
    max_iter : int, optional
        Max number of iterations for the current reference computation, defaults to 6.

    References
    ----------
    .. [#Mey2006] Meyer, Böcker, “Optimum control for interior permanent magnet
       synchronous motors (IPMSM) in constant torque and flux weakening range,” Proc.
       EPE-PEMC, 2006, https://doi.org/10.1109/EPEPEMC.2006.4778413

    .. [#Awa2018] Awan, Song, Saarakkala, Hinkkanen, “Optimal torque control of
       saturated  synchronous motors: Plug-and-play method,” IEEE Trans. Ind. Appl.,
       2018, https://doi.org/10.1109/TIA.2018.2862410

    """

    def __init__(
        self,
        par: SynchronousMachinePars | SaturatedSynchronousMachinePars,
        i_s_max: float,
        psi_s_min: float | None = None,
        psi_s_max: float = inf,
        k_u: float = 1.0,
        k_mtpv: float = 1.0,
        max_iter: int = 6,
    ) -> None:
        self.par = par
        self.k_u = k_u
        self.k_mtpv = k_mtpv
        self.max_iter = max_iter

        # Set limits
        psi_s_min = par.psi_f if psi_s_min is None else psi_s_min
        self.psi_s_limits = (psi_s_min, psi_s_max)

        # Generate LUTs
        loci = ControlLoci(par)

        # MTPA locus
        mtpa = loci.compute_mtpa_locus(i_s_max)
        self.mtpa_i_s = mtpa.i_s_dq_vs_tau_M

        # MTPV limit
        mtpv = loci.compute_mtpv_locus(abs(mtpa.psi_s_dq[-1]))
        self.mtpv_psi_s = mtpv.psi_s_dq_vs_psi_s_abs

        # Current limit
        gamma1 = phase(loci.compute_mtpv_current(i_s_max))
        gamma2 = phase(mtpa.i_s_dq[-1])
        lim = loci.compute_const_current_locus(i_s_max, (gamma1, gamma2))
        self.lim_i_s = lim.i_s_dq_vs_psi_s_abs

    def _get_mtpa_flux(self, tau_M_ref: float) -> complex:
        """Get the maximum-torque-per-ampere (MTPA) flux linkage."""
        i_s = self.mtpa_i_s(abs(tau_M_ref))
        psi_s = complex(self.par.psi_s_dq(i_s))
        return psi_s

    def _get_mtpv_flux_and_torque(self, psi_s_ref: float) -> tuple[complex, float]:
        """Get the maximum-torque-per-volt (MTPV) references."""
        psi_s = self.mtpv_psi_s(psi_s_ref)
        i_s = complex(self.par.i_s_dq(psi_s))
        tau_M = 1.5 * self.par.n_p * (i_s * psi_s.conjugate()).imag
        return psi_s, tau_M

    def _get_current_limit_torque(self, psi_s_ref: float) -> float:
        """Get torque corresponding to the current limit."""
        i_s = self.lim_i_s(psi_s_ref)
        psi_s = complex(self.par.psi_s_dq(i_s))
        tau_M = 1.5 * self.par.n_p * (i_s * psi_s.conjugate()).imag
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
        mtpa_psi_s = self._get_mtpa_flux(tau_M_ref)
        psi_s_ref = clip(abs(mtpa_psi_s), *self.psi_s_limits)

        # Maximum flux (field weakening)
        psi_s_max = self._get_max_flux(w_m, u_dc)
        psi_s_ref = min(psi_s_ref, psi_s_max)

        # MTPV limit
        mtpv_psi_s, mtpv_tau_M = self._get_mtpv_flux_and_torque(psi_s_ref)
        tau_M_ref = min(self.k_mtpv * mtpv_tau_M, abs(tau_M_ref)) * sign(tau_M_ref)

        # Current limit
        lim_tau_M = self._get_current_limit_torque(psi_s_ref)
        tau_M_ref = min(lim_tau_M, abs(tau_M_ref)) * sign(tau_M_ref)

        # Store for the current reference computation
        self._mtpv_psi_s = mtpv_psi_s

        return psi_s_ref, tau_M_ref

    def compute_current_ref(self, psi_s_ref: float, tau_M_ref: float) -> complex:
        """Compute the current reference."""
        delta_range = (0, phase(self._mtpv_psi_s))

        def error(delta: float) -> float:
            psi_s = psi_s_ref * exp(1j * delta)
            i_s = complex(self.par.i_s_dq(psi_s))
            tau_M = 1.5 * self.par.n_p * (i_s * psi_s.conjugate()).imag
            return tau_M_ref - tau_M

        if error(delta_range[0]) * error(delta_range[1]) >= 0:
            delta = 0.0
        else:
            delta = root_scalar(
                error, bracket=delta_range, method="brentq", maxiter=self.max_iter
            ).root

        psi_s = psi_s_ref * exp(1j * delta)
        i_s = complex(self.par.i_s_dq(psi_s))

        i_s_ref = i_s if tau_M_ref > 0 else i_s.conjugate()  # Set direction

        return i_s_ref
