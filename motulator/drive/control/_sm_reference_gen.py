"""Reference generation for synchronous machine drives."""

from cmath import phase
from math import inf, pi, sqrt

from motulator.common.utils._utils import clip, sign
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
        Bandwidth of the current-reference tracking (rad/s), defaults to 2*pi*400. It
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
        alpha_cur: float = 2 * pi * 400,
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
