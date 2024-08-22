"""Current-vector control methods for synchronous machine drives."""

from dataclasses import dataclass, InitVar

import numpy as np

from motulator.drive.control import DriveControlSystem, SpeedController
from motulator.common.control import ComplexPIController
from motulator.drive.utils import SynchronousMachinePars
from motulator.drive.control.sm._common import Observer, ObserverCfg
from motulator.drive.control.sm._torque import TorqueCharacteristics


# %%
class CurrentVectorControl(DriveControlSystem):
    """
    Current vector control for synchronous machine drives.

    This class interconnects the subsystems of the control system and provides
    the interface to the solver.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine model parameters.
    cfg : CurrentReferenceCfg
        Reference generation configuration.
    T_s : float, optional
        Sampling period (s). The default is 250e-6.
    J : float, optional
        Moment of inertia (kgm²). Needed only for the speed controller.
    alpha_c : float, optional
        Current controller bandwidth (rad/s). The default is 2*pi*200.
    alpha_o : float, optional
        Observer bandwidth (rad/s). The default is 2*pi*100.
    sensorless : bool, optional
        If True, sensorless control is used. The default is True.

    Attributes
    ----------
    current_reference : CurrentReference
        Current reference generator.
    observer : Observer | None
        Flux and rotor position observer, used in the sensorless mode only.
    current_ctrl : CurrentController
        Current controller. The default is CurrentController(par, 2*np.pi*200).
    speed_ctrl : SpeedController | None
        Speed controller. The default is SpeedController(par.J, 2*np.pi*4).

    """

    def __init__(
        self,
        par,
        cfg,
        T_s=250e-6,
        J=None,
        alpha_c=2 * np.pi * 200,
        alpha_o=2 * np.pi * 100,
        sensorless=True,
    ):
        super().__init__(par, T_s, sensorless)
        self.current_reference = CurrentReference(par, cfg)
        self.current_ctrl = CurrentController(par, alpha_c)
        if J is not None:
            self.speed_ctrl = SpeedController(J, 2 * np.pi * 4)
        else:
            self.speed_ctrl = None
        if sensorless:
            self.observer = Observer(ObserverCfg(par, sensorless, alpha_o=alpha_o))

    def get_feedback_signals(self, mdl):
        """Override the base class method."""
        fbk = super().get_feedback_signals(mdl)

        if not self.observer:
            # No observer, use the measured values
            fbk.i_s = np.exp(-1j * fbk.theta_m) * fbk.i_ss
            fbk.u_s = np.exp(-1j * fbk.theta_m) * fbk.u_ss
            fbk.w_s = fbk.w_m  # Angular speed of the coordinate system

        return fbk

    def output(self, fbk):
        """Output"""
        ref = super().output(fbk)
        ref = super().get_torque_reference(fbk, ref)
        ref = self.current_reference.output(fbk, ref)
        ref.u_s = self.current_ctrl.output(ref.i_s, fbk.i_s)
        u_ss = ref.u_s * np.exp(1j * fbk.theta_m)
        ref.d_abc = self.pwm(ref.T_s, u_ss, fbk.u_dc, fbk.w_s)

        return ref

    def update(self, fbk, ref):
        """Update"""
        super().update(fbk, ref)
        self.current_reference.update(fbk, ref)
        self.current_ctrl.update(ref.T_s, fbk.u_s, fbk.w_s)


# %%
class CurrentController(ComplexPIController):
    """
    Current controller for synchronous machines.

    This provides an interface of a current controller for synchronous machines
    [#Awa2019a]_. The gains are initialized based on the desired closed-loop
    bandwidth and the inductances.

    Parameters
    ----------
    par : SynchronousMachinePars
        Synchronous machine parameters, should contain `L_d` and `L_q` (H).
    alpha_c : float
        Closed-loop bandwidth (rad/s).

    References
    ----------
    .. [#Awa2019a] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current
       control of saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
       https://doi.org/10.1109/TIA.2019.2919258

    """

    def __init__(self, par, alpha_c):
        k_p, k_i, k_t = 2 * alpha_c, alpha_c**2, alpha_c
        super().__init__(k_p, k_i, k_t)
        self.L_d = par.L_d
        self.L_q = par.L_q

    def output(self, ref_i, i):
        # Extends the base class method by transforming the currents to the
        # flux linkages, which is a simple way to take the saliency into
        # account
        ref_psi = self.L_d * ref_i.real + 1j * self.L_q * ref_i.imag
        psi = self.L_d * i.real + 1j * self.L_q * i.imag

        return super().output(ref_psi, psi)


# %%
# pylint: disable=too-many-instance-attributes
@dataclass
class CurrentReferenceCfg:
    """
    Reference generation configuration.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine model parameters.
    max_i_s : float
        Maximum stator current (A).
    min_psi_s : float, optional
        Minimum stator flux (Vs). The default is `psi_f`.
    nom_w_m : float, optional
        Nominal rotor angular speed (electrical rad/s). Needed if `k_fw` is not
        directly provided.
    alpha_fw : float, optional
        Field-weakening bandwidth (rad/s). The default is 2*pi*20.
    k_fw : float, optional
        Field-weakening gain. The default is `alpha_fw/(w_m_nom*par.L_d)`.
    k_u : float, optional
        Voltage utilization factor. The default is 0.95.

    Attributes
    ----------
    mtpa_i_sd : callable
        MTPA d-axis current (A) as a function of the torque (Nm).
    lim_tau_M : callable
        Torque limit (Nm) as a function of the stator flux linkage (Vs). This
        limit merges the MTPV and current limits.
    lim_i_sd : callable
        d-axis current limit (A) as a function of the stator flux linkage (Vs).
        This limit merges the MTPV and current limits.

    """

    par: InitVar[SynchronousMachinePars]
    max_i_s: float
    min_psi_s: float = None
    nom_w_m: InitVar[float] = None
    alpha_fw: InitVar[float] = 2 * np.pi * 20
    k_fw: float = None
    k_u: float = 0.95

    def __post_init__(self, par, nom_w_m, alpha_fw):
        # Minimum stator flux
        if self.min_psi_s is None:
            self.min_psi_s = par.psi_f
        # Field-weakening gain
        if self.k_fw is None:
            self.k_fw = alpha_fw / (nom_w_m * par.L_d)
        # Generate LUTs
        tq = TorqueCharacteristics(par)
        mtpa = tq.mtpa_locus(self.max_i_s, self.min_psi_s)
        lim = tq.mtpv_and_current_limits(self.max_i_s)
        # MTPA locus
        self.mtpa_i_sd = mtpa.i_sd_vs_tau_M
        # Merged MTPV and current limits
        self.lim_tau_M = lim.tau_M_vs_abs_psi_s
        self.lim_i_sd = lim.i_sd_vs_tau_M


# %%
class CurrentReference:
    """
    Current reference calculation.

    This method includes the MTPA locus and field-weakening operation based on
    the unlimited voltage reference feedback. The MTPV and current limits are
    taken into account. This resembles the method presented [#Bed2020]_.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine model parameters.
    cfg : CurrentReferenceCfg
        Reference generation configuration.

    Notes
    -----
    Instead of the PI controller used in [#Bed2020]_, we use a simpler integral
    controller with a constant gain. The resulting operating-point-dependent
    closed-loop pole could be derived using (12) of the paper. Unlike in
    [#Bed2020]_, the MTPV limit is also included here by means of limiting the
    reference torque and the d-axis current reference.

    References
    ----------
    .. [#Bed2020] Bedetti, Calligaro, Petrella, "Analytical design and
       autotuning of adaptive flux-weakening voltage regulation loop in IPMSM
       drives with accurate torque regulation," IEEE Trans. Ind. Appl., 2020,
       https://doi.org/10.1109/TIA.2019.2942807

    """

    def __init__(self, par, cfg):
        self.par, self.cfg = par, cfg
        self.ref_i_sd = 0  # State

    def output(self, fbk, ref):
        """Compute the stator current reference."""
        par, cfg = self.par, self.cfg

        def limit_torque(ref, fbk):
            if np.abs(fbk.w_m) > 0:
                max_psi_s = cfg.k_u * fbk.u_dc / np.sqrt(3) / np.abs(fbk.w_m)
                max_tau_M = cfg.lim_tau_M(max_psi_s)
            else:
                max_tau_M = cfg.lim_tau_M(np.inf)

            if np.abs(ref.tau_M) > max_tau_M:
                ref.tau_M = np.sign(ref.tau_M) * max_tau_M

            return ref.tau_M

        # Limit the torque reference according to MTPV and current limits
        ref.tau_M = limit_torque(ref, fbk)

        # q-axis current reference
        psi_t = par.psi_f + (par.L_d - par.L_q) * self.ref_i_sd
        ref_i_sq = ref.tau_M / (1.5 * par.n_p * psi_t) if psi_t != 0 else 0

        # Limit the q-axis current reference
        mtpa_i_sd = cfg.mtpa_i_sd(np.abs(ref.tau_M))
        max_i_sq = np.min(
            [
                np.sqrt(cfg.max_i_s**2 - self.ref_i_sd**2),
                np.sqrt(cfg.max_i_s**2 - mtpa_i_sd**2),
            ]
        )
        if np.abs(ref_i_sq) > max_i_sq:
            ref_i_sq = np.sign(ref_i_sq) * max_i_sq

        # Current reference
        ref.i_s = self.ref_i_sd + 1j * ref_i_sq

        # Limited torque (for the speed controller)
        # ref.tau_M_lim = 1.5*par.n_p*psi_t*ref_i_sq
        ref.tau_M = 1.5 * par.n_p * psi_t * ref_i_sq

        return ref

    def update(self, fbk, ref):
        """Field-weakening control based on the unlimited reference voltage."""
        cfg = self.cfg
        max_u_s = cfg.k_u * fbk.u_dc / np.sqrt(3)
        self.ref_i_sd += ref.T_s * cfg.k_fw * (max_u_s - np.abs(ref.u_s))

        # Limit the current
        mtpa_i_sd = cfg.mtpa_i_sd(np.abs(ref.tau_M))
        lim_i_sd = cfg.lim_i_sd(np.abs(ref.tau_M))
        self.ref_i_sd = np.clip(self.ref_i_sd, lim_i_sd, mtpa_i_sd)
