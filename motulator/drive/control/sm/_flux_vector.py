"""Flux-vector control of synchronous machine drives."""

from dataclasses import dataclass, InitVar

import numpy as np

from motulator.drive.control import DriveCtrl, SpeedCtrl
from motulator.drive.utils import SynchronousMachinePars
from motulator.drive.control.sm._common import Observer, ObserverCfg
from motulator.drive.control.sm._torque import TorqueCharacteristics


# %%
class FluxVectorCtrl(DriveCtrl):
    """
    Flux-vector control of synchronous machine drives.

    This class implements a variant of flux-vector control [#Pel2009]_. Rotor 
    coordinates as well as decoupling between the stator flux and torque 
    channels are used according to [#Awa2019b]_. Here, the stator flux 
    magnitude and the electromagnetic torque are selected as controllable 
    variables. Proportional controllers are used for simplicity. The magnetic 
    saturation is not considered in this implementation.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine model parameters.
    cfg : FluxTorqueReferenceCfg
        Reference generation configuration.
    alpha_psi : float, optional
        Bandwidth of the flux controller (rad/s). The default is 2*pi*100.
    alpha_tau : float, optional
        Bandwidth of the torque controller (rad/s). The default is 2*pi*200.
    alpha_o : float, optional
        Observer bandwidth (rad/s). The default is 2*pi*100.
    J : float, optional
        Moment of inertia (kgm²). Needed only for the speed controller. 
    T_s : float
        Sampling period (s). The default is 250e-6.
    sensorless : bool, optional
        If True, sensorless control is used. The default is True.

    References
    ----------
    .. [#Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented 
       control of IPM drives with variable DC link in the field-weakening 
       region,” IEEE Trans.Ind. Appl., 2009, 
       https://doi.org/10.1109/TIA.2009.2027167

    .. [#Awa2019b] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented 
       control of synchronous motors: A systematic design procedure," IEEE 
       Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

    """

    def __init__(
            self,
            par,
            cfg,
            alpha_psi=2*np.pi*100,
            alpha_tau=2*np.pi*200,
            alpha_o=2*np.pi*100,
            J=None,
            T_s=250e-6,
            sensorless=True):
        super().__init__(par, T_s, sensorless)
        # Subsystems
        self.flux_torque_reference = FluxTorqueReference(cfg)
        if J is not None:
            self.speed_ctrl = SpeedCtrl(J, 2*np.pi*4)
        else:
            self.speed_ctrl = None
        self.observer = Observer(
            ObserverCfg(par, alpha_o=alpha_o, sensorless=sensorless))
        # Bandwidths
        self.alpha_psi = alpha_psi
        self.alpha_tau = alpha_tau

    def output(self, fbk):
        """Calculate references."""
        par = self.par

        # Get the references from the outer loop
        ref = super().output(fbk)
        ref = super().get_torque_reference(fbk, ref)
        ref = self.flux_torque_reference(fbk, ref)

        # Flux and torque estimates
        tau_M = 1.5*par.n_p*np.imag(fbk.i_s*np.conj(fbk.psi_s))

        # Auxiliary current
        i_a = fbk.psi_s.real/par.L_q + 1j*fbk.psi_s.imag/par.L_d - fbk.i_s

        # Torque-production factor, c_tau = 0 corresponds to the MTPV condition
        c_tau = 1.5*par.n_p*np.real(i_a*np.conj(fbk.psi_s))

        # References for the flux and torque controllers
        v_psi = self.alpha_psi*(ref.psi_s - np.abs(fbk.psi_s))
        v_tau = self.alpha_tau*(ref.tau_M - tau_M)
        if c_tau > 0:
            v = (
                1.5*par.n_p*np.abs(fbk.psi_s)*i_a*v_psi +
                1j*fbk.psi_s*v_tau)/c_tau
        else:
            v = v_psi

        # Stator voltage reference
        ref.u_s = par.R_s*fbk.i_s + 1j*fbk.w_m*fbk.psi_s + v
        u_ss = ref.u_s*np.exp(1j*fbk.theta_m)

        ref.d_abc = self.pwm(ref.T_s, u_ss, fbk.u_dc, fbk.w_s)

        return ref


# %%
@dataclass
class FluxTorqueReferenceCfg:
    """
    Reference generation configuration.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine model parameters.
    max_i_s : float
        Maximum stator current (A). 
    min_psi_s : float, optional
        Minimum stator flux (Vs). The default is `par.psi_f`.
    max_psi_s : float, optional
        Maximum stator flux (Vs). The default is inf.
    k_u : float, optional
        Voltage utilization factor. The default is 0.95.

    """
    par: InitVar[SynchronousMachinePars]
    max_i_s: float = None
    min_psi_s: float = None
    max_psi_s: float = np.inf
    k_u: float = .95

    def __post_init__(self, par):
        self.min_psi_s = (
            par.psi_f if self.min_psi_s is None else self.min_psi_s)
        # Generate LUTs
        tq = TorqueCharacteristics(par)
        mtpa = tq.mtpa_locus(self.max_i_s, self.min_psi_s)
        lim = tq.mtpv_and_current_limits(self.max_i_s)
        # MTPA locus
        self.mtpa_psi_s = mtpa.abs_psi_s_vs_tau_M
        # Merged MTPV and current limits
        self.lim_tau_M = lim.tau_M_vs_abs_psi_s


# %%
class FluxTorqueReference:
    """
    Flux and torque references.

    The current and MTPV limits as well as the MTPA locus are implemented as
    look-up tables, which are generated based on the constant machine model
    parameters.

    Parameters
    ----------
    cfg : FluxTorqueReferenceCfg
        Reference generation configuration.

    """

    def __init__(self, cfg):
        self.cfg = cfg

    def __call__(self, fbk, ref):
        """Compute the stator flux reference and limit the torque reference."""
        cfg = self.cfg

        # Get the MTPA flux
        mtpa_psi_s = cfg.mtpa_psi_s(np.abs(ref.tau_M))
        mtpa_psi_s = np.clip(mtpa_psi_s, cfg.min_psi_s, cfg.max_psi_s)

        # Field weakening
        w_m = fbk.w_m if hasattr(fbk, 'w_m') else ref.w_m  # For V/Hz control
        max_u_s = cfg.k_u*fbk.u_dc/np.sqrt(3)
        max_psi_s = max_u_s/np.abs(w_m) if w_m != 0 else np.inf

        # Flux reference
        ref.psi_s = np.min([max_psi_s, mtpa_psi_s])

        # Limit the torque reference according to the MTPV and current limits
        lim_tau_M = cfg.lim_tau_M(ref.psi_s)
        ref.tau_M = np.min([lim_tau_M, np.abs(ref.tau_M)])*np.sign(ref.tau_M)

        return ref
