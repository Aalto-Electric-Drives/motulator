"""Flux-vector control of synchronous machine drives."""

from dataclasses import dataclass, InitVar
import numpy as np
from motulator.control._common import SpeedCtrl, DriveCtrl
from motulator.control.sm._torque import TorqueCharacteristics
from motulator.control.sm._common import ModelPars, Observer, ObserverCfg


# %%
class FluxVectorCtrl(DriveCtrl):
    """
    Flux-vector control of synchronous machine drives.

    This class implements a variant of flux-vector control [#Pel2009]_. Rotor 
    coordinates as well as decoupling between the stator flux and torque 
    channels are used according to [#Awa2019b]_. Here, the stator flux 
    magnitude and the electromagnetic torque are selected as controllable 
    variables. 

    Notes
    -----
    Proportional controllers are used for simplicity. The magnetic saturation 
    is not considered in this implementation.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    ref : FluxTorqueReferenceCfg
        Reference generation parameters.
    alpha_psi : float, optional
        Bandwidth of the flux controller (rad/s). The default is 2*pi*100.
    alpha_tau : float, optional
        Bandwidth of the torque controller (rad/s). The default is 2*pi*200.
    T_s : float
        Sampling period (s). The default is 250e-6.
    sensorless : bool, optional
        If True, sensorless control is used. The default is True.

    Attributes
    ----------
    observer : Observer
        Flux observer, having both sensorless and sensored modes.
    flux_torque_ref : FluxTorqueReference
        Flux and torque reference generator.
    speed_ctrl : SpeedCtrl
        Speed controller.
    w_m_ref : callable
        Speed reference (electrical rad/s) as a function of time (s).
    pwm : PWM
        Pulse-width modulation.

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
            ref_par,
            alpha_psi=2*np.pi*100,
            alpha_tau=2*np.pi*200,
            alpha_o=2*np.pi*100,
            T_s=250e-6,
            sensorless=True):
        super().__init__(par, T_s, sensorless)
        # Subsystems
        self.flux_torque_ref = FluxTorqueReference(ref_par)
        self.speed_ctrl = SpeedCtrl(par.J, 2*np.pi*4)
        self.observer = Observer(
            ObserverCfg(par, alpha_o=alpha_o, sensorless=sensorless))
        # Bandwidths
        self.alpha_psi = alpha_psi
        self.alpha_tau = alpha_tau

    # pylint: disable=too-many-locals
    def output(self, fbk):
        """Calculate references."""

        par = self.par

        ref = super().output(fbk)
        ref = super().get_torque_reference(fbk, ref)

        ref.psi_s, ref.tau_M_lim = self.flux_torque_ref(
            ref.tau_M, fbk.w_m, fbk.u_dc)

        # Flux and torque estimates
        tau_M = 1.5*par.n_p*np.imag(fbk.i_s*np.conj(fbk.psi_s))

        # Auxiliary current
        i_a = fbk.psi_s.real/par.L_q + 1j*fbk.psi_s.imag/par.L_d - fbk.i_s

        # Torque-production factor, c_tau = 0 corresponds to the MTPV condition
        c_tau = 1.5*par.n_p*np.real(i_a*np.conj(fbk.psi_s))

        # References for the flux and torque controllers
        v_psi = self.alpha_psi*(ref.psi_s - np.abs(fbk.psi_s))
        v_tau = self.alpha_tau*(ref.tau_M_lim - tau_M)
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
    par : ModelPars
        Machine model parameters.
    i_s_max : float
        Maximum stator current (A). 
    psi_s_min : float, optional
        Minimum stator flux (Vs). The default is `psi_f`.
    psi_s_max : float, optional
        Maximum stator flux (Vs). The default is inf.
    k_u : float, optional
        Voltage utilization factor. The default is 0.95.

    Attributes
    ----------
    psi_s_mtpa : callable
        MTPA stator flux linkage (Vs) as a function of the torque (Nm).
    tau_M_lim : callable
        Torque limit (Nm) as a function of the stator flux linkage (Vs). This 
        limit merges the MTPV and current limits. 

    """
    par: InitVar[ModelPars]
    i_s_max: float = None
    psi_s_min: float = None
    psi_s_max: float = np.inf
    k_u: float = .95

    def __post_init__(self, par):
        self.psi_s_min = par.psi_f if self.psi_s_min is None else self.psi_s_min
        # Generate LUTs
        tq = TorqueCharacteristics(par)
        mtpa = tq.mtpa_locus(self.i_s_max, self.psi_s_min)
        lim = tq.mtpv_and_current_limits(self.i_s_max)
        # MTPA locus
        self.psi_s_mtpa = mtpa.abs_psi_s_vs_tau_M
        # Merged MTPV and current limits
        self.tau_M_lim = lim.tau_M_vs_abs_psi_s


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
        self.psi_s_min, self.psi_s_max = cfg.psi_s_min, cfg.psi_s_max
        self.k_u = cfg.k_u
        # Merged MTPV and current limits
        self.tau_M_lim = cfg.tau_M_lim
        # MTPA locus
        self.psi_s_mtpa = cfg.psi_s_mtpa

    def __call__(self, tau_M_ref, w_m, u_dc):
        """
        Calculate the stator flux reference and limit the torque reference.

        Parameters
        ----------
        tau_M_ref : float
            Unlimited torque reference (Nm).
        w_m : float
            Rotor speed or its reference (electrical rad/s).
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        psi_s_ref : float
            Stator flux reference (Vs).
        tau_M_ref_lim : float
            Limited torque reference (Nm).

        """
        # Get the MTPA flux
        psi_s_mtpa = self.psi_s_mtpa(np.abs(tau_M_ref))
        psi_s_mtpa = np.clip(psi_s_mtpa, self.psi_s_min, self.psi_s_max)

        # Field weakening
        u_s_max = self.k_u*u_dc/np.sqrt(3)
        psi_s_max = u_s_max/np.abs(w_m) if np.abs(w_m) > 0 else np.inf

        # Flux reference
        psi_s_ref = np.min([psi_s_max, psi_s_mtpa])

        # Limit the torque reference according to the MTPV and current limits
        tau_M_lim = self.tau_M_lim(psi_s_ref)
        tau_M_ref_lim = np.min([tau_M_lim, np.abs(tau_M_ref)
                                ])*np.sign(tau_M_ref)

        return psi_s_ref, tau_M_ref_lim
