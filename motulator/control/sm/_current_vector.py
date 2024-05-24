"""Current vector control methods for synchronous machine drives."""

from dataclasses import dataclass, InitVar
import numpy as np
from motulator.control._common import ComplexPICtrl, SpeedCtrl, DriveCtrl
from motulator.control.sm._torque import TorqueCharacteristics
from motulator.control.sm._common import ModelPars, Observer, ObserverCfg


# %%
class CurrentVectorCtrl(DriveCtrl):
    """
    Current vector control for synchronous machine drives.

    This class interconnects the subsystems of the control system and provides 
    the interface to the solver.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    ref : ReferencePars
        Reference generation parameters.
    T_s : float, optional
        Sampling period (s). The default is 250e-6.
    sensorless : bool, optional
        If True, sensorless control is used. The default is True.

    Attributes
    ----------
    current_ref : CurrentReference
        Current reference generator.
    observer : Observer
        Flux and rotor position observer, used in the sensorless mode only.
    current_ctrl : CurrentCtrl
        Current controller.
        
    """

    def __init__(
            self,
            par,
            ref_par,
            T_s=250e-6,
            alpha_c=2*np.pi*200,
            alpha_o=2*np.pi*100,
            sensorless=True):
        super().__init__(par, T_s, sensorless)
        self.current_ref = CurrentReference(par, ref_par)
        self.current_ctrl = CurrentCtrl(par, alpha_c)
        self.speed_ctrl = SpeedCtrl(par.J, 2*np.pi*4)
        if sensorless:
            self.observer = Observer(
                ObserverCfg(par, sensorless, alpha_o=alpha_o))

    def get_feedback_signals(self, mdl):
        """Override the base class method."""
        fbk = super().get_feedback_signals(mdl)

        if not self.observer:
            # No observer, use the measured values
            fbk.i_s = np.exp(-1j*fbk.theta_m)*fbk.i_ss
            fbk.u_s = np.exp(-1j*fbk.theta_m)*fbk.u_ss
            fbk.w_s = fbk.w_m  # Angular speed of the coordinate system

        return fbk

    def output(self, fbk):

        ref = super().output(fbk)
        ref = super().get_torque_reference(fbk, ref)
        ref.i_s, ref.tau_M_lim = self.current_ref.output(
            ref.tau_M, fbk.w_m, fbk.u_dc)
        ref.u_s = self.current_ctrl.output(ref.i_s, fbk.i_s)
        ref.u_ss = ref.u_s*np.exp(1j*fbk.theta_m)
        ref.d_abc = self.pwm(ref.T_s, ref.u_ss, fbk.u_dc, fbk.w_s)

        return ref

    def update(self, fbk, ref):
        super().update(fbk, ref)
        self.current_ref.update(ref.T_s, ref.tau_M_lim, ref.u_s, fbk.u_dc)
        self.current_ctrl.update(ref.T_s, fbk.u_s, fbk.w_s)


# %%
class CurrentCtrl(ComplexPICtrl):
    """
    Current controller for synchronous machines.

    This provides an interface of a current controller for synchronous machines
    [#Awa2019a]_. The gains are initialized based on the desired closed-loop 
    bandwidth and the inductances. 

    Parameters
    ----------
    par : ModelPars
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
        k_p, k_i, k_t = 2*alpha_c, alpha_c**2, alpha_c
        super().__init__(k_p, k_i, k_t)
        self.L_d = par.L_d
        self.L_q = par.L_q

    def output(self, i_ref, i):
        # Extends the base class method by transforming the currents to the
        # flux linkages, which is a simple way to take the saliency into account
        psi_ref = self.L_d*i_ref.real + 1j*self.L_q*i_ref.imag
        psi = self.L_d*i.real + 1j*self.L_q*i.imag
        return super().output(psi_ref, psi)


# %%
# pylint: disable=too-many-instance-attributes
@dataclass
class CurrentReferenceCfg:
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
    w_m_nom : float, optional
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
    i_sd_mtpa : callable
        MTPA d-axis current (A) as a function of the torque (Nm).
    tau_M_lim : callable
        Torque limit (Nm) as a function of the stator flux linkage (Vs). This
        limit merges the MTPV and current limits.
    i_sd_lim : callable
        d-axis current limit (A) as a function of the stator flux linkage (Vs).
        This limit merges the MTPV and current limits.
    
    """
    par: InitVar[ModelPars]
    i_s_max: float
    psi_s_min: float = None
    w_m_nom: InitVar[float] = None
    alpha_fw: InitVar[float] = 2*np.pi*20
    k_fw: float = None
    k_u: float = 0.95

    def __post_init__(self, par, w_m_nom, alpha_fw):
        # Minimum stator flux
        if self.psi_s_min is None:
            self.psi_s_min = par.psi_f
        # Field-weakening gain
        if self.k_fw is None:
            self.k_fw = alpha_fw/(w_m_nom*par.L_d)
        # Generate LUTs
        tq = TorqueCharacteristics(par)
        mtpa = tq.mtpa_locus(self.i_s_max, self.psi_s_min)
        lim = tq.mtpv_and_current_limits(self.i_s_max)
        # MTPA locus
        self.i_sd_mtpa = mtpa.i_sd_vs_tau_M
        # Merged MTPV and current limits
        self.tau_M_lim = lim.tau_M_vs_abs_psi_s
        self.i_sd_lim = lim.i_sd_vs_tau_M


# %%
class CurrentReference:
    """
    Current reference calculation.

    This method includes the MTPA locus and field-weakening operation based on
    the unlimited voltage reference feedback. The MTPV and current limits are
    taken into account. This resembles the method presented [#Bed2020]_.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    ref : CurrentReferenceCfg
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

    def __init__(self, par, ref):
        # Machine model parameters
        self.n_p = par.n_p
        self.L_d, self.L_q, self.psi_f = par.L_d, par.L_q, par.psi_f
        # Reference generation parameters
        self.i_sd_mtpa = ref.i_sd_mtpa  # MTPA locus
        self.tau_M_lim = ref.tau_M_lim  # Merged MTPV and current limits
        self.i_sd_lim = ref.i_sd_lim  # Merged MTPV and current limits
        self.psi_s_min = ref.psi_s_min  # Minimum flux linkage
        self.i_s_max = ref.i_s_max  # Maximum current
        self.k_fw = ref.k_fw  # Field-weakening gain
        self.k_u = ref.k_u  # Voltage utilization factor
        # State
        self.i_sd_ref = 0

    def output(self, tau_M_ref, w_m, u_dc):
        """
        Compute the stator current reference.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference (Nm).
        w_m : float
            Rotor speed (electrical rad/s)
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        i_s_ref : complex
            Stator current reference (A).
        tau_M_ref_lim : float
            Limited torque reference (Nm).

        """

        def limit_torque(tau_M_ref, w_m, u_dc):
            if np.abs(w_m) > 0:
                psi_s_max = self.k_u*u_dc/np.sqrt(3)/np.abs(w_m)
                tau_M_max = self.tau_M_lim(psi_s_max)
            else:
                tau_M_max = self.tau_M_lim(np.inf)

            if np.abs(tau_M_ref) > tau_M_max:
                tau_M_ref = np.sign(tau_M_ref)*tau_M_max

            return tau_M_ref

        # Limit the torque reference according to MTPV and current limits
        tau_M_ref = limit_torque(tau_M_ref, w_m, u_dc)

        # q-axis current reference
        psi_t = self.psi_f + (self.L_d - self.L_q)*self.i_sd_ref
        i_sq_ref = tau_M_ref/(1.5*self.n_p*psi_t) if psi_t != 0 else 0

        # Limit the q-axis current reference
        i_sd_mtpa = self.i_sd_mtpa(np.abs(tau_M_ref))
        i_sq_max = np.min([
            np.sqrt(self.i_s_max**2 - self.i_sd_ref**2),
            np.sqrt(self.i_s_max**2 - i_sd_mtpa**2)
        ])
        if np.abs(i_sq_ref) > i_sq_max:
            i_sq_ref = np.sign(i_sq_ref)*i_sq_max

        # Current reference
        i_s_ref = self.i_sd_ref + 1j*i_sq_ref

        # Limited torque (for the speed controller)
        tau_M_ref_lim = 1.5*self.n_p*psi_t*i_sq_ref

        return i_s_ref, tau_M_ref_lim

    def update(self, T_s, tau_M_ref_lim, u_s_ref, u_dc):
        """
        Field-weakening control based on the unlimited reference voltage.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        tau_M_ref_lim : float
            Limited torque reference (Nm).
        u_s_ref : complex
            Unlimited stator voltage reference (V).
        u_dc : float 
            DC-bus voltage (V).

        """
        u_s_max = self.k_u*u_dc/np.sqrt(3)
        self.i_sd_ref += T_s*self.k_fw*(u_s_max - np.abs(u_s_ref))

        # Limit the current
        i_sd_mtpa = self.i_sd_mtpa(np.abs(tau_M_ref_lim))
        i_sd_lim = self.i_sd_lim(np.abs(tau_M_ref_lim))
        self.i_sd_ref = np.clip(self.i_sd_ref, i_sd_lim, i_sd_mtpa)
