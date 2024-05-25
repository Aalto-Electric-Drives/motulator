"""
Vector control methods for induction machine drives.

The algorithms are written based on the inverse-Γ model.

"""
from dataclasses import dataclass, InitVar
import numpy as np
from motulator.control._common import ComplexPICtrl, SpeedCtrl, DriveCtrl
from motulator.control.im._common import Observer, ObserverCfg, ModelPars


# %%
class CurrentVectorCtrl(DriveCtrl):
    """Vector control for induction machine drives."""

    def __init__(self, par, cfg, T_s=250e-6, sensorless=True):
        super().__init__(par, T_s, sensorless)
        self.current_reference = CurrentReference(par, cfg)
        self.current_ctrl = CurrentCtrl(par, 2*np.pi*200)
        self.speed_ctrl = SpeedCtrl(par.J, 2*np.pi*4)
        self.observer = Observer(ObserverCfg(par, T_s, sensorless=sensorless))

    def output(self, fbk):
        ref = super().output(fbk)
        ref = super().get_torque_reference(fbk, ref)
        ref = self.current_reference.output(fbk, ref)
        ref.u_s = self.current_ctrl.output(ref.i_s, fbk.i_s)
        u_ss = ref.u_s*np.exp(1j*fbk.theta_s)
        ref.d_abc = self.pwm(ref.T_s, u_ss, fbk.u_dc, fbk.w_s)
        return ref

    def update(self, fbk, ref):
        super().update(fbk, ref)
        self.current_reference.update(fbk, ref)
        self.current_ctrl.update(ref.T_s, fbk.u_s, fbk.w_s)


# %%
class CurrentCtrl(ComplexPICtrl):
    """
    2DOF PI current controller for induction machines.

    This class provides an interface for a current controller for induction 
    machines. The gains are initialized based on the desired closed-loop 
    bandwidth and the leakage inductance. 

    Parameters
    ----------
    par : ModelPars
        Machine parameters, contains the leakage inductance `L_sgm` (H).  
    alpha_c : float
        Closed-loop bandwidth (rad/s).

    """

    def __init__(self, par, alpha_c):
        k_t = alpha_c*par.L_sgm
        k_i = alpha_c*k_t
        k_p = 2*k_t
        super().__init__(k_p, k_i, k_t)


# %%
# pylint: disable=too-many-instance-attributes
@dataclass
class CurrentReferenceCfg:
    """
    Reference generation configuration.

    This dataclass stores the nominal and limit values needed for reference 
    generation. For calculating the rotor flux reference, the machine 
    parameters are also required.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    max_i_s : float
        Maximum stator current (A). 
    nom_u_s : float, optional
        Nominal stator voltage (V). The default is sqrt(2/3)*400.
    nom_w_s : float, optional
        Nominal stator angular frequency (rad/s). The default is 2*pi*50.
    nom_psi_R : float, optional
        Nominal rotor flux linkage (Vs). The default is 
        `(nom_u_s/nom_w_s)/(1 + L_sgm/L_M)`.
    k_fw : float, optional
        Field-weakening gain (1/H). The default is `2*R_R/(nom_w_s*L_sgm**2)`.
    k_u : float, optional
        Voltage utilization factor. The default is 0.95.
    
    """
    par: InitVar[ModelPars] = None
    max_i_s: float = None
    nom_u_s: InitVar[float] = np.sqrt(2/3)*400
    nom_w_s: InitVar[float] = 2*np.pi*50
    nom_psi_R: float = None
    k_fw: float = None
    k_u: float = 0.95

    def __post_init__(self, par, nom_u_s, nom_w_s):
        nom_psi_s = nom_u_s/nom_w_s  # Nominal stator flux
        if self.nom_psi_R is None:
            # Nominal rotor flux (omitting the slip)
            self.nom_psi_R = nom_psi_s/(1 + par.L_sgm/par.L_M)
        if self.k_fw is None:
            self.k_fw = 2*par.R_R/(nom_w_s*par.L_sgm**2)


# %%
class CurrentReference:
    """
    Current reference generation.

    In the base-speed region, the current reference in rotor-flux coordinates 
    is given by::

        ref_i_s = nom_psi_R/L_M + 1j*ref_tau_M/(1.5*n_p*abs(psi_R))

    where `nom_psi_R` is the nominal rotor flux magnitude and `psi_R` is the 
    estimated rotor flux. The field-weakening operation is based on adjusting 
    the flux-producing current component::

        ref_i_s.real = (k_fw/s)*(max_u_s - abs(ref_u_s))

    where `1/s` refers to integration, ``max_u_s = k_u*u_dc/sqrt(3)`` is the 
    maximum stator voltage in the linear modulation region, `ref_u_s` is the 
    (unlimited) stator voltage reference, and `k_fw` is the field-weakening 
    gain. The field-weakening method and its tuning corresponds roughly to
    [#Hin2006]_. Furthermore, the torque-producing current component 
    `ref_i_s.imag` is limited based on the maximum stator current and the 
    breakdown slip. 

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    cfg : CurrentReferenceCfg
        Reference generation configuration.

    References
    ----------
    .. [#Hin2006] Hinkkanen, Luomi, "Braking scheme for vector-controlled 
       induction motor drives equipped with diode rectifier without braking 
       resistor," IEEE Trans. Ind. Appl., 2006, 
       https://doi.org/10.1109/TIA.2006.880852

    """

    def __init__(self, par, cfg):
        self.par, self.cfg = par, cfg
        self.cfg.nom_i_sd = cfg.nom_psi_R/par.L_M  # Nominal d-axis current
        self.ref_i_sd = self.cfg.nom_i_sd  # State

    def output(self, fbk, ref):
        """Compute the stator current reference."""
        par, cfg = self.par, self.cfg  # Unpack
        ref_i_sd = self.ref_i_sd  # Get the state

        def q_axis_current_limit(ref_i_sd, psi_R):
            # Priority given to the d component
            max_i_sq1 = np.sqrt(cfg.max_i_s**2 - ref_i_sd**2)
            # Breakdown torque limit
            max_i_sq2 = psi_R/par.L_sgm + ref_i_sd
            # q-axis current limit
            max_i_sq = np.min([max_i_sq1, max_i_sq2])
            return max_i_sq

        # q-axis current reference
        ref_i_sq = ref.tau_M/(1.5*par.n_p*fbk.psi_R) if fbk.psi_R > 0 else 0

        # Limit the current
        max_i_sq = q_axis_current_limit(ref_i_sd, fbk.psi_R)
        ref_i_sq = np.clip(ref_i_sq, -max_i_sq, max_i_sq)

        # Current reference
        ref.i_s = ref_i_sd + 1j*ref_i_sq

        # Limited torque (for the speed controller)
        ref.tau_M_lim = 1.5*par.n_p*fbk.psi_R*ref_i_sq

        return ref

    def update(self, fbk, ref):
        """Field-weakening based on the unlimited reference voltage."""
        cfg = self.cfg
        max_u_s = cfg.k_u*fbk.u_dc/np.sqrt(3)
        self.ref_i_sd += ref.T_s*cfg.k_fw*(max_u_s - np.abs(ref.u_s))
        self.ref_i_sd = np.clip(self.ref_i_sd, -cfg.max_i_s, cfg.nom_i_sd)
