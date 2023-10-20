"""
Vector control methods for induction machine drives.

The algorithms are written based on the inverse-Γ model.

"""
from dataclasses import dataclass, InitVar
import numpy as np
from motulator._helpers import abc2complex
from motulator.control._common import PWM, Ctrl, ComplexPICtrl, SpeedCtrl
from motulator.control.im._observers import Observer, FullOrderObserver
from motulator._utils import Bunch


# %%
@dataclass
class ModelPars:
    """
    Inverse-Γ model parameters of an induction machine.
    
    Parameters
    ----------
    R_s : float
        Stator resistance (Ω).
    R_R : float
        Rotor resistance (Ω).
    L_sgm : float
        Leakage inductance (H).
    L_M : float
        Magnetizing inductance (H).
    n_p : int
        Number of pole pairs.  
    J : float
        Moment of inertia (kgm²).  
    
    """
    R_s: float = None
    R_R: float = None
    L_sgm: float = None
    L_M: float = None
    n_p: int = None
    J: float = None


# %%
# pylint: disable=too-many-instance-attributes
class VectorCtrl(Ctrl):
    """
    Vector control for induction machine drives.

    This class interconnects the subsystems of the control system and provides 
    the interface to the solver.

    Parameters
    ----------
    par : InductionMachinePars
        Machine model parameters.
    ref : CurrentReferencePars
        Parameters for reference generation.
    T_s : float, optional
        Sampling period (s). The default is 250e-6.
    sensorless : bool, optional
        If True, sensorless control is used. The default is True.

    Attributes
    ----------
    current_ref : CurrentReference
        Current reference generator.
    observer : Observer
        Flux and speed observer. 
    current_ctrl : CurrentCtrl
        Current controller.
    speed_ctrl : SpeedCtrl 
        Speed controller.
    pwm : PWM
        Pulse-width modulation.
    w_m_ref : callable
        Speed reference (electrical rad/s) as a function of time (s).

    """

    def __init__(self, par, ref, T_s=250e-6, sensorless=True):
        super().__init__()
        self.w_m_ref = callable
        self.T_s = T_s
        self.sensorless = sensorless
        self.n_p = par.n_p
        self.current_ref = CurrentReference(par, ref)
        self.current_ctrl = CurrentCtrl(par, 2*np.pi*200)
        self.observer = Observer(par, sensorless=sensorless)
        self.speed_ctrl = SpeedCtrl(par.J, 2*np.pi*4)
        self.pwm = PWM()

    # pylint: disable=too-many-locals
    def __call__(self, mdl):
        """
        Run the main control loop.

        Parameters
        ----------
        mdl : Drive
            Continuous-time system model for getting the feedback signals.

        Returns
        -------
        T_s : float
            Sampling period (s).
        d_abc : ndarray, shape (3,)
            Duty ratios.

        """
        # Get the speed reference
        w_m_ref = self.w_m_ref(self.clock.t)

        # Feedback signals
        i_s_abc = mdl.machine.meas_currents()
        u_dc = mdl.converter.meas_dc_voltage()

        # Get the states
        if self.sensorless:
            w_m = self.observer.w_m
        else:
            w_m = self.n_p*mdl.mechanics.meas_speed()
        u_s = self.pwm.realized_voltage
        psi_R, theta_s = self.observer.psi_R, self.observer.theta_s

        # Space vector and coordinate transformation
        i_s = np.exp(-1j*theta_s)*abc2complex(i_s_abc)

        # Controller outputs
        tau_M_ref = self.speed_ctrl.output(w_m_ref/self.n_p, w_m/self.n_p)
        i_s_ref, tau_M_ref_lim = self.current_ref.output(tau_M_ref, psi_R)
        u_s_ref = self.current_ctrl.output(i_s_ref, i_s)

        # Save data
        data = Bunch(
            i_s=i_s,
            i_s_ref=i_s_ref,
            psi_R=psi_R,
            t=self.clock.t,
            tau_M_ref_lim=tau_M_ref_lim,
            theta_s=theta_s,
            u_dc=u_dc,
            u_s=u_s,
            w_m=w_m,
            w_m_ref=w_m_ref,
        )
        self.save(data)

        # Compute the flux angular frequency and update the observer states
        # (w_m needed only in sensored mode)
        w_s = self.observer(self.T_s, u_s, i_s, w_m)

        # Update the controller states
        self.speed_ctrl.update(self.T_s, tau_M_ref_lim)
        self.current_ref.update(self.T_s, u_s_ref, u_dc)
        self.current_ctrl.update(self.T_s, u_s, w_s)
        self.clock.update(self.T_s)

        # Calculate the duty ratios and update the voltage estimate state
        d_abc = self.pwm(self.T_s, u_s_ref, u_dc, theta_s, w_s)

        return self.T_s, d_abc


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
class CurrentReferencePars:
    """
    Parameters for reference generation.

    This dataclass stores the nominal and limit values needed for reference 
    generation. For calculating the rotor flux reference, the machine parameters 
    are also required.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    i_s_max : float
        Maximum stator current (A). 
    u_s_nom : float, optional
        Nominal stator voltage (V). The default is sqrt(2/3)*400.
    w_s_nom : float, optional
        Nominal stator angular frequency (rad/s). The default is 2*pi*50.
    psi_R_nom : float, optional
        Nominal rotor flux linkage (Vs). The default is 
        `(u_s_nom/w_s_nom)/(1 + L_sgm/L_M)`.
    k_fw : float, optional
        Field-weakening gain (1/H). The default is `2*R_R/(w_s_nom*L_sgm**2)`.
    k_u : float, optional
        Voltage utilization factor. The default is 0.95.
    
    """
    par: InitVar[ModelPars] = None
    i_s_max: float = None
    u_s_nom: InitVar[float] = np.sqrt(2/3)*400
    w_s_nom: InitVar[float] = 2*np.pi*50
    psi_R_nom: float = None
    k_fw: float = None
    k_u: float = 0.95

    def __post_init__(self, par, u_s_nom, w_s_nom):
        psi_s_nom = u_s_nom/w_s_nom  # Nominal stator flux
        if self.psi_R_nom is None:
            # Nominal rotor flux (omitting the slip)
            self.psi_R_nom = psi_s_nom/(1 + par.L_sgm/par.L_M)
        if self.k_fw is None:
            self.k_fw = 2*par.R_R/(w_s_nom*par.L_sgm**2)


# %%
class CurrentReference:
    """
    Current reference generation.

    In the base-speed region, the current reference in rotor-flux coordinates is
    given by::

        i_s_ref = psi_R_nom/L_M + 1j*tau_M_ref/(1.5*n_p*abs(psi_R))

    where `psi_R_nom` is the nominal rotor flux magnitude and `psi_R` is the 
    estimated rotor flux. The field-weakening operation is based on adjusting 
    the flux-producing current component::

        i_s_ref.real = (k_fw/s)*(u_s_max - abs(u_s_ref))

    where `1/s` refers to integration, ``u_s_max = k_u*u_dc/sqrt(3)`` is the 
    maximum stator voltage in the linear modulation region, `u_s_ref` is the 
    (unlimited) stator voltage reference, and `k_fw` is the field-weakening 
    gain. The field-weakening method and its tuning corresponds roughly to
    [#Hin2006]_. Furthermore, the torque-producing current component 
    `i_s_ref.imag` is limited based on the maximum stator current and the 
    breakdown slip. 

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    ref : CurrentReferencePars
        Reference generation parameters.

    References
    ----------
    .. [#Hin2006] Hinkkanen, Luomi, "Braking scheme for vector-controlled 
       induction motor drives equipped with diode rectifier without braking 
       resistor," IEEE Trans. Ind. Appl., 2006, 
       https://doi.org/10.1109/TIA.2006.880852

    """

    def __init__(self, par, ref):
        # Machine model parameters
        self.L_sgm = par.L_sgm
        self.n_p = par.n_p
        # Other parameters
        self.i_s_max = ref.i_s_max
        self.k_u = ref.k_u
        # Field-weakening gain
        self.k_fw = ref.k_fw
        # Nominal d-axis current
        self.i_sd_nom = ref.psi_R_nom/par.L_M
        # State variable
        self.i_sd_ref = self.i_sd_nom

    def output(self, tau_M_ref, psi_R):
        """
        Compute the stator current reference.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference (Nm).
        psi_R : float
            Rotor flux magnitude (Vs).

        Returns
        -------
        i_s_ref : complex
            Stator current reference (A).
        tau_M_ref_lim : float
            Limited torque reference (Nm).

        """

        def q_axis_current_limit(i_sd_ref, psi_R):
            # Priority given to the d component
            i_sq_max1 = np.sqrt(self.i_s_max**2 - i_sd_ref**2)
            # Breakdown torque limit
            i_sq_max2 = psi_R/self.L_sgm + i_sd_ref
            # q-axis current limit
            i_sq_max = np.min([i_sq_max1, i_sq_max2])
            return i_sq_max

        # q-axis current reference
        i_sq_ref = tau_M_ref/(1.5*self.n_p*psi_R) if psi_R > 0 else 0

        # Limit the current
        i_sq_max = q_axis_current_limit(self.i_sd_ref, psi_R)
        i_sq_ref = np.clip(i_sq_ref, -i_sq_max, i_sq_max)

        # Current reference
        i_s_ref = self.i_sd_ref + 1j*i_sq_ref

        # Limited torque (for the speed controller)
        tau_M_ref_lim = 1.5*self.n_p*psi_R*i_sq_ref

        return i_s_ref, tau_M_ref_lim

    def update(self, T_s, u_s_ref, u_dc):
        """
        Field-weakening based on the unlimited reference voltage.

        Parameters
        ----------
        T_s : float
            Sampling period (s)
        u_s_ref : complex
            Unlimited stator voltage reference (V).
        u_dc : float
            DC-bus voltage (V).

        """
        u_s_max = self.k_u*u_dc/np.sqrt(3)
        self.i_sd_ref += T_s*self.k_fw*(u_s_max - np.abs(u_s_ref))
        self.i_sd_ref = np.clip(self.i_sd_ref, -self.i_s_max, self.i_sd_nom)
