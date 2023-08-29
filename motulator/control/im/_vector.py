"""
Vector control methods for induction machine drives.

The algorithms are written based on the inverse-Γ model.

"""
from dataclasses import dataclass, InitVar
import numpy as np
from motulator._helpers import abc2complex
from motulator.control._common import PWM, Ctrl, ComplexPICtrl, SpeedCtrl
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
    
    Methods
    -------
    output(tau_M_ref, psi_R)
        Compute the stator current reference.
    update(T_s, u_s_ref, u_dc)
        Field-weakening based on the unlimited reference voltage.
    
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


# %%
class Observer:
    """
    Reduced-order flux observer for induction machines.

    This class implements a reduced-order flux observer for induction machines.
    Both sensored and sensorless operation are supported. The observer structure
    is similar to [#Hin2010]_. Both sensored and sensorless operation are 
    supported. The observer operates in estimated rotor flux coordinates. 
    
    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    k : callable, optional
        Observer gain as a function of the rotor angular speed. The default is 
        ``lambda w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)`` if
        `sensorless` else ``lambda w_m: 1 + 0.2*abs(w_m)/(R_R/L_M - 1j*w_m)``.
    alpha_o : float, optional
        Observer bandwidth (rad/s). The default is 2*pi*40.
    sensorless : bool, optional
        If True, sensorless mode is used. The default is True.

    Attributes
    ----------
    psi_R : float
        Rotor flux magnitude estimate (Vs).
    theta_s : float
        Rotor flux angle estimate (rad).
    w_m : float
        Rotor angular speed estimate (electrical rad/s). In sensored mode, this
        is the measured low-pass-filtered speed.

    Notes
    -----
    The pure voltage model corresponds to ``k = lambda w_m: 0``, resulting in 
    the marginally stable estimation-error dynamics. The current model is 
    obtained by setting ``k = lambda w_m: 1``. 

    References
    ----------
    .. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers 
       with stator-resistance adaptation for speed-sensorless induction motor 
       drives," IEEE Trans. Power Electron., 2010, 
       https://doi.org/10.1109/TPEL.2009.2039650

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, par, k=None, alpha_o=2*np.pi*40, sensorless=True):
        # Model parameters
        self.R_s, self.R_R, self.L_sgm = par.R_s, par.R_R, par.L_sgm
        self.alpha = par.R_R/par.L_M

        # Other parameters
        self.alpha_o = alpha_o
        self.sensorless = sensorless

        # Observer gains
        self.k1 = k
        if self.sensorless:
            if self.k1 is None:  # If not given, use the default gains
                self.k1 = lambda w_m: (.5*self.alpha + .2*np.abs(w_m))/(
                    self.alpha - 1j*w_m)
            self.k2 = self.k1
        else:
            if self.k1 is None:
                self.k1 = lambda w_m: 1 + .2*np.abs(w_m)/(self.alpha - 1j*w_m)
            self.k2 = 0*self.k1

        # States
        self.psi_R, self.theta_s, self.w_m, self._i_s_old = 0, 0, 0, 0

    def _f(self, T_s, u_s, i_s, w_m):
        # Right-hand-side of the differential equation.

        # Induced voltage from stator quantities (without the w_s term that is
        # taken into account separately to avoid an algebraic loop)
        v_s = u_s - self.R_s*i_s - self.L_sgm*(i_s - self._i_s_old)/T_s

        # Induced voltage from the rotor quantities
        v_r = self.R_R*i_s - (self.alpha - 1j*w_m)*self.psi_R

        # Angular frequency of the rotor flux vector
        k1, k2 = self.k1(w_m), self.k2(w_m)
        den = self.psi_R + self.L_sgm*np.real((1 - k1)*i_s + k2*np.conj(i_s))
        num = np.imag(v_s + k1*(v_r - v_s) + k2*np.conj(v_r - v_s))
        w_s = num/den if den > 0 else w_m

        # Compute the derivative of the flux magnitude for the state update
        v = v_s - 1j*w_s*self.L_sgm*i_s
        dpsi_R = np.real(v + k1*(v_r - v) + k2*np.conj(v_r - v))

        return dpsi_R, w_s  # Note: w_s = dtheta_s

    # pylint: disable=too-many-arguments
    def _update(self, T_s, i_s, dpsi_R, w_s, w_m):
        # Update the states
        self.psi_R += T_s*dpsi_R
        self.theta_s += T_s*w_s  # Next line: limit into [-pi, pi)
        self.theta_s = np.mod(self.theta_s + np.pi, 2*np.pi) - np.pi
        self.w_m += T_s*self.alpha_o*(w_m - self.w_m)
        # Store the old current
        self._i_s_old = i_s

    def __call__(self, T_s, u_s, i_s, w_m=None):
        """
        Compute the angular frequency of the flux and update the states.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_s : complex
            Stator voltage (V) in estimated rotor flux coordinates.
        i_s : complex
            Stator current (A) in estimated rotor flux coordinates.
        w_m : float, optional
            Rotor angular speed (electrical rad/s). This signal is only used in
            sensored mode. The default is None.

        Returns
        -------
        w_s : float
            Angular frequency (rad/s) of the rotor flux estimate.

        """
        # Compute the time derivative of the flux estimate
        if self.sensorless:
            dpsi_R, w_s = self._f(T_s, u_s, i_s, self.w_m)
            # Slip and unfiltered speed estimates
            w_r = self.R_R*i_s.imag/self.psi_R if self.psi_R > 0 else 0
            w_m = w_s - w_r
        else:
            dpsi_R, w_s = self._f(T_s, u_s, i_s, w_m)

        # Update the states
        self._update(T_s, i_s, dpsi_R, w_s, w_m)

        return w_s
