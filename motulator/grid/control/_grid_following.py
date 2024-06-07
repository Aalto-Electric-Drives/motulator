"""grid following control methods for grid onverters."""

# %%
from __future__ import annotations
from collections.abc import Callable
from dataclasses import dataclass, field

from types import SimpleNamespace
#from motulator._utils import Bunch
import numpy as np

from motulator.common.utils._utils import abc2complex
from motulator.common.control import (Ctrl, PWM, ComplexFFPICtrl, Clock)


# %%
@dataclass
class GridFollowingCtrlPars:
    """
    grid-following control parameters.

    """
    # pylint: disable=too-many-instance-attributes
    # General control parameters
    p_g_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: 0) # active power reference
    q_g_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: 0) # reactive power reference
    u_dc_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: 650) # DC voltage reference, only used if
                                    # the dc voltage control mode is activated.
    T_s: float = 1/(16e3) # sampling time of the controller.
    delay: int = 1
    u_gN: float = np.sqrt(2/3)*400  # PCC voltage, in volts.
    w_g: float = 2*np.pi*50 # grid frequency, in Hz
    f_sw: float = 8e3 # switching frequency, in Hz.

    # Scaling ratio of the abc/dq transformation
    k_scal: float = 3/2

    # Current controller parameters
    alpha_c: float = 2*np.pi*400 # current controller bandwidth.

    # Phase Locked Loop (PLL) control parameters
    w_0_pll: float = 2*np.pi*20 # undamped natural frequency
    zeta: float = 1 # damping ratio

    # Low pass filter for voltage feedforward term
    alpha_ff: float = 2*np.pi*(4*50) # low pass filter bandwidth

    # Use the filter capacitance voltage measurement or PCC voltage
    on_u_cap: bool = 0 # 1 if capacitor voltage is used, 0 if PCC is used

    # DC-voltage controller
    on_v_dc: bool = 0 # put 1 to activate dc voltage controller. 0 is p-mode
    zeta_dc: float = 1 # damping ratio
    p_max: float = 10e3 # maximum power reference, in W.
    w_0_dc: float = 2*np.pi*30 # controller undamped natural frequency, in rad/s.

    # Current limitation
    i_max: float = 20 # maximum current modulus in A

    # Passive component parameter estimates
    L_f: float = 10e-3 # filter inductance, in H.
    C_dc: float = 1e-3 # DC bus capacitance, in F.


# %%
class GridFollowingCtrl(Ctrl):
    """
    Grid following control for power converters.

    Parameters
    ----------
    pars : GridFollowingCtrlPars
        Control parameters.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, pars):
        super().__init__()
        self.t = 0
        self.T_s = pars.T_s
        # Instantiate classes
        self.pwm = PWM(six_step=False)
        self.clock = Clock()
        self.pll = PLL(pars)
        self.current_ctrl = CurrentCtrl(pars, pars.alpha_c)
        self.current_ref_calc = CurrentRefCalc(pars)
        self.dc_bus_volt_ctrl = DCBusVoltCtrl(
            pars.zeta_dc, pars.w_0_dc, pars.p_max)
        # Parameters
        self.u_gN = pars.u_gN
        self.w_g = pars.w_g
        self.f_sw = pars.f_sw
        self.L_f = pars.L_f
        # Power references
        self.p_g_ref = pars.p_g_ref
        self.q_g_ref = pars.q_g_ref
        # Activation/deactivation of the DC voltage controller
        self.on_v_dc = pars.on_v_dc
        # DC voltage reference
        self.u_dc_ref = pars.u_dc_ref
        # DC-bus capacitance
        self.C_dc = pars.C_dc
        # Calculated current controller gains:
        self.k_p_i = pars.alpha_c*pars.L_f
        self.k_i_i = np.power(pars.alpha_c,2)*pars.L_f
        self.r_i = pars.alpha_c*pars.L_f
        # Calculated maximum current in A
        self.i_max = pars.i_max
        # Calculated PLL estimator gains
        self.k_p_pll = 2*pars.zeta*pars.w_0_pll/pars.u_gN
        self.k_i_pll = pars.w_0_pll*pars.w_0_pll/pars.u_gN
        # Low pass filter for voltage feedforward term
        self.alpha_ff = pars.alpha_ff
        # Measure the voltage at the PCC or the capacitor of LCL (if used)?
        self.on_u_cap = pars.on_u_cap
        # States
        self.u_c_i = 0j
        self.theta_p = 0
        self.u_g_filt = pars.u_gN + 1j*0

    def __call__(self, mdl):
        """
        Run the main control loop.

        Parameters
        ----------
        mdl : StiffSourceAndLFilterModel / StiffSourceAndLCLFilterModel /
              ACFlexSourceAndLFilterModel / ACFlexSourceAndLCLFilterModel /
              DCBusAndLFilterModel / DCBusAndLCLFilterModel
            Continuous-time model of a voltage-source grid model with a filter
            and a resistive-inductive line for getting the feedback signals.

        Returns
        -------
        T_s : float
            Sampling period.
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.

        """
        # Measure the feedback signals
        i_c_abc = mdl.grid_filter.meas_currents()
        u_dc = mdl.converter.meas_dc_voltage()
        if self.on_u_cap == True:
            u_g_abc = mdl.grid_filter.meas_cap_voltage()
        else:
            u_g_abc = mdl.grid_filter.meas_pcc_voltage()
        # Obtain the converter voltage calculated with the PWM
        u_c = self.pwm.realized_voltage

        # Define the active and reactive power references at the given time 
        u_dc_ref = self.u_dc_ref(self.clock.t)
        # Definition of capacitance energy variables for the DC-bus controller
        W_dc_ref = 0.5*self.C_dc*u_dc_ref**2
        W_dc = 0.5*self.C_dc*u_dc**2
        if self.on_v_dc:
            p_g_ref = self.dc_bus_volt_ctrl.output(W_dc_ref, W_dc)
            q_g_ref = self.q_g_ref(self.clock.t)
        else:
            p_g_ref = self.p_g_ref(self.clock.t)
            q_g_ref = self.q_g_ref(self.clock.t)

        # Generate the current references
        i_c_ref = self.current_ref_calc.output(p_g_ref, q_g_ref)

        # Transform the measured current in dq frame
        i_c = np.exp(-1j*self.theta_p)*abc2complex(i_c_abc)

        # Calculation of PCC voltage in synchronous frame
        u_g = np.exp(-1j*self.theta_p)*abc2complex(u_g_abc)

        # Use of PLL to bring ugq to zero
        u_g_q, abs_u_g, w_pll, theta_pll = self.pll.output(u_g_abc)

        # Calculation of the modulus of current reference
        i_abs = np.abs(i_c_ref)
        i_cd_ref = np.real(i_c_ref)
        i_cq_ref = np.imag(i_c_ref)

        # And current limitation algorithm
        if i_abs > 0:
            i_ratio = self.i_max/i_abs
            i_cd_ref = np.sign(i_cd_ref)*np.min(
                [i_ratio*np.abs(i_cd_ref),np.abs(i_cd_ref)])
            i_cq_ref = np.sign(i_cq_ref)*np.min(
                [i_ratio*np.abs(i_cq_ref),np.abs(i_cq_ref)])
            i_c_ref = i_cd_ref + 1j*i_cq_ref

        # Low pass filter for the feedforward PCC voltage:
        u_g_filt = self.u_g_filt

        # Voltage reference generation in synchronous coordinates
        u_c_ref = self.current_ctrl.output(i_c_ref, i_c, u_g_filt, self.w_g)

        # Use the function from control commons:
        d_abc_ref = self.pwm(self.T_s, u_c_ref, u_dc,
                                           self.theta_p, self.w_g)

        # Data logging
        data = SimpleNamespace(
            w_c = w_pll, theta_c = self.theta_p, u_c_ref = u_c_ref,
            u_c = u_c, i_c = i_c, abs_u_g = abs_u_g,
            d_abc_ref = d_abc_ref, i_c_ref = i_c_ref, u_dc = u_dc,
            t = self.clock.t, p_g_ref = p_g_ref, u_dc_ref = u_dc_ref,
            q_g_ref=q_g_ref, u_g = u_g,
                     )
        self.save(data)

        # Update the states
        self.theta_p = theta_pll
        self.w_p = w_pll
        self.current_ctrl.update(self.T_s, u_c)
        self.clock.update(self.T_s)
        self.pll.update(u_g_q)
        if self.on_v_dc:
            self.dc_bus_volt_ctrl.update(self.T_s, p_g_ref)
        # Update the low pass filer integrator for feedforward action
        self.u_g_filt = (1 - self.T_s*self.alpha_ff)*u_g_filt + (
            self.T_s*self.alpha_ff*u_g)

        return self.T_s, d_abc_ref


# %%
class PLL:

    """
    PLL synchronizing loop.

    Parameters
    ----------
    u_g_abc : ndarray, shape (3,)
        Phase voltages at the PCC.

    Returns
    -------
    u_g_q : float
        q-axis of the PCC voltage (V)
    abs_u_g : float
        amplitude of the voltage waveform, in V
    theta_pll : float
        estimated phase angle (in rad).
        
    """

    def __init__(self, pars):

        """
        Parameters
        ----------
        pars : GridFollowingCtrlPars
           Control parameters.
    
        """
        self.T_s = pars.T_s
        self.w_0_pll = pars.w_0_pll
        self.k_p_pll = 2*pars.zeta*pars.w_0_pll/pars.u_gN
        self.k_i_pll = pars.w_0_pll*pars.w_0_pll/pars.u_gN
        # Initial states
        self.w_pll = pars.w_g
        self.theta_p = 0


    def output(self, u_g_abc):

        """
        Compute the estimated frequency and phase angle using the PLL.
    
        Parameters
        ----------
        u_g_abc : ndarray, shape (3,)
            Grid 3-phase voltage.
    
        Returns
        -------
        u_g_q : float
            Error signal (in V, corresponds to the q-axis grid voltage).
        abs_u_g : float
            magnitude of the grid voltage vector (in V).
        w_g_pll : float
            estimated grid frequency (in rad/s).
        theta_pll : float
            estimated phase angle (in rad).
        """

        u_g_ab = u_g_abc[0] - u_g_abc[1] # calculation of phase-to-phase voltages
        u_g_bc = u_g_abc[1] - u_g_abc[2] # calculation of phase-to-phase voltages

        # Calculation of u_g in complex form (stationary coordinates)
        u_g_s = (2/3)*u_g_ab +(1/3)*u_g_bc + 1j*(np.sqrt(3)/(3))*u_g_bc
        # And then in general coordinates
        u_g = u_g_s*np.exp(-1j*self.theta_p)
        # Definition of the error using the q-axis voltage
        u_g_q = np.imag(u_g)

        # Absolute value of the grid-voltage vector
        abs_u_g = np.abs(u_g)

        # Calculation of the estimated PLL frequency
        w_g_pll = self.k_p_pll*u_g_q + self.w_pll

        # Estimated phase angle
        theta_pll = self.theta_p + self.T_s*w_g_pll

        return u_g_q, abs_u_g, w_g_pll, theta_pll


    def update(self, u_g_q):
        """
        Update the integral state.
    
        Parameters
        ----------
        u_g_q : real
            Error signal (in V, corresponds to the q-axis grid voltage).
    
        """

        # Calculation of the estimated PLL frequency
        w_g_pll = self.k_p_pll*u_g_q + self.w_pll

        # Update the integrator state
        self.w_pll = self.w_pll + self.T_s*self.k_i_pll*u_g_q
        # Update the grid-voltage angle state
        self.theta_p = self.theta_p + self.T_s*w_g_pll
        self.theta_p = np.mod(self.theta_p, 2*np.pi)    # Limit to [0, 2*pi]


# %%
class CurrentCtrl(ComplexFFPICtrl):
    """
    2DOF PI current controller for grid converters.

    This class provides an interface for a current controller for grid 
    converters. The gains are initialized based on the desired closed-loop 
    bandwidth and the filter inductance. 

    Parameters
    ----------
    par : ModelPars
        Grid converter parameters, contains the filter inductance `L_f` (H).  
    alpha_c : float
        Closed-loop bandwidth (rad/s).

    """

    def __init__(self, par, alpha_c):
        k_t = alpha_c*par.L_f
        k_i = alpha_c*k_t
        k_p = 2*k_t
        L_f = par.L_f
        super().__init__(k_p, k_i, k_t, L_f)


# %%
class CurrentRefCalc:

    """
    Current controller reference generator
    
    This class is used to generate the current references for the current
    controllers based on the active and reactive power references.
    
    """

    def __init__(self, pars):

        """
        Parameters
        ----------
        pars : GridFollowingCtrlPars
            Control parameters.
    
        """
        self.u_gN = pars.u_gN


    def output(self, p_g_ref, q_g_ref):

        """
        Current reference genetator.
    
        Parameters
        ----------
        p_g_ref : float
            active power reference
        q_g_ref : float
            reactive power reference
    
        Returns
        -------
        i_c_ref : float
            current reference in the rotationary frame
            
        """

        # Calculation of the current references in the stationary frame:
        i_c_ref = 2*p_g_ref/(3*self.u_gN) -2*1j*q_g_ref/(3*self.u_gN)

        return i_c_ref
