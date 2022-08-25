# pylint: disable=invalid-name
"""Current vector control for synchronous motor drives."""

from __future__ import annotations
from collections.abc import Callable
from dataclasses import dataclass, field
import numpy as np

from motulator.helpers import abc2complex, Bunch
from motulator.control.common import Ctrl, SpeedCtrl, PWM
from motulator.control.sm_torque import TorqueCharacteristics


# %%
@dataclass
class SynchronousMotorVectorCtrlPars:
    """Vector control parameters for synchronous motors."""

    # pylint: disable=too-many-instance-attributes
    # Speed reference (in electrical rad/s)
    w_m_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: (t > .2)*(2*np.pi*75))
    # Mode
    sensorless: bool = True
    use_injection: bool = False
    # Sampling period
    T_s: float = 125e-6
    # Bandwidths
    alpha_c: float = 2*np.pi*100
    alpha_fw: float = 2*np.pi*20
    alpha_s: float = 2*np.pi*2
    # Maximum values
    tau_M_max: float = 1.5*14
    i_s_max: float = 2*np.sqrt(2)*4.3

    # tau_M_max: float = 2*14
    # i_s_max: float = 1.5*np.sqrt(2)*5
    psi_s_min: float = 0
    # Voltage margin
    k_u: float = .95
    # Nominal values
    w_nom: float = 2*np.pi*75
    # Motor parameter estimates
    R_s: float = 3.6
    L_d: float = .036
    L_q: float = .051
    psi_f: float = .545
    p: int = 3
    J: float = .015
    # Sensorless observer
    w_o: float = 2*np.pi*40  # Used only in the sensorless mode
    zeta_inf: float = .2
    # Square Wave parameters (only used for injection)
    sw_amplitude: float = 20
    sw_frequency: float = 1/(2*T_s)
    alpha_o: float = 2*np.pi*22
    def plot_luts(self, base):
        """
        Plot control look-up tables.

        Parameters
        ----------
        base : BaseValues
            Base values for scaling the plots.

        """
        tq = TorqueCharacteristics(self)
        tq.plot_current_loci(self.i_s_max, base)
        tq.plot_torque_flux(self.i_s_max, base)
        tq.plot_torque_current(self.i_s_max, base)
        # tq.plot_flux_loci(self.i_s_max, base)


# %%
class SynchronousMotorVectorCtrl(Ctrl):
    """
    Vector control for a synchronous motor drive.

    This class interconnects the subsystems of the control system and
    provides the interface to the solver.

    Parameters
    ----------
    pars : SynchronousMotorVectorCtrlPars
        Control parameters.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, pars):
        super().__init__()
        self.T_s = pars.T_s
        self.w_m_ref = pars.w_m_ref
        self.p = pars.p
        self.sensorless = pars.sensorless
        self.use_injection = pars.use_injection
        self.current_ctrl = CurrentCtrl(pars)
        self.speed_ctrl = SpeedCtrl(pars)
        self.current_ref = CurrentRef(pars)
        self.pwm = PWM(pars)
        if not pars.sensorless:
            self.observer = None

        elif self.use_injection:
                self.Vinj = -pars.sw_amplitude  
                self.i_s_k = 0                  
                self.i_s_k1 = 0          
                self.observer = SensorlessObserverSWInj(pars)      
        else:
                self.observer = SensorlessObserver(pars)

    def __call__(self, mdl):
        """
        Run the main control loop.

        Parameters
        ----------
        mdl : SynchronousMotorDrive
            Continuous-time model of a synchronous motor drive for getting the
            feedback signals.

        Returns
        -------
        T_s : float
            Sampling period.
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.

        """
        # Get the speed reference
        w_m_ref = self.w_m_ref(self.t)

        # Measure the feedback signals
        i_s_abc = mdl.motor.meas_currents()  # Phase currents
        u_dc = mdl.conv.meas_dc_voltage()  # DC-bus voltage

        if not self.sensorless:
            # Measure the rotor speed
            w_m = self.p*mdl.mech.meas_speed()
            # Limit the electrical rotor position into [-pi, pi)
            theta_m = np.mod(
                self.p*mdl.mech.meas_position() + np.pi, 2*np.pi) - np.pi
        else:
            # Get the rotor speed and position estimates
            w_m, theta_m = self.observer.w_m, self.observer.theta_m

        # Get the realized voltage from the PWM method
        u_s = self.pwm.realized_voltage

        # Current vector in estimated rotor coordinates
        i_s = np.exp(-1j*theta_m)*abc2complex(i_s_abc)

        # Outputs
        tau_M_ref = self.speed_ctrl.output(w_m_ref/self.p, w_m/self.p)
        i_s_ref, tau_M_ref_lim = self.current_ref.output(tau_M_ref, w_m, u_dc)
        if not self.use_injection:
            u_s_ref = self.current_ctrl.output(i_s_ref, i_s)
        else:
            # Update square wave voltage
            self.Vinj = -self.Vinj
            # Update current samples
            self.i_s_k2 = self.i_s_k1
            self.i_s_k1 = self.i_s_k
            self.i_s_k = i_s
            # Filter out fundamental current  
            i_s = (i_s + self.i_s_k1)/2
            u_s_ref = self.current_ctrl.output(i_s_ref, i_s)
            u_s_ref += self.Vinj

        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc, theta_m, w_m)

        # Data logging
        data = Bunch(
            i_s=i_s,
            i_s_ref=i_s_ref,
            t=self.t,
            tau_M_ref_lim=tau_M_ref_lim,
            theta_m=theta_m,
            u_dc=u_dc,
            u_s=u_s,
            w_m=w_m,
            w_m_ref=w_m_ref,
        )
        self.save(data)

        # Update states
        if self.sensorless:
            if self.use_injection:
                self.observer.update(self.i_s_k, self.i_s_k1, self.i_s_k2)
            else:
                self.observer.update(u_s, i_s)
        self.speed_ctrl.update(tau_M_ref_lim)
        self.current_ref.update(tau_M_ref_lim, u_s_ref, u_dc)
        self.current_ctrl.update(u_s_ref_lim, w_m)
        self.pwm.update(u_s_ref_lim)
        self.update_clock(self.T_s)

        return self.T_s, d_abc_ref


# %%
class CurrentCtrl:
    """
    2DOF PI current controller.

    This controller corresponds to [1]_. The continuous-time complex-vector
    design corresponding to (13) is used here. This design could be
    equivalently presented as a 2DOF PI controller.

    Parameters
    ----------
    pars : SynchronousMotorVectorCtrlPars (or its subset)
        Control parameters.

    Notes
    -----
    For better performance at high speeds with low sampling frequencies, the
    discrete-time design in (18) is recommended. This implementation does not
    take the magnetic saturation into account.

    References
    ----------
    .. [1] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
       saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
       https://doi.org/10.1109/TIA.2019.2919258

    """

    def __init__(self, pars):

        self.T_s = pars.T_s
        self.L_d = pars.L_d
        self.L_q = pars.L_q
        self.alpha_c = pars.alpha_c
        # Integral state
        self.u_i = 0
        # Memory for the update method
        self.e = 0
        self.u_s_ref = 0

    def output(self, i_s_ref, i_s):
        """
        Compute the unlimited voltage reference.

        Parameters
        ----------
        i_s_ref : complex
            Current reference.
        i_s : complex
            Measured current.

        Returns
        -------
        u_s_ref : complex
            Unlimited voltage reference.

        """
        # Gains
        k_t = self.alpha_c
        k = 2*self.alpha_c
        # PM-flux linkage cancels out
        psi_s_ref = self.L_d*i_s_ref.real + 1j*self.L_q*i_s_ref.imag
        psi_s = self.L_d*i_s.real + 1j*self.L_q*i_s.imag
        self.u_s_ref = k_t*psi_s_ref - k*psi_s + self.u_i
        # Error signal (scaled, corresponds to the stator flux linkage).
        self.e = psi_s_ref - psi_s

        return self.u_s_ref

    def update(self, u_s_ref_lim, w_m):
        """
        Update the integral state.

        Parameters
        ----------
        u_s_ref_lim : complex
            Limited voltage reference.
        w_m : float
            Angular rotor speed.

        """
        k_t = self.alpha_c
        k_i = self.alpha_c*(self.alpha_c + 1j*w_m)
        self.u_i += self.T_s*k_i*(self.e + (u_s_ref_lim - self.u_s_ref)/k_t)


# %%
class CurrentRef:
    """
    Current reference calculation.

    This method includes the MTPA locus and field-weakenting operation based on
    the unlimited voltage reference feedback. The MTPV and current limits are
    taken into account. This resembles the method presented [2]_.

    Parameters
    ----------
    pars : SynchronousMotorVectorCtrlPars (or its subset)
        Control parameters.

    Notes
    -----
    Instead of the PI controller used in [2]_, we use a simpler integral
    controller with a constant gain. The resulting operating-point-dependent
    closed-loop pole could be derived using (12) of the paper. Unlike in [2]_,
    the MTPV limit is also included here by means of limiting the reference
    torque and the d-axis current reference.

    References
    ----------
    .. [2] Bedetti, Calligaro, Petrella, "Analytical design and autotuning of
       adaptive flux-weakening voltage regulation loop in IPMSM drives with
       accurate torque regulation," IEEE Trans. Ind. Appl., 2020,
       https://doi.org/10.1109/TIA.2019.2942807

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, pars):

        self.T_s = pars.T_s
        self.i_s_max = pars.i_s_max
        self.p = pars.p
        self.L_d = pars.L_d
        self.L_q = pars.L_q
        self.psi_f = pars.psi_f
        self.k = pars.alpha_fw/(pars.w_nom*self.L_d)
        self.k_u = pars.k_u
        # Generate LUTs
        tq = TorqueCharacteristics(pars)
        mtpa = tq.mtpa_locus(i_s_max=pars.i_s_max)
        lims = tq.mtpv_and_current_limits(i_s_max=pars.i_s_max)
        # MTPA locus
        self.i_sd_mtpa = mtpa.i_sd_vs_tau_M
        # Merged MTPV and current limits
        self.tau_M_lim = lims.tau_M_vs_abs_psi_s
        self.i_sd_lim = lims.i_sd_vs_tau_M
        # Initial value
        self.i_sd_ref = 0

    def output(self, tau_M_ref, w_m, u_dc):
        """
        Compute the stator current reference.

        Parameters
        ----------
        tau_M_ref : float
            Torque reference.
        w_m : float
            Rotor speed (in electrical rad/s)
        u_dc : float
            DC-bus voltage.

        Returns
        -------
        i_s_ref : complex
            Stator current reference.
        tau_M_ref_lim : float
            Limited torque reference.

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
        i_sq_ref = tau_M_ref/(1.5*self.p*psi_t) if psi_t != 0 else 0

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
        tau_M_ref_lim = 1.5*self.p*psi_t*i_sq_ref

        return i_s_ref, tau_M_ref_lim

    def update(self, tau_M_ref_lim, u_s_ref, u_dc):
        """
        Field-weakening based on the unlimited reference voltage.

        Parameters
        ----------
        tau_M_ref_lim : float
            Limited torque reference.
        u_s_ref : complex
            Unlimited stator voltage reference.
        u_dc : DC-bus voltage.
            float.

        """
        u_s_max = self.k_u*u_dc/np.sqrt(3)
        self.i_sd_ref += self.T_s*self.k*(u_s_max - np.abs(u_s_ref))

        # Limit the current
        i_sd_mtpa = self.i_sd_mtpa(np.abs(tau_M_ref_lim))
        i_sd_lim = self.i_sd_lim(np.abs(tau_M_ref_lim))

        if self.i_sd_ref > i_sd_mtpa:
            self.i_sd_ref = i_sd_mtpa
        elif self.i_sd_ref < i_sd_lim:
            self.i_sd_ref = i_sd_lim


# %%
class SensorlessObserver:
    """
    Sensorless observer.

    This observer corresponds to [3]_. The observer gain decouples the
    electrical and mechanical dynamics and allows placing the poles of the
    corresponding linearized estimation error dynamics. This implementation
    operates in estimated rotor coordinates.

    Parameters
    ----------
    pars : SynchronousMotorVectorCtrlPars (or its subset)
        Control parameters.

    References
    ----------
    .. [3] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
       sensorless synchronous motor drives: Framework for design and analysis,"
       IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

    """

    # pylint: disable=too-many-instance-attributes
    # pylint: disable=too-few-public-methods
    def __init__(self, pars):

        self.T_s = pars.T_s
        self.R_s = pars.R_s
        self.L_d = pars.L_d
        self.L_q = pars.L_q
        self.psi_f = pars.psi_f
        self.k_p = 2*pars.w_o
        self.k_i = pars.w_o**2
        self.b_p = .5*pars.R_s*(pars.L_d + pars.L_q)/(pars.L_d*pars.L_q)
        self.zeta_inf = pars.zeta_inf
        # Initial states
        self.theta_m, self.w_m, self.psi_s = 0, 0, pars.psi_f

    def update(self, u_s, i_s, *_):
        """
        Update the states for the next sampling period.

        Parameters
        ----------
        u_s : complex
            Stator voltage in estimated rotor coordinates.
        i_s : complex
            Stator current in estimated rotor coordinates.

        """
        # Auxiliary flux (12)
        psi_a = self.psi_f + (self.L_d - self.L_q)*np.conj(i_s)

        # Estimation error (6)
        e = self.L_d*i_s.real + 1j*self.L_q*i_s.imag + self.psi_f - self.psi_s

        # Pole locations are chosen according to (36), with c = w_m**2
        # and w_inf = inf, and the gain corresponding to (30) is used
        k = self.b_p + 2*self.zeta_inf*np.abs(self.w_m)
        if np.abs(psi_a) > 0:
            # Correction voltage
            v = k*psi_a*np.real(e/psi_a)
            # Error signal (10)
            eps = -np.imag(e/psi_a)
        else:
            v, eps = 0, 0

        # Speed estimation (9)
        w_m = self.k_p*eps + self.w_m

        # Update the states
        self.psi_s += self.T_s*(u_s - self.R_s*i_s - 1j*w_m*self.psi_s + v)
        self.w_m += self.T_s*self.k_i*eps
        self.theta_m += self.T_s*w_m  # Next line: limit into [-pi, pi)
        self.theta_m = np.mod(self.theta_m + np.pi, 2*np.pi) - np.pi
#%%
class SensorlessObserverSWInj:
    """
    A sensorless observer corresponding to the paper "PWM Switching Frequency Signal Injection
    Sensorless Method in IPMSMs":

    https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=6266731
    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : data object
            Controller parameters.

        """
        self.T_s = pars.T_s
        self.R_s = pars.R_s
        self.L_d = pars.L_d
        self.L_q = pars.L_q
        self.p   = pars.p
        self.J   = pars.J
        self.psi_f = pars.psi_f
        self.wn = 0.25648*pars.alpha_o
        self.k_p = 3*self.wn**2
        self.k_i = self.wn**3
        self.k_d = 3*self.wn
        self.Vinj = pars.sw_amplitude
        # Initial states
        self.theta_m, self.theta_m_mech, self.w_m, self.dw_m = 0, 0, 0, 0

    def update(self, i_s_k, i_s_k1, i_s_k2):
        """
        Update the states for the next sampling period.

        Parameters
        ----------
        i_s_k: complex
            Stator current in estimated rotor coordinates at sample instance k.   
        i_s_k1 : complex
            Stator current in estimated rotor coordinates at sample instance k-1.   
        i_s_k2 : complex
            Stator current in estimated rotor coordinates at sample instance k-2.
        """
        # Calculate current ripple
        di_s_PN = (i_s_k1 - i_s_k2) - (i_s_k - i_s_k1)    
        di_s_PN_m = di_s_PN*np.exp(1j*np.pi/4)
        di_s = np.abs(di_s_PN_m.imag) - np.abs(di_s_PN_m.real)

        psi_s = (self.L_d*i_s_k.real + self.psi_f) + 1j*self.L_q*i_s_k.imag

        # Calculate error gain
        k = self.L_d*self.L_q/((self.L_q-self.L_d)*self.Vinj*self.T_s*2)
        
        # Estimate error in theta_e
        e = di_s*k

        # Estimate torque for feed-forward
        tau_est = 1.5*self.p*np.imag(i_s_k*np.conj(psi_s))

        dw_m = self.dw_m + self.k_p*e + tau_est*self.p/self.J
        w_m = self.w_m + self.k_d*e
        
        # Update the integral states
        self.dw_m += self.k_i*e*self.T_s
        self.w_m += dw_m*self.T_s
        self.theta_m += self.T_s*w_m  # Next line: limit into [-pi, pi)
        self.theta_m = np.mod(self.theta_m, 2*np.pi)