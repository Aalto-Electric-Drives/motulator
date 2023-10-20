"""
Observer-based V/Hz control for induction machine drives.

This implements the observer-based V/Hz control method described in [#Tii2022]_. 
The state-feedback control law is in the alternative form which uses an
intermediate stator current reference.

References
----------
.. [#Tii2022] Tiitinen, Hinkkanen, Harnefors, "Stable and passive observer-based 
   V/Hz control for induction motors," Proc. IEEE ECCE, Detroit, MI, Oct. 2022,
   https://doi.org/10.1109/ECCE50734.2022.9948057

"""

# %%
from dataclasses import dataclass
import numpy as np
from motulator.control._common import Ctrl, PWM, RateLimiter
from motulator.control.im._observers import FluxObserver
from motulator._helpers import abc2complex
from motulator._utils import Bunch


# %%
# pylint: disable=too-many-instance-attributes
@dataclass
class ObserverBasedVHzCtrlPars:
    """
    Parameters for the control system.

    Parameters
    ----------
    psi_s_nom : float
        Nominal stator flux linkage (Vs). 
    i_s_max : float, optional
        Maximum stator current (A). The default is inf.
    k_tau : float, optional
        Torque controller gain. The default is 3.
    alpha_psi : float, optional
        Stator flux control bandwidth (rad/s). The default is 2*pi*20.
    alpha_f : float, optional
        Torque high-pass filter bandwidth (rad/s). The default is 2*pi*1.
    alpha_r : float, optional
        Low-pass-filter bandwidth (rad/s) for slip angular frequency. The
        default is 2*pi*1.
    slip_compensation : bool, optional
        Enable slip compensation. The default is False.

    """
    # par: InitVar[ModelPars | None] = None
    psi_s_nom: float = None
    i_s_max: float = np.inf
    k_tau: float = 3.
    alpha_psi: float = 2*np.pi*20
    alpha_f: float = 2*np.pi*1
    alpha_r: float = 2*np.pi*1
    slip_compensation: bool = False


# %%
class ObserverBasedVHzCtrl(Ctrl):
    """
    Observer-based V/Hz control for induction machines.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    ctrl_par : ObserverBasedVHzCtrlPars
        Control system parameters.
    T_s : float, optional
        Sampling period (s). The default is 250e-6.

    Attributes
    ----------
    observer : SensorlessObserverExtCoord
        Sensorless reduced-order flux observer in external coordinates.
    rate_limiter : RateLimiter
        Rate limiter for the speed reference.
    pwm : PWM
        Pulse-width modulation.
    w_m_ref : callable
        Speed reference (electrical rad/s) as a function of time (s).

    """

    # pylint: disable=too-many-instance-attributes, too-many-arguments
    def __init__(self, par, ctrl_par, T_s=250e-6):
        super().__init__()
        self.T_s = T_s
        # Model parameters
        self.R_s, self.R_R = par.R_s, par.R_R
        self.L_sgm = par.L_sgm
        self.n_p = par.n_p
        # References
        self.w_m_ref = callable
        self.psi_s_ref = ctrl_par.psi_s_nom
        self.i_s_max = ctrl_par.i_s_max
        # Control parameters
        self.slip_compensation = ctrl_par.slip_compensation
        self.alpha_f = ctrl_par.alpha_f
        self.alpha_r = ctrl_par.alpha_r
        self.alpha_psi = ctrl_par.alpha_psi
        self.k_tau = ctrl_par.k_tau
        # Instantiate classes
        self.observer = FluxObserver(par, alpha_o=2*np.pi*40)
        self.rate_limiter = RateLimiter()
        self.pwm = PWM(six_step=False)
        # States
        self.theta_s, self.tau_M_ref, self.w_r_ref = 0, 0, 0

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
        w_m_ref = self.rate_limiter(self.T_s, self.w_m_ref(self.clock.t))

        # Measure the feedback signals
        i_s_abc = mdl.machine.meas_currents()  # Phase currents
        u_dc = mdl.converter.meas_dc_voltage()  # DC-bus voltage

        # Get the states
        u_s = self.pwm.realized_voltage
        psi_R = self.observer.psi_R
        tau_M_ref = self.tau_M_ref
        w_r_ref = self.w_r_ref
        theta_s = self.theta_s

        # Space vector and coordinate transformation
        i_s = np.exp(-1j*theta_s)*abc2complex(i_s_abc)

        # Torque estimate
        tau_M = 1.5*self.n_p*np.imag(i_s*np.conj(psi_R))

        # Slip frequency compensation (if enabled) for the low-pass filter.
        # Note, could also be based on the low-pass filtered torque.
        psi_R_sqr = np.abs(psi_R)**2
        if self.slip_compensation and psi_R_sqr > 0:
            w_r = self.R_R*tau_M/(1.5*self.n_p*psi_R_sqr)
        else:
            w_r = 0

        # Slip compensation. Uses the low-pass filtered slip estimate w_r_ref.
        # Note if slip compensation disabled w_r_ref == 0.
        w_s_ref = w_m_ref + w_r_ref

        # Dynamic frequency
        w_s = w_s_ref - self.k_tau*(tau_M - tau_M_ref)

        # State feedback
        u_s_ref = self._state_feedback(i_s, psi_R, w_s)

        # Data logging
        data = Bunch(
            i_s=i_s,
            psi_s=psi_R + self.L_sgm*i_s,
            psi_s_ref=self.psi_s_ref,
            t=self.clock.t,
            theta_s=self.theta_s,
            u_dc=u_dc,
            u_s=u_s,
            w_m=self.observer.w_m,
            w_m_ref=w_m_ref,
            w_s=w_s,
            tau_M=tau_M,
        )
        self.save(data)

        # Update the states
        self.observer.update(self.T_s, u_s, i_s, w_s)
        self.w_r_ref += self.T_s*self.alpha_r*(w_r - self.w_r_ref)
        self.tau_M_ref += self.T_s*self.alpha_f*(tau_M - self.tau_M_ref)
        self.theta_s += self.T_s*w_s  # Next line: limit into [-pi, pi)
        self.theta_s = np.mod(self.theta_s + np.pi, 2*np.pi) - np.pi
        self.clock.update(self.T_s)

        # Calculate the duty ratios and update the voltage estimate state
        d_abc = self.pwm(self.T_s, u_s_ref, u_dc, theta_s, w_s)

        return self.T_s, d_abc

    def _state_feedback(self, i_s, psi_R, w_s):
        # Internal current reference for state feedback
        i_s_ref = (self.psi_s_ref - psi_R)/self.L_sgm
        # Limit the reference
        if np.abs(i_s_ref) > self.i_s_max:
            i_s_ref = self.i_s_max*i_s_ref/np.abs(i_s_ref)
        # State feedback
        u_s_ref = (
            self.R_s*i_s_ref + 1j*w_s*self.psi_s_ref +
            self.L_sgm*self.alpha_psi*(i_s_ref - i_s))
        return u_s_ref
