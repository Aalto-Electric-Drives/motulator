"""
Observer-based V/Hz control for induction motor drives.

This implements the observer-based V/Hz control method described in [1]_. The state-feedback
control law is in the alternative form which uses an intermediate stator current reference.

References
----------
.. [1] Tiitinen, Hinkkanen, Harnefors, "Stable and passive observer-based V/Hz
    control for Induction Motors" in Proc. IEEE ECCE, Detroit, MI, Oct. 2022.
"""

# %%
from __future__ import annotations
from collections.abc import Callable
from dataclasses import dataclass, field
import numpy as np

from motulator.control.common import Ctrl, PWM, RateLimiter
from motulator.helpers import abc2complex, Bunch

# %%
@dataclass
class InductionMotorObsVHzCtrlPars:
    """Obs. V/Hz control parameters."""

    # Speed reference (in electrical rad/s)
    w_m_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: (t > .2)*(2*np.pi*50))

    # Control
    T_s: float = 250e-6
    psi_s_nom: float = 1.04  # 1 p.u.
    rate_limit: float = 2*np.pi*120
    i_s_max: float = 1.5 * np.sqrt(2) * 5
    alpha_f: float = 2*np.pi*1
    alpha_psi: float = 2*np.pi*20
    k_tau: float = 3.

    # Slip compensation
    slip_compensation: bool = True
    alpha_r: float = 2*np.pi*1

    # Observer
    alpha_o: float = 2*np.pi*40
    zeta_inf: float = .7

    # Motor parameter estimates (inverse-Γ model)
    R_s: float = 3.7
    R_R: float = 2.1
    L_sgm: float = .021
    L_M: float = .224
    p: int = 2


class InductionMotorVHzObsCtrl(Ctrl):
    """Observer-based V/Hz control for induction motors."""

    def __init__(self, pars):
        super().__init__()
        # Instantiate classes
        self.observer = SensorlessFluxObserver(pars)
        self.pwm = PWM(pars)
        self.rate_limiter = RateLimiter(pars)
        # Reference
        self.w_m_ref = pars.w_m_ref
        self.psi_s_ref = pars.psi_s_nom
        # Control Parameters
        self.T_s = pars.T_s
        self.alpha_f = pars.alpha_f
        self.alpha_r = pars.alpha_r
        self.alpha_psi = pars.alpha_psi
        self.p = pars.p
        self.k_tau = pars.k_tau
        self.i_s_max = pars.i_s_max
        self.slip_compensation = pars.slip_compensation
        # Motor parameters
        self.R_s = pars.R_s
        self.R_R = pars.R_R
        self.L_sgm = pars.L_sgm
        # Initial states
        self.theta_s, self.tau_M_ref, self.w_r_ref = 0, 0, 0

    def __call__(self, mdl):
        """
        Run the main control loop.

        Parameters
        ----------
        mdl : InductionMotorDrive
            Continuous-time model of an induction motor drive for getting the
            feedback signals.

        Returns
        -------
        T_s : float
            Sampling period.
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.

        """
        # Get the speed reference
        w_m_ref = self.rate_limiter(self.w_m_ref(self.t))

        # Measure the feedback signals
        i_s_abc = mdl.motor.meas_currents()  # Phase currents
        u_dc = mdl.conv.meas_dc_voltage()  # DC-bus voltage

        # Space vector and coordinate transformation
        i_s = np.exp(-1j * self.theta_s) * abc2complex(i_s_abc)

        # Get the states
        u_s = self.pwm.realized_voltage
        psi_R = self.observer.psi_R
        tau_M_ref = self.tau_M_ref
        w_r_ref = self.w_r_ref

        # Torque estimate (11c)
        tau_M = 1.5 * self.p * np.imag(i_s * np.conj(psi_R))

        # Slip frequency compensation (if enabled) for the low-pass filter.
        # Note, could also be based on the low-pass filtered torque.
        psi_R_sqr = np.abs(psi_R)**2
        if self.slip_compensation and psi_R_sqr > 0:
            w_r = self.R_R * tau_M / (1.5*self.p*psi_R_sqr)
        else:
            w_r = 0

        # Slip compensation. (9) Uses the low-pass filtered slip-estimate w_r_ref
        # Note if slip compensation disabled w_r_ref == 0
        w_s_ref = w_m_ref + w_r_ref

        # Dynamic frequency (7a)
        w_s = w_s_ref - self.k_tau * (tau_M - tau_M_ref)

        # State feedback
        u_s_ref, i_s_ref = self.state_feedback(i_s, psi_R, w_s)

        # Duty ratios
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc,
                                                 self.theta_s, w_s)

        # Data logging
        data = Bunch(
            i_s=i_s,
            psi_s=psi_R + self.L_sgm * i_s,
            psi_s_ref=self.psi_s_ref,
            t=self.t,
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
        self.pwm.update(u_s_ref_lim)
        self.observer.update(u_s, i_s, w_s)
        self.tau_M_ref += self.T_s*self.alpha_f*(tau_M - self.tau_M_ref)
        self.theta_s += self.T_s*w_s  # Next line: limit into [-pi, pi)
        self.theta_s = np.mod(self.theta_s + np.pi, 2*np.pi) - np.pi
        self.w_r_ref += self.T_s*self.alpha_r*(w_r - self.w_r_ref)
        self.update_clock(self.T_s)

        return self.T_s, d_abc_ref

    def state_feedback(self, i_s, psi_R, w_s):
        """
        Compute the stator voltage reference.

        """
        # Internal current reference for state feedback (6b)
        i_s_ref = (self.psi_s_ref - psi_R)/self.L_sgm
        # Limit the reference
        if np.abs(i_s_ref) > self.i_s_max:
            i_s_ref = self.i_s_max*i_s_ref/np.abs(i_s_ref)
        # State feedback (6a)
        u_s_ref = (self.R_s*i_s_ref + 1j*w_s*self.psi_s_ref
                   + self.L_sgm*self.alpha_psi*(i_s_ref - i_s))
        return u_s_ref, i_s_ref


class SensorlessFluxObserver:
    """
    Sensorless reduced-order flux observer.

    This observer is a variant of [1]_. The observer gain decouples the
    electrical and mechanical dynamics and allows placing the poles of the
    corresponding linearized estimation error dynamics. This implementation
    operates in controller coordinates.

    Parameters
    ----------
    pars : InductionMotorVHzObsCtrlPars
        Control parameters.

    Notes
    -----

    References
    ----------
    .. [1] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers with
       stator-resistance adaptation for speed-sensorless induction motor
       drives," IEEE Trans. Power Electron., 2010,
       https://doi.org/10.1109/TPEL.2009.2039650

    """

    # pylint: disable=too-many-instance-attributes, too-few-public-methods
    def __init__(self, pars):
        self.T_s = pars.T_s
        self.R_s = pars.R_s
        self.R_R = pars.R_R
        self.L_sgm = pars.L_sgm
        self.alpha = pars.R_R/pars.L_M
        self.alpha_o = pars.alpha_o
        self.zeta_inf = pars.zeta_inf
        # Initial states
        self.psi_R, self.i_s_old, self.w_m = 0j, 0j, 0

    def update(self, u_s, i_s, w_s):
        """
        Updates the states of the observer.

        Parameters
        ----------
        u_s : complex
            Stator voltage.
        i_s : complex
            Stator current.
        w_s : float
            Angular frequency of the reference frame.

        """
        # Decay rate
        lambd = self.zeta_inf*np.abs(w_s) + .5*self.alpha
        # Observer gain (without the orthogonal projection which is
        # embedded into the state update)
        g_o = 2*lambd*(self.alpha + 1j*self.w_m)/(self.alpha**2 + self.w_m**2)

        # Time derivative of the stator current
        di_s = (i_s - self.i_s_old)/self.T_s

        # Error voltage
        e = (self.L_sgm*(di_s + 1j*w_s*i_s) + (self.R_s + self.R_R)*i_s
             - (self.alpha - 1j*self.w_m)*self.psi_R - u_s)

        # Error signal
        psi_R_sqr = np.abs(self.psi_R)**2
        err = e*np.conj(self.psi_R)/psi_R_sqr if psi_R_sqr > 0 else 0

         # Update the states
        self.w_m -= self.T_s*self.alpha_o*err.imag
        self.psi_R += self.T_s*(u_s - self.R_s*i_s - self.L_sgm*di_s
                                - 1j*w_s*(self.psi_R + self.L_sgm*i_s)
                                + g_o*self.psi_R*err.real)
        self.i_s_old = i_s