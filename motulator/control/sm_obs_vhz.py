"""
Observer-based V/Hz control for synchronous motor drives corresponding to [1.]_.

References
----------
.. [1] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors, "Stable and passive
    observer-based V/Hz control for synchronous Motors" in Proc.
    IEEE ECCE, Detroit, MI, Oct. 2022.
"""

from collections.abc import Callable
from dataclasses import dataclass, field
import numpy as np

from motulator.helpers import abc2complex, Bunch
from motulator.control.common import Ctrl, PWM, RateLimiter

@dataclass
class SynchronousMotorVHzObsCtrlPars:
    """Obs. V/Hz control parameters of synchronous motors."""

    # Speed reference (in electrical rad/s)
    w_m_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: (t > .2)*(2*np.pi*75))

    # Control
    T_s: float = 250e-6
    psi_s_nom: float = np.sqrt(2/3)*370/(2*np.pi*75)
    rate_limit: float = 2*np.pi*120*10

    alpha_psi: float = 2*np.pi*50
    alpha_f: float = 2*np.pi*1
    k_tau: float = 3.

    # Observer
    alpha_o: float = 2*np.pi*20
    zeta_inf: float = .7

    # Motor parameter estimates
    R_s: float = 3.6
    L_d: float = .036
    L_q: float = .051
    psi_f: float = .545
    p: int = 3


class SynchronousMotorVHzObsCtrl(Ctrl):
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
        # Parameters
        self.T_s = pars.T_s
        self.alpha_f = pars.alpha_f
        self.alpha_psi = pars.alpha_psi
        self.p = pars.p
        self.k_tau = pars.k_tau
        # Motor parameters
        self.R_s = pars.R_s
        # Initial states
        self.theta_s, self.tau_M_ref = 0, 0

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
        w_m_ref = self.rate_limiter(self.w_m_ref(self.t))

        # Measure the feedback signals
        i_s_abc = mdl.motor.meas_currents()  # Phase currents
        u_dc = mdl.conv.meas_dc_voltage()  # DC-bus voltage

        # Space vector and coordinate transformation
        i_s = np.exp(-1j*self.theta_s)*abc2complex(i_s_abc)

        # Get the states
        u_s = self.pwm.realized_voltage
        psi_s = self.observer.psi_s
        tau_M_ref = self.tau_M_ref

        # Electromagnetic torque (7d)
        tau_M = 1.5*self.p*np.imag(i_s*np.conj(psi_s))

        # Dynamic frequency (5a)
        w_s = w_m_ref - self.k_tau * (tau_M - tau_M_ref)

        # Voltage reference (4)
        u_s_ref = self.R_s*i_s + 1j*w_s*self.psi_s_ref + self.alpha_psi*(self.psi_s_ref - psi_s)

        # Duty ratios
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc,
                                                 self.theta_s, w_s)
        # Data logging
        data = Bunch(
            i_s=i_s,
            psi_s=psi_s,
            psi_s_ref=self.psi_s_ref,
            t=self.t,
            theta_s=self.theta_s,
            u_dc=u_dc,
            u_s=u_s,
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
        self.update_clock(self.T_s)

        return self.T_s, d_abc_ref


class SensorlessFluxObserver:
    """
    Sensorless stator flux observer.

    This observer is a variant of [1]_. The observer gain decouples the
    electrical and mechanical dynamics and allows placing the poles of the
    corresponding linearized estimation error dynamics.

    Parameters
    ----------
    pars : SynchronousMotorObsVHzCtrlPars
        Control parameters.

    References
    ----------
    .. [1] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
       sensorless synchronous motor drives: Framework for design and analysis,"
       IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

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
        self.psi_f = pars.psi_f
        self.alpha_o = pars.alpha_o
        self.b_p = .5*pars.R_s*(pars.L_d + pars.L_q)/(pars.L_d*pars.L_q)
        self.zeta_inf = pars.zeta_inf
        # Initial states
        self.delta, self.psi_s = 0, pars.psi_f

    def update(self, u_s, i_s, w_s):
        """
        Update the states for the next sampling period.
        Parameters
        ----------
        u_s : complex
            Stator voltage.
        i_s : complex
            Stator current.
        w_s : float
            Stator angular frequency.
        """
        # Inductance matrix elements
        L_x = self.L_d*np.cos(self.delta)**2 + self.L_q*np.sin(self.delta)**2
        L_y = self.L_d*np.sin(self.delta)**2 + self.L_q*np.cos(self.delta)**2
        L_xy = .5*(self.L_q - self.L_d)*np.sin(2*self.delta)

        # PM-flux linkage
        psi_F = self.psi_f*np.exp(-1j*self.delta)

        # Auxiliary flux (12)
        psi_a = psi_F + (L_x - L_y)*np.conj(i_s) + 2j*L_xy*np.conj(i_s)

        # Estimation error (6)
        e = (L_x*i_s.real + 1j*L_y*i_s.imag + 1j*L_xy*np.conj(i_s) + psi_F
             - self.psi_s)

        # Pole locations are chosen according to (36), with c = w_m**2
        # and w_inf = inf, and the gain corresponding to (30) is used
        k = self.b_p + 2*self.zeta_inf*np.abs(w_s)
        psi_a_sqr = np.abs(psi_a)**2
        if psi_a_sqr > 0:
            # Correction voltage
            v = k*psi_a*np.real(psi_a*np.conj(e))/psi_a_sqr
            # Error signal (10)
            w_delta = -self.alpha_o*np.imag(psi_a*np.conj(e))/psi_a_sqr
        else:
            v, w_delta = 0, 0

        # Update the states
        self.psi_s += self.T_s*(u_s - self.R_s*i_s - 1j*w_s*self.psi_s + v)
        self.delta += self.T_s*w_delta

