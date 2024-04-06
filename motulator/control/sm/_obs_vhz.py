"""Observer-based V/Hz control of synchronous motor drives."""

from dataclasses import dataclass, InitVar
import numpy as np
from motulator._helpers import abc2complex
from motulator.control._common import Ctrl, PWM, RateLimiter
from motulator.control.sm._flux_vector import (
    FluxTorqueReference, FluxTorqueReferencePars)
from motulator.control.sm._observers import FluxObserver
from motulator._utils import Bunch


# %%
# pylint: disable=too-many-instance-attributes
@dataclass
class ObserverBasedVHzCtrlPars(FluxTorqueReferencePars):
    """
    Parameters for the control system.

    This class extends FluxTorqueReferencePars with the parameters needed for
    the observer-based V/Hz control.

    Parameters
    ----------
    alpha_psi : float, optional
        Flux control bandwidth (rad/s). The default is 2*pi*50.
    alpha_tau : float
        Torque control bandwidth (rad/s). The default is 2*pi*50.
    alpha_f : float, optional
        Bandwidth of the high-pass filter (rad/s). The default is 2*pi*1.

    """
    alpha_psi: float = 2*np.pi*50
    alpha_tau: InitVar[float] = 2*np.pi*50
    alpha_f: float = 2*np.pi*1

    # pylint: disable=arguments-differ
    def __post_init__(self, par, alpha_tau):
        super().__post_init__(par)
        # Gain k_tau
        G = (par.L_d - par.L_q)/(par.L_d*par.L_q)
        psi_s0 = par.psi_f if par.psi_f > 0 else self.psi_s_min
        if par.psi_f > 0:  # PMSM or PM-SyRM
            c_delta0 = 1.5*par.n_p*(par.psi_f*psi_s0/par.L_d - G*psi_s0**2)
        else:  # SyRM
            c_delta0 = 1.5*par.n_p*G*psi_s0**2
        self.k_tau = alpha_tau/c_delta0


# %%
class ObserverBasedVHzCtrl(Ctrl):
    """
    Observer-based V/Hz control for synchronous motors.

    This observer-based V/Hz control control method is based on [#Tii2022]_.

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
    w_m_ref : callable
        Rotor speed reference (electrical rad/s).

    References
    ----------
    .. [#Tii2022] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors, 
       "Stable and passive observer-based V/Hz control for synchronous Motors," 
       Proc. IEEE ECCE, 2022, https://doi.org/10.1109/ECCE50734.2022.9947858

    """

    # pylint: disable=too-many-instance-attributes, too-many-arguments
    def __init__(self, par, ctrl_par, T_s=250e-6):
        super().__init__()
        self.T_s = T_s
        # Motor parameter estimates
        self.R_s = par.R_s
        self.n_p = par.n_p
        # Controller parameters
        self.alpha_f = ctrl_par.alpha_f
        self.alpha_psi = ctrl_par.alpha_psi
        self.k_tau = ctrl_par.k_tau
        # Subsystems
        self.pwm = PWM()
        self.flux_torque_ref = FluxTorqueReference(ctrl_par)
        self.observer = FluxObserver(par)
        self.rate_limiter = RateLimiter(np.inf)
        self.w_m_ref = callable
        # States
        self.theta_s, self.tau_M_ref = 0, 0

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
        if self.rate_limiter is not None:
            w_m_ref = self.rate_limiter(self.T_s, self.w_m_ref(self.clock.t))
        else:
            w_m_ref = self.w_m_ref(self.clock.t)

        # Feedback signals
        i_s_abc = mdl.machine.meas_currents()  # Phase currents
        u_dc = mdl.converter.meas_dc_voltage()  # DC-bus voltage
        u_s = self.pwm.realized_voltage  # Realized voltage from PWM

        # Get the states
        psi_s = self.observer.psi_s
        theta_s = self.theta_s

        # Space vector and coordinate transformation
        i_s = np.exp(-1j*theta_s)*abc2complex(i_s_abc)

        # Get the states
        tau_M_ref = self.tau_M_ref

        # Limited flux and torque references
        psi_s_ref, _ = self.flux_torque_ref(tau_M_ref, w_m_ref, u_dc)

        # Electromagnetic torque
        tau_M = 1.5*self.n_p*np.imag(i_s*np.conj(psi_s))

        # Dynamic frequency
        w_s = w_m_ref - self.k_tau*(tau_M - tau_M_ref)

        # Voltage reference
        err = psi_s_ref - psi_s
        u_s_ref = self.R_s*i_s + 1j*w_s*psi_s_ref + self.alpha_psi*err

        # Data logging
        data = Bunch(
            i_s=i_s,
            psi_s=psi_s,
            psi_s_ref=psi_s_ref,
            t=self.clock.t,
            theta_s=theta_s,
            u_dc=u_dc,
            u_s=u_s,
            w_m_ref=w_m_ref,
            w_s=w_s,
            tau_M=tau_M,
        )
        self.save(data)

        # Update the states
        self.observer.update(self.T_s, u_s, i_s, w_s)
        self.tau_M_ref += self.T_s*self.alpha_f*(tau_M - self.tau_M_ref)
        self.theta_s += self.T_s*w_s  # Next line: limit into [-pi, pi)
        self.theta_s = np.mod(self.theta_s + np.pi, 2*np.pi) - np.pi
        self.clock.update(self.T_s)

        # PWM output
        d_abc = self.pwm(self.T_s, u_s_ref, u_dc, theta_s, w_s)

        return self.T_s, d_abc
