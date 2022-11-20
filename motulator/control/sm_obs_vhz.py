# pylint: disable=invalid-name
"""
Observer-based V/Hz control for synchronous motor drives.

This method is based on [1]_.

References
----------
.. [1] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors, "Stable
    and passive observer-based V/Hz control for synchronous Motors," in Proc.
    IEEE ECCE, Detroit, MI, Oct. 2022.

"""

from typing import Callable
from dataclasses import dataclass, field
import numpy as np

from motulator.helpers import abc2complex, Bunch
from motulator.control.common import Ctrl, PWM, RateLimiter
from motulator.control.sm_torque import TorqueCharacteristics


# %%
@dataclass
class SynchronousMotorVHzObsCtrlPars:
    """Control parameters."""

    # pylint: disable=too-many-instance-attributes
    # Speed reference (in electrical rad/s)
    w_m_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: (t > .2)*(2*np.pi*75))

    # Control
    T_s: float = 250e-6
    psi_s_max: float = np.sqrt(2/3)*370/(2*np.pi*75)
    psi_s_min: float = .5*np.sqrt(2/3)*370/(2*np.pi*75)
    rate_limit: float = np.inf
    i_s_max: float = 1.5*np.sqrt(2)*5
    alpha_psi: float = 2*np.pi*50
    alpha_tau: float = 2*np.pi*50
    alpha_f: float = 2*np.pi*1
    k_u: float = 1

    # Observer
    alpha_o: float = 2*np.pi*20
    zeta_inf: float = .7

    # Motor parameter estimates
    R_s: float = 3.6
    L_d: float = .036
    L_q: float = .051
    psi_f: float = .545
    p: int = 3


# %%
class SynchronousMotorVHzObsCtrl(Ctrl):
    """
    Observer-based V/Hz control for synchronous motors.

    Parameters
    ----------
    pars : SynchronousMotorVHzObsCtrlPars
        Control parameters.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, pars):
        super().__init__()
        # Instantiate classes
        self.observer = SensorlessFluxObserver(pars)
        self.pwm = PWM(pars)
        self.rate_limiter = RateLimiter(pars)
        self.flux_torque_ref = FluxTorqueRef(pars)
        # Reference
        self.w_m_ref = pars.w_m_ref
        # Motor parameters
        self.R_s = pars.R_s
        self.p = pars.p
        # Controller parameters
        self.T_s = pars.T_s
        self.alpha_f = pars.alpha_f
        self.alpha_psi = pars.alpha_psi
        # Gain k_tau
        G = (pars.L_d - pars.L_q)/(pars.L_d*pars.L_q)
        if pars.psi_f > 0:  # PMSM or PM-SyRM
            # c_delta0 is c_delta @ psi_s = psi_s_min and delta = 0
            c_delta0 = 1.5*pars.p*(
                pars.psi_f*pars.psi_s_min/pars.L_d - G*pars.psi_s_min**2)
        else:  # SyRM
            c_delta0 = 1.5*pars.p*G*pars.psi_s_min**2
        self.k_tau = pars.alpha_tau/c_delta0
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

        # Limited flux and torque references
        psi_s_ref, tau_M_ref_lim = self.flux_torque_ref(
            tau_M_ref, w_m_ref, u_dc)

        # Electromagnetic torque (7d)
        tau_M = 1.5*self.p*np.imag(i_s*np.conj(psi_s))

        # Dynamic frequency (5a)
        w_s = w_m_ref - self.k_tau*(tau_M - tau_M_ref_lim)

        # Voltage reference (4)
        err = psi_s_ref - psi_s
        u_s_ref = self.R_s*i_s + 1j*w_s*psi_s_ref + self.alpha_psi*err

        # Duty ratios
        d_abc_ref, u_s_ref_lim = self.pwm.output(
            u_s_ref, u_dc, self.theta_s, w_s)

        # Data logging
        data = Bunch(
            i_s=i_s,
            psi_s=psi_s,
            psi_s_ref=psi_s_ref,
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


# %%
class FluxTorqueRef:
    """
    Flux and torque references.

    Parameters
    ----------
    pars : SynchronousMotorVHzObsCtrlPars
        Control parameters.

    """

    # pylint: disable=too-few-public-methods
    def __init__(self, pars):
        self.psi_s_min = pars.psi_s_min
        try:
            self.psi_s_max = pars.psi_s_max
        except AttributeError:
            self.psi_s_max = np.inf
        self.k_u = pars.k_u
        # Merged MTPV and current limits
        tq = TorqueCharacteristics(pars)
        lims = tq.mtpv_and_current_limits(i_s_max=pars.i_s_max)
        self.tau_M_lim = lims.tau_M_vs_abs_psi_s
        # MTPA locus
        mtpa = tq.mtpa_locus(i_s_max=pars.i_s_max)
        self.psi_s_mtpa = mtpa.abs_psi_s_vs_tau_M

    def __call__(self, tau_M_ref, w_m, u_dc):
        """
        Calculate the stator flux reference and limit the torque reference.

        Parameters
        ----------
        tau_M_ref : float
            Unlimited torque reference.
        w_m : float
            Rotor speed or its reference (in electrical rad/s).
        u_dc : float
            DC-bus voltage.

        Returns
        -------
        psi_s_ref : float
            Stator flux reference.
        tau_M_ref_lim : float
            Limited torque reference.

        """
        # Get the MTPA flux
        psi_s_mtpa = self.psi_s_mtpa(np.abs(tau_M_ref))
        np.clip(psi_s_mtpa, self.psi_s_min, self.psi_s_max, psi_s_mtpa)

        # Field weakening
        u_s_max = self.k_u*u_dc/np.sqrt(3)
        psi_s_max = u_s_max/np.abs(w_m) if np.abs(w_m) > 0 else np.inf

        # Flux reference
        psi_s_ref = np.min([psi_s_max, psi_s_mtpa])

        # Limit the torque reference according to the MTPV and current limits
        tau_M_lim = self.tau_M_lim(psi_s_ref)
        tau_M_ref_lim = np.min([tau_M_lim, np.abs(tau_M_ref)
                                ])*np.sign(tau_M_ref)

        return psi_s_ref, tau_M_ref_lim


# %%
class SensorlessFluxObserver:
    """
    Sensorless stator flux observer.

    This observer is a variant of [1]_. The observer gain decouples the
    electrical and mechanical dynamics and allows placing the poles of the
    corresponding linearized estimation error dynamics. For simplicity, the
    current model is here implemented in rotor coordinates, however this is
    mathematically equivalent to controller coordinates implementation in [2]_.

    Parameters
    ----------
    pars : SynchronousMotorObsVHzCtrlPars
        Control parameters.

    References
    ----------
    .. [1] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
        sensorless synchronous motor drives: Framework for design and
        analysis," IEEE Trans. Ind. Appl., 2018,
        https://doi.org/10.1109/TIA.2018.2858753
    .. [2] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors,
        "Stable and passive observer-based V/Hz control for synchronous
        Motors," in Proc. IEEE ECCE, Detroit, MI, Oct. 2022.

    """

    def __init__(self, pars):
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
        # Transformations to rotor coordinates
        i_sr = i_s*np.exp(1j*self.delta)
        psi_sr = self.psi_s*np.exp(1j*self.delta)

        # Auxiliary flux and estimation error in rotor coordinates
        psi_ar = self.psi_f + (self.L_d - self.L_q)*np.conj(i_sr)
        e_r = self.L_d*i_sr.real + 1j*self.L_q*i_sr.imag + self.psi_f - psi_sr

        # Auxiliary flux in controller coordinates
        psi_a = np.exp(-1j*self.delta)*psi_ar

        g_o = self.b_p + 2*self.zeta_inf*np.abs(w_s)

        if np.abs(psi_ar) > 0:
            # Correction voltage in controller coordinates
            v = g_o*psi_a*np.real(e_r/psi_ar)
            # Error signal
            w_delta = self.alpha_o*np.imag(e_r/psi_ar)
        else:
            v, w_delta = 0, 0

        # Update the states
        self.psi_s += self.T_s*(u_s - self.R_s*i_s - 1j*w_s*self.psi_s + v)
        self.delta += self.T_s*w_delta
