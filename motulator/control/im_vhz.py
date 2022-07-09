# pylint: disable=C0103
'''
This module contains V/Hz control for induction motor drives.

The method is similar to [1]_. Open-loop V/Hz control can be obtained as a
special case by choosing::

    R_s, R_R = 0, 0
    k_u, k_w = 0, 0

Notes
-----
The low-pass-filtered values are marked with ref at the end of the variable
name. These slowly varying quasi-steady-state quantities can be seen to
represent the operating point (marked with the subscript 0 in [1]_).

References
----------
.. [1] Hinkkanen, Tiitinen, Mölsä, Harnefors, "On the stability of
   volts-per-hertz control for induction motors," IEEE J. Emerg. Sel. Topics
   Power Electron., 2022, https://doi.org/10.1109/JESTPE.2021.3060583

'''
# %%
from __future__ import annotations
from collections.abc import Callable
from dataclasses import dataclass, field
import numpy as np

from motulator.control.common import PWM, RateLimiter, Delay, Datalogger
from motulator.helpers import abc2complex, Bunch


# %%
@dataclass
class InductionMotorVHzCtrlPars:
    """
    V/Hz control parameters.

    """
    # pylint: disable=too-many-instance-attributes
    # Speed reference (in electrical rad/s)
    w_m_ref: Callable[[float], float] = field(
        repr=False, default=lambda t: (t > .2)*(2*np.pi*50))
    sensorless: bool = field(repr=False, default=True)  # Always sensorless
    T_s: float = 250e-6
    delay: int = 1
    psi_s_nom: float = 1.04  # 1 p.u.
    rate_limit: float = 2*np.pi*120
    # Motor parameter estimates
    R_s: float = 3.7
    R_R: float = 2.1
    L_sgm: float = .021
    L_M: float = .224
    k_u: float = 1
    k_w: float = 4


# %%
class InductionMotorVHzCtrl(Datalogger):
    """
    V/Hz control with the stator current feedback.

    Parameters
    ----------
    pars : InductionMotorVHzCtrlPars
        Control parameters.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, pars):
        super().__init__()
        self.t = 0
        self.T_s = pars.T_s
        self.w_m_ref = pars.w_m_ref
        self.sensorless = True
        # Instantiate classes
        self.pwm = PWM(pars)
        self.rate_limiter = RateLimiter(pars)
        self.delay = Delay(pars.delay)
        # Parameters
        self.k_u = pars.k_u
        self.k_w = pars.k_w
        self.psi_s_ref = pars.psi_s_nom
        self.R_s = pars.R_s
        self.R_R = pars.R_R
        self.L_sgm = pars.L_sgm
        self.L_M = pars.L_M
        w_rb = pars.R_R*(pars.L_M + pars.L_sgm)/(pars.L_sgm*pars.L_M)
        self.alpha_f: float = .1*w_rb
        self.alpha_i: float = .1*w_rb
        # States
        self.i_s_ref = 0j
        self.theta_s = 0
        self.w_r_ref = 0
        self.desc = pars.__repr__()

    def __call__(self, mdl):
        """
        Main control loop.

        Parameters
        ----------
        mdl : InductionMotorDrive
            Continuous-time model of an induction motor drive for getting the
            feedback signals.

        Returns
        -------
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.
        T_s : float
            Sampling period.

        """
        # Rate limit the frequency reference
        w_m_ref = self.rate_limiter(self.w_m_ref(self.t))

        # Measure the feedback signals
        i_s_abc = mdl.motor.meas_currents()  # Phase currents
        u_dc = mdl.conv.meas_dc_voltage()  # DC-bus voltage

        # Space vector transformation
        i_s = np.exp(-1j*self.theta_s)*abc2complex(i_s_abc)

        # Slip compensation
        w_s_ref = w_m_ref + self.w_r_ref

        # Dynamic stator frequency and slip frequency
        w_s, w_r = self.stator_freq(w_s_ref, i_s)

        # Voltage reference
        u_s_ref = self.voltage_reference(w_s, i_s)

        # Compute the duty ratios
        u_s = self.pwm.realized_voltage  # Used only for datalogging
        d_abc_ref = self.pwm(u_s_ref, u_dc, self.theta_s, w_s)

        # Data logging
        data = Bunch(i_s_ref=self.i_s_ref, i_s=i_s, u_s=u_s, w_m_ref=w_m_ref,
                     w_r=w_r, w_s=w_s, psi_s_ref=self.psi_s_ref,
                     theta_s=self.theta_s, u_dc=u_dc, t=self.t)
        self.save(data)

        # Update the states
        self.i_s_ref += self.T_s*self.alpha_i*(i_s - self.i_s_ref)
        self.w_r_ref += self.T_s*self.alpha_f*(w_r - self.w_r_ref)
        self.theta_s += self.T_s*w_s
        self.theta_s = np.mod(self.theta_s, 2*np.pi)    # Limit to [0, 2*pi]
        self.t += self.T_s

        return d_abc_ref, self.T_s

    def stator_freq(self, w_s_ref, i_s):
        """
        Compute the dynamic stator frequency.

        This computes the dynamic stator frequency reference used in the
        coordinate transformations.

        """
        # Operating-point quantities
        psi_R_ref = self.psi_s_ref - self.L_sgm*self.i_s_ref
        psi_R_ref_sqr = np.abs(psi_R_ref)**2
        # Compute the dynamic stator frequency
        if psi_R_ref_sqr > 0:
            # Slip estimate based on the measured current
            w_r = self.R_R*np.imag(i_s*np.conj(psi_R_ref))/psi_R_ref_sqr
            # Dynamic frequency
            w_s = w_s_ref + self.k_w*(self.w_r_ref - w_r)
        else:
            w_s, w_r = 0, 0
        return w_s, w_r

    def voltage_reference(self, w_s, i_s):
        """
        Compute the stator voltage reference.

        """
        # Nominal magnetizing current
        i_sd_nom = self.psi_s_ref/(self.L_M + self.L_sgm)
        # Operating-point current for RI compensation
        i_s_ref0 = i_sd_nom + 1j*self.i_s_ref.imag
        # Term -R_s omitted to avoid problems due to the voltage saturation
        # k = -R_s + k_u*L_sgm*(alpha + 1j*w_m0)
        k = self.k_u*self.L_sgm*(self.R_R/self.L_M + 1j*w_s)
        u_s_ref = (self.R_s*i_s_ref0 + 1j*w_s*self.psi_s_ref
                   + k*(self.i_s_ref - i_s))
        return u_s_ref

    def __repr__(self):
        return self.desc
