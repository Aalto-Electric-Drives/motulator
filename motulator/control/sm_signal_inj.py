# pylint: disable=invalid-name
"""
Sensorless control with signal injection for synchronous motor drives.

This module contains a simple example of square-wave signal injection for low-
speed operation. A phase-locked loop is used to track the rotor position. For
a wider speed range, signal injection could be combined to a model-based
observer. The effects of magnetic saturation are not compensated for in this
version.

"""

from __future__ import annotations
from dataclasses import dataclass
import numpy as np

from motulator.helpers import abc2complex, Bunch
from motulator.control.common import Ctrl, SpeedCtrl, PWM
from motulator.control.sm_vector import (
    CurrentCtrl, CurrentRef, SynchronousMotorVectorCtrlPars)


# %%
@dataclass
class SynchronousMotorSignalInjectionCtrlPars(SynchronousMotorVectorCtrlPars):
    """Square-wave signal injection parameters."""

    U_inj: float = 200


# %%
class SynchronousMotorSignalInjectionCtrl(Ctrl):
    """
    Sensorless control with signal injection for a synchronous motor drive.

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
        self.current_ctrl = CurrentCtrl(pars)
        self.speed_ctrl = SpeedCtrl(pars)
        self.current_ref = CurrentRef(pars)
        self.pwm = PWM(pars)
        self.pll = PhaseLockedLoop(pars)
        self.signal_inj = SignalInjection(pars)

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

        # Get the rotor speed and position estimates
        w_m, theta_m = self.pll.w_m, self.pll.theta_m

        # Current vector in estimated rotor coordinates
        i_s = np.exp(-1j*theta_m)*abc2complex(i_s_abc)

        # Filter the current measurement for the current controller
        i_s_filt = self.signal_inj.filter_current(i_s)

        # Outputs
        tau_M_ref = self.speed_ctrl.output(w_m_ref/self.p, w_m/self.p)
        i_s_ref, tau_M_ref_lim = self.current_ref.output(tau_M_ref, w_m, u_dc)
        err = self.signal_inj.output(i_s.imag)
        # Superimpose the excitation voltage on the d-axis
        u_s_ref = self.current_ctrl.output(
            i_s_ref, i_s_filt) + self.signal_inj.u_sd_inj
        d_abc_ref, u_s_ref_lim = self.pwm.output(u_s_ref, u_dc, theta_m, w_m)

        # Data logging
        data = Bunch(
            i_s=i_s_filt,
            i_s_ref=i_s_ref,
            t=self.t,
            tau_M_ref_lim=tau_M_ref_lim,
            theta_m=theta_m,
            u_dc=u_dc,
            u_s=u_s_ref_lim,
            w_m=w_m,
            w_m_ref=w_m_ref,
        )
        self.save(data)

        # Update states
        self.speed_ctrl.update(tau_M_ref_lim)
        self.current_ref.update(tau_M_ref_lim, u_s_ref, u_dc)
        self.current_ctrl.update(u_s_ref_lim, w_m)
        self.pwm.update(u_s_ref_lim)
        self.signal_inj.update(i_s)
        self.pll.update(err)
        self.update_clock(self.T_s)

        return self.T_s, d_abc_ref


# %%
class SignalInjection:
    """
    Estimate the rotor position error based on signal injection.

    This signal injection method estimates the rotor position error based on
    the injected switching frequency signal, according to [1]_. The estimate
    can be used in a phase-locked loop or in a state observer to robustify
    low-speed sensorless operation.

    Parameters
    ----------
    pars : SynchronousMotorSignalInjectionCtrlPars
        Control parameters.

    References
    ----------
    .. [1] Kim, Ha, Sul, "PWM switching frequency signal injection sensorless
       method in IPMSM," IEEE Trans. Ind. Appl., 2012,
       https://doi.org/10.1109/TIA.2012.2210175

    """

    def __init__(self, pars):
        self.u_sd_inj = pars.U_inj
        # Initial states
        self.i_s_old, self.i_s_older = 0, 0
        # Error gain
        self.k = .5*pars.L_d*pars.L_q/((pars.L_q - pars.L_d)*pars.T_s)

    def output(self, i_sq):
        """
        Compute the rotor position estimation error.

        Parameters
        ----------
        i_sq : float
            Stator current q-component in estimated rotor coordinates.

        Returns
        -------
        err : float
            Rotor position estimation error.

        """
        di_sq = i_sq - 2*self.i_s_old.imag + self.i_s_older.imag
        err = self.k*di_sq/self.u_sd_inj if np.abs(self.u_sd_inj) > 0 else 0
        return err

    def update(self, i_s):
        """
        Store the old current values for the next sampling period.

        Parameters
        ----------
        i_s : complex
            Stator current in estimated rotor coordinates.

        """
        # Update the integral states
        self.i_s_older = self.i_s_old
        self.i_s_old = i_s
        # Reverse the d-axis square wave injection voltage
        self.u_sd_inj = -self.u_sd_inj

    def filter_current(self, i_s):
        """
        Filter the stator current using the previously measured value.

        Parameters
        ----------
        i_s : complex
            Unfiltered stator current in estimated rotor coordinates.

        Returns
        -------
        i_s_filt : complex
            Filtered stator current in estimated rotor coordinates.

        """
        # Filter currents
        i_s_filt = .5*(i_s + self.i_s_old)
        return i_s_filt


# %%
class PhaseLockedLoop:
    """
    Simple phase-locked loop for rotor-position estimation.

    Parameters
    ----------
    pars : SynchronousMotorVectorCtrlPars
        Control parameters.

    """

    def __init__(self, pars):
        self.T_s = pars.T_s
        # Gains
        self.k_p = 2*pars.w_o
        self.k_i = pars.w_o**2
        # # Initial states
        self.theta_m, self.w_m = 0, 0

    def update(self, err):
        """
        Update the states for the next sampling period.

        Parameters
        ----------
        err : Rotor position error.

        """
        # Speed estimation
        w_m = self.k_p*err + self.w_m

        # Update the states
        self.w_m += self.T_s*self.k_i*err
        self.theta_m += self.T_s*w_m  # Next line: limit into [-pi, pi)
        self.theta_m = np.mod(self.theta_m + np.pi, 2*np.pi) - np.pi
