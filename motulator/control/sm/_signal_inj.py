"""Sensorless control with signal injection for synchronous machine drives."""

import numpy as np
from motulator._helpers import abc2complex
from motulator.control._common import Ctrl, PWM, SpeedCtrl
from motulator.control.sm._vector import CurrentCtrl, CurrentReference
from motulator._utils import Bunch


# %%
class SignalInjectionCtrl(Ctrl):
    """
    Sensorless control with signal injection for synchronous machine drives.

    This class implements a square-wave signal injection for low-speed 
    operation according to [#Kim2012]_. A phase-locked loop is used to track 
    the rotor position. 
    
    Notes
    -----
    For a wider speed range, signal injection could be combined to a 
    model-based observer. The effects of magnetic saturation are not
    compensated for in this version.

    References
    ----------
    .. [#Kim2012] Kim, Ha, Sul, "PWM switching frequency signal injection 
       sensorless method in IPMSM," IEEE Trans. Ind. Appl., 2012,
       https://doi.org/10.1109/TIA.2012.2210175

    Parameters
    ----------
    T_s : float
        Sampling period (s).
    pars : ModelPars
        Machine model parameters.
    U_inj : float
        Amplitude of the injected voltage (V).
    w_o : float
        PLL natural frequency (rad/s).

    Attributes
    ----------
    current_ctrl : CurrentCtrl
        Current controller.
    speed_ctrl : SpeedCtrl
        Speed controller.
    current_ref : CurrentReference
        Current reference generator.
    pll : PhaseLockedLoop
        Phase-locked loop.
    signal_inj : SignalInjection
        Signal injection.
    w_m_ref : callable
        Speed reference (electrical rad/s).
    pwm : PWM
        Pulse-width modulation.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, par, ref, T_s=250e-6):
        super().__init__()
        self.T_s = T_s
        self.n_p = par.n_p
        self.current_ref = CurrentReference(par, ref)
        self.current_ctrl = CurrentCtrl(par, 2*np.pi*200)
        self.pll = PhaseLockedLoop(w_o=2*np.pi*40)
        self.signal_inj = SignalInjection(par, U_inj=250)
        self.speed_ctrl = SpeedCtrl(par.J, 2*np.pi*4)
        self.pwm = PWM()
        self.w_m_ref = callable

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

        # Measure the feedback signals
        i_s_abc = mdl.machine.meas_currents()  # Phase currents
        u_dc = mdl.converter.meas_dc_voltage()  # DC-bus voltage
        u_s = self.pwm.realized_voltage

        # Get the rotor speed and position estimates
        w_m, theta_m = self.pll.w_m, self.pll.theta_m

        # Current vector in estimated rotor coordinates
        i_s = np.exp(-1j*theta_m)*abc2complex(i_s_abc)

        # Filter the current measurement for the current controller
        i_s_filt = self.signal_inj.filter_current(i_s)

        # Outputs
        tau_M_ref = self.speed_ctrl.output(w_m_ref/self.n_p, w_m/self.n_p)
        i_s_ref, tau_M_ref_lim = self.current_ref.output(tau_M_ref, w_m, u_dc)
        err = self.signal_inj.output(self.T_s, i_s.imag)
        # Superimpose the excitation voltage on the d-axis
        u_s_ref = self.current_ctrl.output(
            i_s_ref, i_s_filt) + self.signal_inj.u_sd_inj

        # Data logging
        data = Bunch(
            i_s=i_s_filt,
            i_s_ref=i_s_ref,
            t=self.clock.t,
            tau_M_ref_lim=tau_M_ref_lim,
            theta_m=theta_m,
            u_dc=u_dc,
            u_s=u_s,
            w_m=w_m,
            w_m_ref=w_m_ref,
        )
        self.save(data)

        # Update states
        self.speed_ctrl.update(self.T_s, tau_M_ref_lim)
        self.current_ref.update(self.T_s, tau_M_ref_lim, u_s_ref, u_dc)
        self.current_ctrl.update(self.T_s, u_s, w_m)
        self.signal_inj.update(i_s)
        self.pll.update(self.T_s, err)
        self.clock.update(self.T_s)

        # PWM output
        d_abc = self.pwm(self.T_s, u_s_ref, u_dc, theta_m, w_m)

        return self.T_s, d_abc


# %%
class SignalInjection:
    """
    Estimate the rotor position error based on signal injection.

    This signal injection method estimates the rotor position error based on
    the injected switching frequency signal. The estimate can be used in a 
    phase-locked loop or in a state observer to robustify low-speed sensorless 
    operation.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    U_inj : float
        Injected voltage amplitude (V).

    """

    def __init__(self, par, U_inj):
        # Error gain
        self.k = .5*par.L_d*par.L_q/((par.L_q - par.L_d))
        # States
        self._i_s_old, self._i_s_older = 0, 0
        self.u_sd_inj = U_inj

    def output(self, T_s, i_sq):
        """
        Compute the rotor position estimation error.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        i_sq : float
            q-axis stator current (A) in estimated rotor coordinates.

        Returns
        -------
        err : float
            Rotor position estimation error (electrical rad).

        """
        di_sq = i_sq - 2*self._i_s_old.imag + self._i_s_older.imag
        err = (self.k/T_s)*di_sq/self.u_sd_inj if np.abs(
            self.u_sd_inj) > 0 else 0
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
        self._i_s_older = self._i_s_old
        self._i_s_old = i_s
        # Reverse the d-axis square wave injection voltage
        self.u_sd_inj = -self.u_sd_inj

    def filter_current(self, i_s):
        """
        Filter the stator current using the previously measured value.

        Parameters
        ----------
        i_s : complex
            Unfiltered stator current (A) in estimated rotor coordinates.

        Returns
        -------
        i_s_filt : complex
            Filtered stator current (A) in estimated rotor coordinates.

        """
        # Filter currents
        i_s_filt = .5*(i_s + self._i_s_old)
        return i_s_filt


# %%
class PhaseLockedLoop:
    """
    Simple phase-locked loop for rotor-position estimation.

    Parameters
    ----------
    w_o : float
        Natural frequency (rad/s).

    """

    def __init__(self, w_o):
        # Gains
        self.k_p = w_o
        self.k_i = w_o**2
        # States
        self.theta_m, self.w_m = 0, 0

    def update(self, T_s, err):
        """
        Update the states for the next sampling period.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        err : float
            Rotor position error (rad).

        """
        # Speed estimation
        w_m = self.k_p*err + self.w_m

        # Update the states
        self.w_m += T_s*self.k_i*err
        self.theta_m += T_s*w_m  # Next line: limit into [-pi, pi)
        self.theta_m = np.mod(self.theta_m + np.pi, 2*np.pi) - np.pi
