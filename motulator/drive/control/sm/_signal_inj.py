"""Sensorless control with signal injection for synchronous machine drives."""

from types import SimpleNamespace
import numpy as np

from motulator.drive.control import DriveControlSystem, SpeedController
from motulator.drive.control.sm._current_vector import (
    CurrentController,
    CurrentReference,
)
from motulator.common.utils import wrap


# %%
class SignalInjectionControl(DriveControlSystem):
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
    par : SynchronousMachinePars
        Machine model parameters.
    cfg : CurrentReferenceCfg
        Reference generation configuration.
    J : float, optional
        Moment of inertia (kgm²). Needed only for the speed controller.
    T_s : float
        Sampling period (s).

    """

    def __init__(self, par, cfg, J=None, T_s=250e-6):
        super().__init__(par, T_s, sensorless=True)
        self.current_ref = CurrentReference(par, cfg)
        self.current_ctrl = CurrentController(par, 2*np.pi*200)
        self.pll = PhaseLockedLoop(w_o=2*np.pi*40)
        self.signal_inj = SignalInjection(par, U_inj=250)
        if J is not None:
            self.speed_ctrl = SpeedController(J, 2*np.pi*4)
        else:
            self.speed_ctrl = None
        self.observer = None

    def get_feedback_signals(self, mdl):
        fbk = super().get_feedback_signals(mdl)

        # Get the rotor speed and position estimates
        fbk.w_m = self.pll.state.w_m
        fbk.theta_m = self.pll.state.theta_m

        # Current vector in (estimated) rotor coordinates
        fbk.i_s = np.exp(-1j*fbk.theta_m)*fbk.i_ss
        fbk.u_s = np.exp(-1j*fbk.theta_m)*fbk.u_ss

        # Filter the current measurement for the current controller
        fbk.i_s_flt = self.signal_inj.filter_current(fbk.i_s)

        return fbk

    def output(self, fbk):
        """Compute outputs."""
        ref = super().output(fbk)
        ref = super().get_torque_reference(fbk, ref)

        ref = self.current_ref.output(fbk, ref)

        # Superimpose the excitation voltage on the d-axis
        ref.u_s = (
            self.current_ctrl.output(ref.i_s, fbk.i_s_flt) +
            self.signal_inj.u_sd_inj)

        ref.u_ss = ref.u_s*np.exp(1j*fbk.theta_m)
        ref.d_abc = self.pwm(ref.T_s, ref.u_ss, fbk.u_dc, fbk.w_m)

        return ref

    def update(self, fbk, ref):
        super().update(fbk, ref)

        err = self.signal_inj.output(ref.T_s, fbk.i_s.imag)
        self.pll.update(ref.T_s, err)
        self.current_ref.update(fbk, ref)
        self.current_ctrl.update(ref.T_s, fbk.u_s, fbk.w_m)
        self.signal_inj.update(fbk.i_s)


# %%
class SignalInjection:
    """
    Estimate the rotor position error based on signal injection.

    This signal-injection method estimates the rotor position error based on
    the injected switching frequency signal. The estimate can be used in a
    phase-locked loop or in a state observer to robustify low-speed sensorless
    operation.

    Parameters
    ----------
    par : SynchronousMachinePars
        Machine model parameters.
    U_inj : float
        Injected voltage amplitude (V).

    """

    def __init__(self, par, U_inj):
        self.k = 0.5*par.L_d*par.L_q/((par.L_q - par.L_d))  # Error gain
        self._old_i_s, self._older_i_s = 0, 0
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
        di_sq = i_sq - 2*self._old_i_s.imag + self._older_i_s.imag
        err = (self.k/T_s)*di_sq/self.u_sd_inj if np.abs(
            self.u_sd_inj) > 0 else 0

        return err

    def filter_current(self, i_s):
        """
        Filter the stator current using the previously measured value.

        Parameters
        ----------
        i_s : complex
            Unfiltered stator current (A) in estimated rotor coordinates.

        Returns
        -------
        i_s_flt : complex
            Filtered stator current (A) in estimated rotor coordinates.

        """
        # Filter currents
        i_s_flt = 0.5*(i_s + self._old_i_s)

        return i_s_flt

    def update(self, i_s):
        """
        Store the old current values for the next sampling period.

        Parameters
        ----------
        i_s : complex
            Stator current in estimated rotor coordinates.

        """
        # Update the integral states
        self._older_i_s = self._old_i_s
        self._old_i_s = i_s
        # Reverse the d-axis square wave injection voltage
        self.u_sd_inj = -self.u_sd_inj


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
        self.gain = SimpleNamespace(k_p=w_o, k_i=w_o**2)
        self.state = SimpleNamespace(theta_m=0, w_m=0)

    def update(self, T_s, err):
        """
        Update the states.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        err : float
            Rotor position error (rad).

        """
        # Speed estimation
        w_m = self.gain.k_p*err + self.state.w_m

        # Update the states
        self.state.w_m += T_s*self.gain.k_i*err
        self.state.theta_m = wrap(self.state.theta_m + T_s*w_m)
