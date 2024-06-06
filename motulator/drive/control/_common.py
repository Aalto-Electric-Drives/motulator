"""Common control functions and classes for machine drives."""

from abc import ABC
from types import SimpleNamespace

import numpy as np

from motulator.utils import abc2complex, wrap
from motulator._common._control import Ctrl, PICtrl


# %%
class SpeedCtrl(PICtrl):
    """
    2DOF PI speed controller.

    This is an interface for a speed controller. The gains are initialized 
    based on the desired closed-loop bandwidth and the rotor inertia estimate.

    Parameters
    ----------
    J : float
        Total inertia of the rotor (kgm²).
    alpha_s : float
        Closed-loop bandwidth (rad/s). 
    max_tau_M : float, optional
        Maximum motor torque (Nm). The default is `inf`.

    """

    def __init__(self, J, alpha_s, max_tau_M=np.inf):
        k_p = 2*alpha_s*J
        k_i = alpha_s**2*J
        k_t = alpha_s*J
        super().__init__(k_p, k_i, k_t, max_tau_M)


# %%
class DriveCtrl(Ctrl, ABC):
    """
    Base class for control of electric machine drives.

    This base class provides typical functionalities for control of electric
    machine drives. This can be used both in speed-control and torque-control 
    modes. 

    Parameters
    ----------
    par : motulator.drive.control.im.ModelPars |\
          motulator.drive.control.sm.ModelPars
        Machine model parameters.
    T_s : float
        Sampling period (s).
    sensorless : bool
        If True, sensorless control mode is used.

    Attributes
    ----------
    ref : SimpleNamespace
        References, possibly containing either of the following fields:

            w_m : callable
                Speed reference (electrical rad/s) as a function of time (s). 
                This signal is needed in speed-control mode.
            tau_M : callable
                Torque reference (Nm) as a function of time (s). This signal
                is needed in torque-control mode.

    observer : motulator.drive.control.im.Observer | \
               motulator.drive.control.sm.Observer | None
        State observer can be None or an instance of either 
        `motulator.drive.control.im.Observer` or 
        `motulator.drive.control.sm.Observer` 
        depending on the machine type. The default is None.
    speed_ctrl : SpeedCtrl | None
        Speed controller. The default is None.

    """

    def __init__(self, par, T_s, sensorless):
        super().__init__(T_s)
        self.par = par
        self.sensorless = sensorless
        self.speed_ctrl = None
        self.observer = None
        self.ref = SimpleNamespace()

    def get_electrical_measurements(self, fbk, mdl):
        """
        Measure the currents and voltages.
        
        Parameters
        ----------
        fbk : SimpleNamespace
            Measured signals are added to this object.
        mdl : Model
            Continuous-time system model.

        Returns
        -------
        fbk : SimpleNamespace
            Measured signals, containing the following fields:

                u_dc : float
                    DC-bus voltage (V).
                i_ss : complex
                    Stator current (A) in stator coordinates.
                u_ss : complex
                    Realized stator voltage (V) in stator coordinates. This
                    signal is obtained from the PWM.

        """
        fbk.u_dc = mdl.converter.meas_dc_voltage()
        fbk.i_ss = abc2complex(mdl.machine.meas_currents())
        fbk.u_ss = self.pwm.get_realized_voltage()

        return fbk

    def get_mechanical_measurements(self, fbk, mdl):
        """
        Measure the speed and position.
        
        Parameters
        ----------
        fbk : SimpleNamespace
            Measured signals are added to this object.
        mdl : Model
            Continuous-time system model.

        Returns
        -------
        fbk : SimpleNamespace
            Measured signals, containing the following fields:

                w_m : float
                    Rotor speed (electrical rad/s).
                theta_m : float
                    Rotor position (electrical rad).
    
        """
        fbk.w_m = self.par.n_p*mdl.mechanics.meas_speed()
        fbk.theta_m = wrap(self.par.n_p*mdl.mechanics.meas_position())

        return fbk

    def get_feedback_signals(self, mdl):
        """Get the feedback signals."""
        fbk = super().get_feedback_signals(mdl)
        fbk = self.get_electrical_measurements(fbk, mdl)
        if not self.sensorless:
            fbk = self.get_mechanical_measurements(fbk, mdl)
        if self.observer:
            fbk = self.observer.output(fbk)

        return fbk

    def get_torque_reference(self, fbk, ref):
        """
        Get the torque reference in vector control.

        This method can be used in vector control to get the torque reference 
        from the speed controller. If the speed controller method `speed_ctrl` 
        is None, the torque reference is obtained directly from the reference.

        Parameters
        ----------
        fbk : SimpleNamespace
            Feedback signals. In speed-control mode, the measured or estimated
            rotor speed `w_m` is used to compute the torque reference.
        ref : SimpleNamespace
            Reference signals, containing the digital time `t`. The speed and 
            torque references are added to this object.

        Returns
        -------
        ref : SimpleNamespace
            Reference signals, containing the following fields:
                
                w_m : float
                    Speed reference (electrical rad/s).
                tau_M : float
                    Torque reference (Nm).  

        """
        if self.speed_ctrl:
            # Speed-control mode
            ref.w_m = self.ref.w_m(ref.t)
            ref_w_M = ref.w_m/self.par.n_p
            w_M = fbk.w_m/self.par.n_p
            ref.tau_M = self.speed_ctrl.output(ref_w_M, w_M)
        else:
            # Torque-control mode
            ref.w_m = None
            ref.tau_M = self.ref.tau_M(ref.t)

        return ref

    def update(self, fbk, ref):
        """Extend the base class method."""
        super().update(fbk, ref)
        if self.speed_ctrl:
            self.speed_ctrl.update(ref.T_s, ref.tau_M)
        if self.observer:
            self.observer.update(ref.T_s, fbk)
