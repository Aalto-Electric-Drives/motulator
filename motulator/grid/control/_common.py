"""Common control functions and classes."""
from abc import ABC
import numpy as np
#from motulator.common.utils._utils import (abc2complex, complex2abc)
from motulator.common.control import (Ctrl, PICtrl)
from motulator.common.utils import abc2complex, wrap
from types import SimpleNamespace


# %%
class DCBusVoltCtrl(PICtrl):
    """
    PI DC-bus voltage controller.

    This provides an interface for a DC-bus controller. The gains are
    initialized based on the desired closed-loop bandwidth and the DC-bus
    capacitance estimate. The PI controller is designed to control the energy
    of the DC-bus capacitance and not the DC-bus voltage in order to have a
    linear closed-loop system [#Hur2001]_.

    Parameters
    ----------
    zeta : float
        Damping ratio of the closed-loop system.
    alpha_dc : float
        Closed-loop bandwidth (rad/s). 
    p_max : float, optional
        Maximum converter power (W). The default is inf.
        
    References
    ----------
    .. [#Hur2001] Hur, Jung, Nam, "A Fast Dynamic DC-Link Power-Balancing
       Scheme for a PWM Converter–Inverter System," IEEE Trans. Ind. Electron.,
       2001, https://doi.org/10.1109/41.937412

    """

    def __init__(self, zeta, alpha_dc, p_max=np.inf):
        k_p = -2*zeta*alpha_dc
        k_i = -(alpha_dc**2)
        k_t = k_p
        super().__init__(k_p, k_i, k_t, p_max)


# %%
class GridConverterCtrl(Ctrl, ABC):
    """
    Base class for control of grid-connected converters.
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
