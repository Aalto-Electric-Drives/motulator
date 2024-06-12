"""Common control functions and classes."""
from abc import ABC
from types import SimpleNamespace

import numpy as np
#from motulator.common.utils._utils import (abc2complex, complex2abc)
#from motulator.common.control import (Ctrl, PICtrl)
from motulator.common.utils import (complex2abc, abc2complex, wrap)

# %%
class PWM:
    """
    Duty ratios and realized voltage for three-phase PWM.

    This contains the computation of the duty ratios and the realized voltage. 
    The realized voltage is computed based on the measured DC-bus voltage and 
    the duty ratios. The digital delay effects are taken into account in the 
    realized voltage, assuming the delay of a single sampling period. The total
    delay is 1.5 sampling periods due to the PWM (or ZOH) delay [#Bae2003]_.

    Parameters
    ----------
    six_step: bool, optional
        Enable six-step operation in overmodulation. The default is False.
    
    Attributes
    ----------
    realized_voltage : complex
        Realized voltage (V) in synchronous coordinates.

    References
    ----------
    .. [#Bae2003] Bae, Sul, "A compensation method for time delay of 
       full-digital synchronous frame current regulator of PWM AC drives," IEEE
       Trans. Ind. Appl., 2003, https://doi.org/10.1109/TIA.2003.810660
    
    """

    def __init__(self, six_step=False):
        self.six_step = six_step
        self.realized_voltage = 0
        self._u_ref_lim_old = 0

    @staticmethod
    def six_step_overmodulation(u_s_ref, u_dc):
        """
        Overmodulation up to six-step operation.

        This method modifies the angle of the voltage reference vector in the
        overmodulation region such that the six-step operation is reached 
        [#Bol1997]_.

        Parameters
        ----------
        u_s_ref : complex
            Reference voltage (V) in stator coordinates.
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        u_s_ref_mod : complex
            Reference voltage (V) in stator coordinates.

        References
        ----------
        .. [#Bol1997] Bolognani, Zigliotto, "Novel digital continuous control of 
           SVM inverters in the overmodulation range," IEEE Trans. Ind. Appl.,
           1997, https://doi.org/10.1109/28.568019

        """
        # Limited magnitude
        r = np.min([np.abs(u_s_ref), 2/3*u_dc])

        if np.sqrt(3)*r > u_dc:
            # Angle and sector of the reference vector
            theta = np.angle(u_s_ref)
            sector = np.floor(3*theta/np.pi)

            # Angle reduced to the first sector (at which sector == 0)
            theta0 = theta - sector*np.pi/3

            # Intersection angle, see Eq. (9)
            alpha_g = np.pi/6 - np.arccos(u_dc/(np.sqrt(3)*r))

            # Modify the angle according to Eq. (4)
            if alpha_g <= theta0 <= np.pi/6:
                theta0 = alpha_g
            elif np.pi/6 <= theta0 <= np.pi/3 - alpha_g:
                theta0 = np.pi/3 - alpha_g

            # Modified reference voltage
            u_s_ref_mod = r*np.exp(1j*(theta0 + sector*np.pi/3))
        else:
            u_s_ref_mod = u_s_ref

        return u_s_ref_mod

    @staticmethod
    def duty_ratios(u_s_ref, u_dc):
        """
        Compute the duty ratios for three-phase PWM.

        This computes the duty ratios using a symmetrical suboscillation
        method. This method is identical to the standard space-vector PWM.

        Parameters
        ----------
        u_s_ref : complex
            Voltage reference in stator coordinates (V).
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        d_abc : ndarray, shape (3,)
            Duty ratios.

        """
        # Phase voltages without the zero-sequence voltage
        u_abc = complex2abc(u_s_ref)

        # Symmetrization by adding the zero-sequence voltage
        u_0 = .5*(np.amax(u_abc) + np.amin(u_abc))
        u_abc -= u_0

        # Preventing overmodulation by means of a minimum phase error method
        m = (2./u_dc)*np.amax(u_abc)
        if m > 1:
            u_abc = u_abc/m

        # Duty ratios
        d_abc = .5 + u_abc/u_dc

        return d_abc

    # pylint: disable=too-many-arguments
    def __call__(self, T_s, u_ref, u_dc, theta, w):
        """
        Compute the duty ratios and update the state.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_ref : complex
            Voltage reference in synchronous coordinates (V).
        u_dc : float
            DC-bus voltage.
        theta : float
            Angle of synchronous coordinates (rad).
        w : float
            Angular speed of synchronous coordinates (rad/s).

        Returns
        -------
        d_abc : ndarray, shape (3,)
            Duty ratios for the next sampling period.

        """
        # Advance the angle due to the computational delay (T_s) and the ZOH
        # (PWM) delay (0.5*T_s)
        theta_comp = theta + 1.5*T_s*w

        # Voltage reference in stator coordinates
        u_s_ref = np.exp(1j*theta_comp)*u_ref

        # Modify angle in the overmodulation region
        if self.six_step:
            u_s_ref = self.six_step_overmodulation(u_s_ref, u_dc)

        # Duty ratios
        d_abc = self.duty_ratios(u_s_ref, u_dc)

        # Realizable voltage
        u_s_ref_lim = abc2complex(d_abc)*u_dc
        u_ref_lim = np.exp(-1j*theta_comp)*u_s_ref_lim

        # Update the voltage estimate for the next sampling instant.
        self.realized_voltage = .5*(self._u_ref_lim_old + u_ref_lim)
        self._u_ref_lim_old = u_ref_lim

        return d_abc


# %%
class PICtrl:
    """
    2DOF PI controller.

    This implements a discrete-time 2DOF PI controller, whose continuous-time
    counterpart is::

        u = k_t*y_ref - k_p*y + (k_i/s)*(y_ref - y)

    where `u` is the controller output, `y_ref` is the reference signal, `y` is
    the feedback signal, and `1/s` refers to integration. The standard PI
    controller is obtained by choosing ``k_t = k_p``. The integrator anti-windup 
    is implemented based on the realized controller output.

    Notes
    -----
    This contoller can be used, e.g., as a speed controller. In this case, `y` 
    corresponds to the rotor angular speed `w_M` and `u` to the torque reference 
    `tau_M_ref`.

    Parameters
    ----------
    k_p : float
        Proportional gain.
    k_i : float
        Integral gain.
    k_t : float, optional
        Reference-feedforward gain. The default is `k_p`.
    u_max : float, optional
        Maximum controller output. The default is inf.

    Attributes
    ----------
    v : float
        Input disturbance estimate.
    u_i : float
        Integral state.

    """

    def __init__(self, k_p, k_i, k_t=None, u_max=np.inf):
        self.u_max = u_max
        # Gains
        self.k_p = k_p
        self.k_t = k_t if k_t is not None else k_p
        self.alpha_i = k_i/self.k_t  # Inverse of the integration time T_i
        # States
        self.v, self.u_i = 0, 0

    def output(self, y_ref, y):
        """
        Compute the controller output.

        Parameters
        ----------
        y_ref : float
            Reference signal.
        y : float
            Feedback signal.

        Returns
        -------
        u : float
            Controller output.

        """
        # Estimate of a disturbance input
        self.v = self.u_i - (self.k_p - self.k_t)*y

        # Controller output
        u = self.k_t*(y_ref - y) + self.v

        # Limit the controller output
        u = np.clip(u, -self.u_max, self.u_max)

        return u

    def update(self, T_s, u_lim):
        """
        Update the integral state.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_lim : float
            Realized (limited) controller output. If the actuator does not
            saturate, ``u_lim = u``.

        """
        self.u_i += T_s*self.alpha_i*(u_lim - self.v)


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
class ComplexPICtrl:
    """
    2DOF synchronous-frame complex-vector PI controller.

    This implements a discrete-time 2DOF synchronous-frame complex-vector PI
    controller, whose continuous-time counterpart is [#Bri2000]_::

        u = k_t*i_ref - k_p*i + (k_i + 1j*w*k_t)/s*(i_ref - i)

    where `u` is the controller output, `i_ref` is the reference signal, `i` is
    the feedback signal, `w` is the angular speed of synchronous coordinates, 
    and `1/s` refers to integration. This algorithm is compatible with both real 
    and complex signals. The 1DOF version is obtained by setting ``k_t = k_p``. 
    The integrator anti-windup is implemented based on the realized controller 
    output.

    Parameters
    ----------
    k_p : float
        Proportional gain.
    k_i : float
        Integral gain.
    k_t : float, optional
        Reference-feedforward gain. The default is `k_p`.

    Attributes
    ----------
    v : complex
        Input disturbance estimate.
    u_i : complex
        Integral state.

    Notes
    -----
    This contoller can be used, e.g., as a current controller. In this case, `i`
    corresponds to the stator current and `u` to the stator voltage.
    
    References
    ----------
    .. [#Bri2000] Briz, Degner, Lorenz, "Analysis and design of current 
       regulators using complex vectors," IEEE Trans. Ind. Appl., 2000,
       https://doi.org/10.1109/28.845057

    """

    def __init__(self, k_p, k_i, k_t=None):
        # Gains
        self.k_p = k_p
        self.k_t = k_t if k_t is not None else k_p
        self.alpha_i = k_i/self.k_t  # Inverse of the integration time T_i
        # States
        self.v, self.u_i = 0, 0

    def output(self, i_ref, i):
        """
        Compute the controller output.

        Parameters
        ----------
        i_ref : complex
            Reference signal.
        i : complex
            Feedback signal.

        Returns
        -------
        u : complex
            Controller output.

        """
        # Disturbance input estimate
        self.v = self.u_i - (self.k_p - self.k_t)*i

        # Controller output
        u = self.k_t*(i_ref - i) + self.v

        return u

    def update(self, T_s, u_lim, w):
        """
        Update the integral state.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_lim : complex
            Realized (limited) controller output. If the actuator does not
            saturate, ``u_lim = u``.
        w : float
            Angular speed of the reference frame (rad/s). 

        """
        self.u_i += T_s*(self.alpha_i + 1j*w)*(u_lim - self.v)


# %%
class ComplexFFPICtrl:
    """
    2DOF Synchronous-frame complex-vector PI controller with feedforward.

    This implements a discrete-time 2DOF synchronous-frame complex-vector PI
    controller similar to [#Bri2000]_, with an additional feedforward signal.                        
    The gain selection corresponding to internal-model-control (IMC) is
    equivalent to the continuous-time version given in [#Har2009]_::

        u = k_p*(i_ref - i) + k_i/s*(i_ref - i) + 1j*w*L_f*i + u_g_ff 
                
    where `u` is the controller output, `i_ref` is the reference signal, `i` is
    the feedback signal, u_g_ff is the (filtered) feedforward signal, `w` is
    the angular speed of synchronous coordinates, '1j*w*L_f' is the decoupling
    term estimate, and `1/s` refers to integration. This algorithm is
    compatible with both real and complex signals. The integrator anti-windup
    is implemented based on the realized controller output.

    Parameters
    ----------
    k_p : float
        Proportional gain.
    k_i : float
        Integral gain.
    k_t : float, optional
        Reference-feedforward gain. The default is `k_p`.
    L_f : float, optional
        Synchronous frame decoupling gain. The default is 0.

    Attributes
    ----------
    v : complex
        Input disturbance estimate.
    u_i : complex
        Integral state.

    Notes
    -----
    This contoller can be used, e.g., as a current controller. In this case,
    `i` corresponds to the converter current and `u` to the converter voltage.
    
    References
    ----------
       
    .. [#Har2009] Harnefors, Bongiorno, "Current controller design
       for passivity of the input admittance," 2009 13th European Conference
       on Power Electronics and Applications, Barcelona, Spain, 2009.

    """

    def __init__(self, k_p, k_i, k_t=None, L_f=None):
        # Gains
        self.k_p = k_p
        self.k_t = k_t if k_t is not None else k_p
        self.alpha_i = k_i/self.k_t  # Inverse of the integration time T_i
        self.L_f = L_f if L_f is not None else 0
        # States
        self.v, self.u_i = 0, 0

    def output(self, i_ref, i, u_ff, w):
        """
        Compute the controller output.

        Parameters
        ----------
        i_ref : complex
            Reference signal.
        i : complex
            Feedback signal.
        u_ff : complex
            Feedforward signal.
        w : float
            Angular speed of the reference frame (rad/s). 

        Returns
        -------
        u : complex
            Controller output.

        """
        # Disturbance input estimate
        self.v = self.u_i - (self.k_p - self.k_t)*i + (u_ff
        + 1j*w*self.L_f*i)

        # Controller output
        u = self.k_t*(i_ref - i) + self.v

        return u

    def update(self, T_s, u_lim):
        """
        Update the integral state.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_lim : complex
            Realized (limited) controller output. If the actuator does not
            saturate, ``u_lim = u``.

        """
        self.u_i += T_s*self.alpha_i*(u_lim - self.v)


# %%
class RateLimiter:
    """
    Rate limiter.

    Parameters
    ----------
    rate_limit : float, optional
        Rate limit. The default is inf.

    """

    def __init__(self, rate_limit=np.inf):
        self.rate_limit = rate_limit
        self._y_old = 0

    def __call__(self, T_s, u):
        """
        Limit the input signal.

        Parameters
        ----------
        u : float
            Input signal.

        Returns
        -------
        T_s : float
            Sampling period (s).
        y : float
            Rate-limited output signal.

        Notes
        -----
        In this implementation, the falling rate limit equals the (negative)
        rising rate limit. If needed, these limits can be separated with minor
        modifications in the class.

        """
        rate = (u - self._y_old)/T_s

        if rate > self.rate_limit:
            # Limit rising rate
            y = self._y_old + T_s*self.rate_limit
        elif rate < -self.rate_limit:
            # Limit falling rate
            y = self._y_old - T_s*self.rate_limit
        else:
            y = u

        # Store the limited output
        self._y_old = y

        return y


# %%
class Clock:
    """Digital clock."""

    def __init__(self):
        self.t = 0
        self.t_reset = 1e9

    def update(self, T_s):
        """
        Update the digital clock.

        Parameters
        ----------
        T_s : float
            Sampling period (s).

        """
        self.t += T_s if self.t < self.t_reset else 0


# %%
class Ctrl:
    """Base class for the control system."""

    def __init__(self):
        self.data = SimpleNamespace()  # Data store
        self.clock = Clock()  # Digital clock

    def __call__(self, mdl):
        """
        Run the main control loop.

        The main control loop is callable that returns the sampling
        period `T_s` (float)  and the duty ratios `d_abc` (ndarray, shape (3,)) 
        for the next sampling period.

        Parameters
        ----------
        mdl : Model
            System model containing methods for getting the feedback signals.

        """
        raise NotImplementedError

    def save(self, **kwargs):
        """
        Save the internal date of the control system.

        Parameters
        ----------
        data : bunch or dict
            Contains the data to be saved.

        """
        for name, arg in kwargs.items():
            if not hasattr(self.data, name):
                setattr(self.data, name, SimpleNamespace())
            data = getattr(self.data, name)
            for key, value in vars(arg).items():
                if not hasattr(data, key):
                    setattr(data, key, [])
                getattr(data, key).append(value)

    def post_process(self):
        """
        Transform the lists to the ndarray format.

        This method can be run after the simulation has been completed in order 
        to simplify plotting and analysis of the stored data.

        """
        self.data = SimpleNamespace()
        for name, values in vars(self.data).items():
            setattr(self.data, name, SimpleNamespace())
            for key, value in vars(values).items():
                setattr(getattr(self.data, name), key, np.asarray(value))


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
