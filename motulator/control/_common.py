"""Common control functions and classes."""

from abc import ABC, abstractmethod
from types import SimpleNamespace
import numpy as np
from motulator._helpers import abc2complex, complex2abc, wrap


# %%
class PWM:
    """
    Duty ratios and realized voltage for three-phase space-vector PWM.

    This computes the duty ratios corresponding to standard space-vector PWM 
    and minimum-amplitude-error overmodulation [#Hav1999]_. The realized 
    voltage is computed based on the measured DC-bus voltage and the duty 
    ratios. The digital delay effects are taken into account in the realized 
    voltage [#Bae2003]_.

    Parameters
    ----------
    six_step: bool, optional
        Enable six-step operation in overmodulation. The default is False.
    k_comp : float, optional
        Compensation factor for the delay effect on the voltage vector angle. 
        The default is 1.5.

    Attributes
    ----------
    u_cs : complex
        Realized converter voltage (V) in stator coordinates over the sampling 
        period with the delay compensation.

    References
    ----------
    .. [#Hav1999] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation 
       characteristics of triangle intersection PWM methods," IEEE Trans. Ind.
       Appl., 1999, https://doi.org/10.1109/28.777199
    
    .. [#Bae2003] Bae, Sul, "A compensation method for time delay of 
       full-digital synchronous frame current regulator of PWM AC drives," IEEE
       Trans. Ind. Appl., 2003, https://doi.org/10.1109/TIA.2003.810660
    
    """

    def __init__(self, six_step=False, k_comp=1.5):
        self.six_step = six_step
        self.k_comp = k_comp
        self.realized_voltage = 0
        self._u_cs_lim_old = 0

    @staticmethod
    def six_step_overmodulation(u_cs_ref, u_dc):
        """
        Overmodulation up to six-step operation.

        This method modifies the angle of the voltage reference vector in the
        overmodulation region such that the six-step operation is reached 
        [#Bol1997]_.

        Parameters
        ----------
        u_cs_ref : complex
            Converter voltage reference (V) in stator coordinates.
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        u_cs_ref_mod : complex
            Converter voltage reference (V) in stator coordinates.

        References
        ----------
        .. [#Bol1997] Bolognani, Zigliotto, "Novel digital continuous control 
           of SVM inverters in the overmodulation range," IEEE Trans. Ind. 
           Appl., 1997, https://doi.org/10.1109/28.568019

        """
        # Limited magnitude
        r = np.min([np.abs(u_cs_ref), 2/3*u_dc])

        if np.sqrt(3)*r > u_dc:
            # Angle and sector of the reference vector
            theta = np.angle(u_cs_ref)
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
            u_cs_ref_mod = r*np.exp(1j*(theta0 + sector*np.pi/3))
        else:
            u_cs_ref_mod = u_cs_ref

        return u_cs_ref_mod

    @staticmethod
    def duty_ratios(u_cs_ref, u_dc):
        """
        Compute the duty ratios for three-phase space-vector PWM.

        This computes the duty ratios corresponding to standard space-vector 
        PWM and minimum-amplitude-error overmodulation [#Hav1999]_.

        Parameters
        ----------
        u_cs_ref : complex
            Converter voltage reference (V) in stator coordinates.
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        d_abc : ndarray, shape (3,)
            Duty ratios.

        """
        # Phase voltages without the zero-sequence voltage
        u_abc = complex2abc(u_cs_ref)

        # Zero-sequence voltage resulting in space-vector PWM
        u_0 = .5*(np.amax(u_abc) + np.amin(u_abc))
        u_abc -= u_0

        # Uncommenting the following lines results in minimum-phase-error
        # overmodulation. See [#Hav1999]_ for a comparison of the methods.
        # m = (2./u_dc)*np.amax(u_abc)
        # if m > 1:
        #    u_abc = u_abc/m

        # Duty ratios
        d_abc = .5 + u_abc/u_dc

        # Minimum-amplitude-error overmodulation
        d_abc = np.clip(d_abc, 0, 1)

        return d_abc

    def output(self, T_s, u_cs_ref, u_dc, w):
        """
        Compute the duty ratios and the limited voltage reference.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_cs_ref : complex
            Converter voltage reference (V) in stator coordinates.
        u_dc : float
            DC-bus voltage (V).
        w : float
            Angular speed of synchronous coordinates (rad/s).

        Returns
        -------
        d_abc : ndarray, shape (3,)
            Duty ratios for the next sampling period.
        u_cs_lim : complex
            Limited voltage reference (V) in synchronous coordinates.
        
        """
        # Advance the angle due to the computational delay (N*T_s) and the ZOH
        # (PWM) delay (0.5*T_s), typically 1.5*T_s*w
        theta_comp = self.k_comp*T_s*w
        u_cs_comp = np.exp(1j*theta_comp)*u_cs_ref

        # Modify angle in the overmodulation region
        if self.six_step:
            u_cs_comp = self.six_step_overmodulation(u_cs_comp, u_dc)

        # Duty ratios
        d_abc = self.duty_ratios(u_cs_comp, u_dc)

        # Limited voltage reference
        u_cs_lim = abc2complex(d_abc)*u_dc

        return d_abc, u_cs_lim

    def update(self, u_cs_lim):
        """Update the realized voltage."""

        # Update the voltage estimate for the next sampling instant.
        self.realized_voltage = .5*(self._u_cs_lim_old + u_cs_lim)
        self._u_cs_lim_old = u_cs_lim

    def __call__(self, T_s, u_cs_ref, u_dc, w):
        d_abc, u_cs_lim = self.output(T_s, u_cs_ref, u_dc, w)
        self.update(u_cs_lim)
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
    controller is obtained by choosing ``k_t = k_p``. The integrator 
    anti-windup is implemented based on the realized controller output.

    Notes
    -----
    This controller can be used, e.g., as a speed controller. In this case, `y` 
    corresponds to the rotor angular speed `w_M` and `u` to the torque 
    reference `tau_M_ref`.

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
    tau_M_max : float, optional
        Maximum motor torque (Nm). The default is inf.

    """

    def __init__(self, J, alpha_s, tau_M_max=np.inf):
        k_p = 2*alpha_s*J
        k_i = alpha_s**2*J
        k_t = alpha_s*J
        super().__init__(k_p, k_i, k_t, tau_M_max)


# %%
class ComplexPICtrl:
    """
    2DOF synchronous-frame complex-vector PI controller.

    This implements a discrete-time 2DOF synchronous-frame complex-vector PI
    controller, whose continuous-time counterpart is [#Bri2000]_::

        u = k_t*i_ref - k_p*i + (k_i + 1j*w*k_t)/s*(i_ref - i)

    where `u` is the controller output, `i_ref` is the reference signal, `i` is
    the feedback signal, `w` is the angular speed of synchronous coordinates, 
    and `1/s` refers to integration. This algorithm is compatible with both 
    real and complex signals. The 1DOF version is obtained by setting 
    ``k_t = k_p``. The integrator anti-windup is implemented based on the 
    realized controller output.

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
    This controller can be used, e.g., as a current controller. In this case, 
    `i` corresponds to the stator current and `u` to the stator voltage.
    
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
        T_s : float
            Sampling period (s).
        u : float
            Input signal.

        Returns
        -------
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
        self.t = (self.t + T_s) % self.t_reset


# %%
class Ctrl(ABC):
    """Base class for control systems."""

    def __init__(self, T_s):
        self.clock = Clock()
        self._data = SimpleNamespace()  # Private data
        self.data = SimpleNamespace()  # Public data
        self.pwm = PWM()
        self.T_s = T_s

    @abstractmethod
    def get_feedback_signals(self, mdl):
        """
        Get the feedback signals.

        Parameters
        ----------
        mdl : Model
            Continuous-time system model.

        Returns
        -------
        fbk : SimpleNamespace
            Feedback signals.
        
        """
        fbk = SimpleNamespace()
        return fbk

    @abstractmethod
    def output(self, fbk):
        """
        Compute the controller outputs.

        Parameters
        ----------
        fbk : SimpleNamespace
            Feedback signals.

        Returns
        -------
        ref : SimpleNamespace
            References, containing at least the following fields:
            T_s : float
                Next sampling period (s).
            d_abc : ndarray, shape (3,)
                Duty ratios.
                    
        """
        ref = SimpleNamespace()
        ref.t = self.clock.t
        ref.T_s = self.T_s  # Desired next sampling period
        return ref

    @abstractmethod
    def update(self, fbk, ref):
        """
        Update the states.
        
        Parameters
        ----------
        fbk : SimpleNamespace
            Feedback signals.
        ref : SimpleNamespace
            Reference signals.
        
        """
        self.T_s = ref.T_s
        self.clock.update(ref.T_s)

    def save(self, **kwargs):
        """
        Save the data of the control system.

        Each keyword represents a data category, and its value (a 
        SimpleNamespace) contains the data for that category.
        
        Parameters
        ----------
        **kwargs : SimpleNamespace
            One or more keyword arguments where the key is the name and the
            value is a SimpleNamespace containing the data to be saved.

        """
        for name, arg in kwargs.items():
            if not hasattr(self._data, name):
                setattr(self._data, name, SimpleNamespace())
            data = getattr(self._data, name)
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
        for name, values in vars(self._data).items():
            setattr(self.data, name, SimpleNamespace())
            for key, value in vars(values).items():
                setattr(getattr(self.data, name), key, np.asarray(value))

    def __call__(self, mdl):
        """
        Main control loop.

        Parameters
        ----------
        mdl : Model
            Continuous-time system model.

        Returns
        -------
        T_s : float
            Sampling period (s).
        d_abc : ndarray, shape (3,)
            Duty ratios.

        """
        # Feedback signals
        fbk = self.get_feedback_signals(mdl)

        # Compute references
        ref = self.output(fbk)

        # Update states
        self.update(fbk, ref)

        # Save data
        self.save(fbk=fbk, ref=ref)

        # Return the sampling period and the duty ratios for the PWM
        return ref.T_s, ref.d_abc


# %%
class DriveCtrl(Ctrl, ABC):
    """
    Base class for control of electric machine drives.

    This base class provides typical functionalities for control of electric
    machine drives. This can be used both in speed-control and torque-control 
    modes. 

    Parameters
    ----------
    model_par : ModelPars
        Machine model parameters.
    T_s : float
        Sampling period (s).
    sensorless : bool
        If True, sensorless control mode is used.

    Attributes
    ----------
    observer : Observer
        State observer. 
    speed_ctrl : SpeedCtrl 
        Speed controller.   
    ref : SimpleNamespace
        References, possibly containing either of the following fields:
        w_m : callable
            Speed reference (mechanical rad/s) as a function of time (s).
        tau_M : callable
            Torque reference (Nm) as a function of time (s).

    """

    def __init__(self, model_par, T_s, sensorless):
        super().__init__(T_s)
        self.par = model_par
        self.sensorless = sensorless
        self.speed_ctrl = None
        self.observer = None
        self.ref = SimpleNamespace()

    def get_electrical_measurements(self, fbk, mdl):
        """Measure the currents and voltages."""
        fbk.u_dc = mdl.converter.meas_dc_voltage()
        fbk.i_ss = abc2complex(mdl.machine.meas_currents())
        fbk.u_ss = self.pwm.realized_voltage
        return fbk

    def get_mechanical_measurements(self, fbk, mdl):
        """Measure the speed and position."""
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
        """Get the torque reference in vector control."""
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
            self.speed_ctrl.update(ref.T_s, ref.tau_M_lim)
        if self.observer:
            self.observer.update(ref.T_s, fbk)
