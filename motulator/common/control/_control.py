"""Common functions and classes for controls."""

from abc import ABC, abstractmethod
from types import SimpleNamespace

import numpy as np

from motulator.common.utils import abc2complex, complex2abc


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
        self._old_u_cs = 0

    @staticmethod
    def six_step_overmodulation(ref_u_cs, u_dc):
        """
        Overmodulation up to six-step operation.

        This method modifies the angle of the voltage reference vector in the
        overmodulation region such that the six-step operation is reached 
        [#Bol1997]_.

        Parameters
        ----------
        ref_u_cs : complex
            Converter voltage reference (V) in stationary coordinates.
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        ref_u_cs : complex
            Modified converter voltage reference (V) in stationary coordinates.

        References
        ----------
        .. [#Bol1997] Bolognani, Zigliotto, "Novel digital continuous control 
           of SVM inverters in the overmodulation range," IEEE Trans. Ind. 
           Appl., 1997, https://doi.org/10.1109/28.568019

        """
        # Limited magnitude
        r = np.min([np.abs(ref_u_cs), 2/3*u_dc])

        if np.sqrt(3)*r > u_dc:
            # Angle and sector of the reference vector
            theta = np.angle(ref_u_cs)
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
            ref_u_cs = r*np.exp(1j*(theta0 + sector*np.pi/3))

        return ref_u_cs

    @staticmethod
    def duty_ratios(ref_u_cs, u_dc):
        """
        Compute the duty ratios for three-phase space-vector PWM.

        This computes the duty ratios corresponding to standard space-vector 
        PWM and minimum-amplitude-error overmodulation [#Hav1999]_.

        Parameters
        ----------
        ref_u_cs : complex
            Converter voltage reference (V) in stationary coordinates.
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        d_abc : ndarray, shape (3,)
            Duty ratios.

        """
        # Phase voltages without the zero-sequence voltage
        u_abc = complex2abc(ref_u_cs)

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

    def output(self, T_s, ref_u_cs, u_dc, w):
        """
        Compute the duty ratios and the limited voltage reference.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        ref_u_cs : complex
            Converter voltage reference (V) in stationary coordinates.
        u_dc : float
            DC-bus voltage (V).
        w : float
            Angular speed of synchronous coordinates (rad/s).

        Returns
        -------
        d_abc : ndarray, shape (3,)
            Duty ratios for the next sampling period.
        u_cs : complex
            Limited voltage reference (V) in stationary coordinates.
        
        """
        # Advance the angle due to the computational delay (N*T_s) and the ZOH
        # (PWM) delay (0.5*T_s), typically 1.5*T_s*w
        theta_comp = self.k_comp*T_s*w
        ref_u_cs = np.exp(1j*theta_comp)*ref_u_cs

        # Modify angle in the overmodulation region
        if self.six_step:
            ref_u_cs = self.six_step_overmodulation(ref_u_cs, u_dc)

        # Duty ratios
        d_abc = self.duty_ratios(ref_u_cs, u_dc)

        # Limited voltage reference
        u_cs = abc2complex(d_abc)*u_dc

        return d_abc, u_cs

    def get_realized_voltage(self):
        """
        Get the realized voltage.
        
        Returns
        -------
        realized_voltage : complex
            Realized converter voltage (V) in stationary coordinates. The 
            effect of the digital delays on the angle are compensated for.
        
        """
        return self.realized_voltage

    def update(self, u_cs):
        """Update the realized voltage."""
        # Update the voltage estimate for the next sampling instant
        self.realized_voltage = .5*(self._old_u_cs + u_cs)
        self._old_u_cs = u_cs

    def __call__(self, T_s, ref_u_cs, u_dc, w):
        d_abc, u_cs = self.output(T_s, ref_u_cs, u_dc, w)
        self.update(u_cs)
        return d_abc


# %%
class PIController:
    """
    2DOF PI controller.

    This implements a discrete-time 2DOF PI controller, whose continuous-time
    counterpart is::

        u = k_t*ref_y - k_p*y + (k_i/s)*(ref_y - y)

    where `u` is the controller output, `y_ref` is the reference signal, `y` is
    the feedback signal, and `1/s` refers to integration. The standard PI
    controller is obtained by choosing ``k_t = k_p``. The integrator 
    anti-windup is implemented based on the realized controller output.

    Notes
    -----
    This controller can be used, e.g., as a speed controller. In this case, `y` 
    corresponds to the rotor angular speed `w_M` and `u` to the torque 
    reference `ref_tau_M`.

    Parameters
    ----------
    k_p : float
        Proportional gain.
    k_i : float
        Integral gain.
    k_t : float, optional
        Reference-feedforward gain. The default is `k_p`.
    max_u : float, optional
        Maximum controller output. The default is `inf`.

    """

    def __init__(self, k_p, k_i, k_t=None, max_u=np.inf):
        self.max_u = max_u
        # Gains
        self.k_p = k_p
        self.k_t = k_t if k_t is not None else k_p
        self.alpha_i = k_i/self.k_t  # Inverse of the integration time T_i
        # States
        self.v, self.u_i = 0, 0

    def output(self, ref_y, y):
        """
        Compute the controller output.

        Parameters
        ----------
        ref_y : float
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
        u = self.k_t*(ref_y - y) + self.v

        # Limit the controller output
        u = np.clip(u, -self.max_u, self.max_u)

        return u

    def update(self, T_s, u):
        """
        Update the integral state.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u : float
            Realized (limited) controller output. 

        """
        self.u_i += T_s*self.alpha_i*(u - self.v)


# %%
class ComplexPIController:
    """
    2DOF synchronous-frame complex-vector PI controller.

    This implements a discrete-time 2DOF synchronous-frame complex-vector PI
    controller, whose continuous-time counterpart is [#Bri2000]_::

        u = k_t*ref_i - k_p*i + (k_i + 1j*w*k_t)/s*(ref_i - i)

    where `u` is the controller output, `ref_i` is the reference signal, `i` is
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

    def output(self, ref_i, i):
        """
        Compute the controller output.

        Parameters
        ----------
        ref_i : complex
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
        u = self.k_t*(ref_i - i) + self.v

        return u

    def update(self, T_s, u, w):
        """
        Update the integral state.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u : complex
            Realized (limited) controller output. 
        w : float
            Angular speed of the reference frame (rad/s). 

        """
        self.u_i += T_s*(self.alpha_i + 1j*w)*(u - self.v)


# %%
class ComplexFFPIController:
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
        self._old_y = 0

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

        """
        # In this implementation, the falling rate limit equals the (negative)
        # rising rate limit. If needed, these limits can be separated with
        # minor modifications in the class.

        rate = (u - self._old_y)/T_s

        if rate > self.rate_limit:
            # Limit rising rate
            y = self._old_y + T_s*self.rate_limit
        elif rate < -self.rate_limit:
            # Limit falling rate
            y = self._old_y - T_s*self.rate_limit
        else:
            y = u

        # Store the limited output
        self._old_y = y

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
class ControlSystem(ABC):
    """
    Base class for control systems.
    
    This base class provides typical functionalities for control systems. It
    can be used as a template for implementing custom controllers. An instance 
    of this class can be called as a function. When called, it runs the main
    control loop.

    Parameters
    ----------
    T_s : float
        Sampling period (s).

    Attributes
    ----------
    clock : Clock
        Digital clock.
    data : SimpleNamespace
        Saved simulation data.
    pwm : PWM   
        Pulse-width modulator.

    """

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

    def main(self, mdl):
        """
        Main control loop.

        This method runs the main control loop, having the following structure:

        1. Get the feedback signals. This step may contain first getting the 
           measurements and then optionally computing the observer outputs.
        2. Compute the reference signals (controller outputs) based on the
           feedback signals.
        3. Update the control system states for the next sampling instant.
        4. Save the feedback signals and the reference signals.
        5. Return the sampling period `T_s` and the duty ratios `d_abc` for the
           carrier comparison.
        
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

        # Compute the references
        ref = self.output(fbk)

        # Update the states
        self.update(fbk, ref)

        # Save the data
        self.save(fbk=fbk, ref=ref)

        # Return the sampling period and duty ratios for the carrier comparison
        return ref.T_s, ref.d_abc

    def __call__(self, mdl):
        return self.main(mdl)
