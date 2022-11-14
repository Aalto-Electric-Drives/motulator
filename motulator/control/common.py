# pylint: disable=invalid-name
"""Common control functions and classes."""

import numpy as np

from motulator.helpers import abc2complex, complex2abc, Bunch


# %%
class PWM:
    """
    Duty ratio references and realized voltage for three-phase PWM.

    This contains the computation of the duty ratio references and the realized
    voltage. The digital delay effects are taken into account in the realized
    voltage.

    Parameters
    ----------
    pars : data object
        Control parameters.

    """

    def __init__(self, pars):
        self.T_s = pars.T_s
        try:
            self.six_step = pars.six_step
        except AttributeError:
            self.six_step = False
        self.realized_voltage = 0
        self._u_ref_lim_old = 0

    @staticmethod
    def six_step_overmodulation(u_s_ref, u_dc):
        """
        Overmodulation up to six-step operation.

        This method modifies the angle of the voltage reference vector in the
        overmodulation region such that the six-step operation is reached [1]_.

        Parameters
        ----------
        u_s_ref : complex
            Voltage reference in stator coordinates.

        Returns
        -------
        u_s_ref_mod : complex
            Voltage reference in stator coordinates.

        References
        ----------
        .. [1] Bolognani, Zigliotto, "Novel digital continuous control of SVM
           inverters in the overmodulation range," IEEE Trans. Ind. Appl.,
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
            Voltage reference in stator coordinates.
        u_dc : float
            DC-bus voltage.

        Returns
        -------
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.

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
        d_abc_ref = .5 + u_abc/u_dc

        return d_abc_ref

    def __call__(self, u_ref, u_dc, theta, w):
        """
        Compute the duty ratios and update the state.

        Parameters
        ----------
        u_ref : complex
            Voltage reference in synchronous coordinates.
        u_dc : float
            DC-bus voltage.
        theta : float
            Angle of synchronous coordinates.
        w : float
            Angular frequency of synchronous coordinates.

        Returns
        -------
        d_abc_ref : ndarray, shape (3,)
            Duty ratio references.

        """
        d_abc_ref, u_ref_lim = self.output(u_ref, u_dc, theta, w)
        self.update(u_ref_lim)

        return d_abc_ref

    def output(self, u_ref, u_dc, theta, w):
        """Compute the duty ratio limited voltage reference."""
        # Advance the angle due to the computational delay (T_s) and
        # the ZOH (PWM) delay (0.5*T_s)
        theta_comp = theta + 1.5*self.T_s*w

        # Voltage reference in stator coordinates
        u_s_ref = np.exp(1j*theta_comp)*u_ref

        # Modify angle in the overmodulation region
        if self.six_step:
            u_s_ref = self.six_step_overmodulation(u_s_ref, u_dc)

        # Duty ratios
        d_abc_ref = self.duty_ratios(u_s_ref, u_dc)

        # Realizable voltage
        u_s_ref_lim = abc2complex(d_abc_ref)*u_dc
        u_ref_lim = np.exp(-1j*theta_comp)*u_s_ref_lim

        return d_abc_ref, u_ref_lim

    def update(self, u_ref_lim):
        """
        Update the voltage estimate for the next sampling instant.

        Parameters
        ----------
        u_ref_lim : complex
            Limited voltage reference in synchronous coordinates.

        """
        self.realized_voltage = .5*(self._u_ref_lim_old + u_ref_lim)
        self._u_ref_lim_old = u_ref_lim


# %%
class SpeedCtrl:
    """
    2DOF PI speed controller.

    This controller is implemented using the disturbance-observer structure.
    The controller is mathematically identical to the 2DOF PI speed controller.

    Parameters
    ----------
    pars : data object
        Control parameters.

    """

    def __init__(self, pars):
        self.T_s = pars.T_s
        self.alpha_s = pars.alpha_s
        self.tau_M_max = pars.tau_M_max
        self.J = pars.J
        # Gain
        self.k = pars.alpha_s*pars.J
        # Integral state
        self.tau_l = 0
        # Load torque estimate (stored for the update method)
        self.tau_L = 0

    def output(self, w_M_ref, w_M):
        """
        Compute the torque reference and the load torque estimate.

        Parameters
        ----------
        w_M_ref : float
            Rotor speed reference (in mechanical rad/s).
        w_M : float
            Rotor speed (in mechanical rad/s).

        Returns
        -------
        tau_M_ref : float
            Torque reference.

        """
        self.tau_L = self.tau_l - self.alpha_s*self.J*w_M
        tau_M_ref = self.k*(w_M_ref - w_M) + self.tau_L

        if np.abs(tau_M_ref) > self.tau_M_max:
            tau_M_ref = np.sign(tau_M_ref)*self.tau_M_max

        return tau_M_ref

    def update(self, tau_M_ref_lim):
        """
        Update the integral state.

        Parameters
        ----------
        tau_M_ref_lim : float
            Realized (limited) torque reference.

        """
        self.tau_l += self.T_s*self.alpha_s*(tau_M_ref_lim - self.tau_L)


# %%
class RateLimiter:
    """
    Rate limiter.

    Parameters
    ----------
    pars : data object
        Control parameters.

    """

    def __init__(self, pars):
        self.T_s = pars.T_s
        self.limit = pars.rate_limit
        self.y_old = 0

    def __call__(self, u):
        """
        Limit the input signal.

        Parameters
        ----------
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
        rate = (u - self.y_old)/self.T_s

        if rate > self.limit:
            # Limit rising rate
            y = self.y_old + self.T_s*self.limit
        elif rate < -self.limit:
            # Limit falling rate
            y = self.y_old - self.T_s*self.limit
        else:
            y = u

        # Store the limited output
        self.y_old = y

        return y


# %%
class Ctrl:
    """Base class for main control loops."""

    def __init__(self):
        self.t = 0  # Digital clock
        self.data = Bunch()  # Data store
        self.t_reset = 1e9  # Time at which the clock is reset

    def __call__(self, mdl):
        """
        Run the main control loop.

        The main control loop is callable that returns the sampling
        period `T_s` (float)  and the duty ratio references `d_abc_ref`
        (ndarray, shape (3,)) for the next sampling period.

        Parameters
        ----------
        mdl : Model
            System model containing methods for getting the feedback signals.

        """
        raise NotImplementedError

    def update_clock(self, T_s):
        """
        Update the digital clock.

        Parameters
        ----------
        T_s : float
            Sampling period.

        """
        self.t += T_s if self.t < self.t_reset else 0

    def save(self, data):
        """
        Save the internal controller data.

        Parameters
        ----------
        data : bunch or dict
            Contains the data to be saved.

        """
        for key, value in data.items():
            self.data.setdefault(key, []).extend([value])

    def post_process(self):
        """
        Transform the lists to the ndarray format.

        This can be run after the simulation has been completed in order to
        simplify plotting and analysis of the stored data.

        """
        for key in self.data:
            self.data[key] = np.asarray(self.data[key])
