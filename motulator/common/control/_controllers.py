"""Common functions and classes for controls."""

from cmath import exp
from math import inf

from motulator.common.utils._utils import clip


# %%
class PIController:
    """
    2DOF PI controller.

    This implements a discrete-time 2DOF PI controller, whose continuous-time
    counterpart is::

        u = k_t*y_ref - k_p*y + (k_i/s)*(y_ref - y) + u_ff

    where `u` is the controller output, `y_ref` is the reference signal, `y` is the
    feedback signal, `u_ff` is the feedforward signal, and `1/s` refers to integration.
    The standard PI controller is obtained by choosing ``k_t = k_p``. The integrator
    anti-windup is implemented based on the realized controller output.

    Parameters
    ----------
    k_p : float
        Proportional gain.
    k_i : float
        Integral gain.
    k_t : float, optional
        Reference-feedforward gain, defaults to `k_p`.
    u_max : float, optional
        Maximum controller output, defaults to `inf`.

    Notes
    -----
    This controller can be used, e.g., as a speed controller. In this case, `y`
    corresponds to the rotor angular speed `w_M` and `u` to the torque reference
    `tau_M_ref`.

    """

    def __init__(
        self, k_p: float, k_i: float, k_t: float | None = None, u_max: float = inf
    ) -> None:
        self.u_max = u_max
        self.k_p = k_p
        self.k_t = k_p if k_t is None else k_t
        self.alpha_i = k_i / self.k_t  # Inverse of the integration time T_i
        self.v, self.u_i = 0.0, 0.0

    def compute_output(self, y_ref: float, y: float, u_ff: float = 0.0) -> float:
        """
        Compute the controller output.

        Parameters
        ----------
        y_ref : float
            Reference signal.
        y : float
            Feedback signal.
        u_ff : float, optional
            Feedforward signal, defaults to 0.

        Returns
        -------
        u : float
            Controller output.

        """
        # Estimate of a disturbance input
        self.v = self.u_i - (self.k_p - self.k_t) * y + u_ff
        u = self.k_t * (y_ref - y) + self.v
        # Limit the controller output
        u = clip(u, -self.u_max, self.u_max)
        return u

    def update(self, T_s: float, u: float) -> None:
        """
        Update the integral state.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u : float
            Realized (limited) controller output.

        """
        self.u_i += T_s * self.alpha_i * (u - self.v)


# %%
class ComplexPIController:
    """
    2DOF synchronous-frame complex-vector PI controller.

    This implements a discrete-time 2DOF synchronous-frame complex-vector PI controller
    [#Bri2000]_. The continuous-time counterpart of the controller is::

        u = k_t*i_ref - k_p*i + (k_i + 1j*w_c*k_t)/s*(i_ref - i) + u_ff

    where `u` is the controller output, `i_ref` is the reference signal, `i` is the
    feedback signal, `w_c` is the angular speed of synchronous coordinates, `u_ff` is
    the feedforward signal, and `1/s` refers to integration. The 1DOF version is
    obtained by setting ``k_t = k_p``. The integrator anti-windup is implemented based
    on the realized controller output.

    Parameters
    ----------
    k_p : float
        Proportional gain.
    k_i : float
        Integral gain.
    k_t : float, optional
        Reference-feedforward gain, defaults to `k_p`.

    Notes
    -----
    This controller can be used, e.g., as a current controller. In this case, `i`
    corresponds to the stator current and `u` to the stator voltage.

    References
    ----------
    .. [#Bri2000] Briz, Degner, Lorenz, "Analysis and design of current regulators using
       complex vectors," IEEE Trans. Ind. Appl., 2000, https://doi.org/10.1109/28.845057

    """

    def __init__(self, k_p: float, k_i: float, k_t: float | None = None) -> None:
        self.k_p = k_p
        self.k_t = k_p if k_t is None else k_t
        self.alpha_i = k_i / self.k_t  # Inverse of the integration time T_i
        self.v: complex = 0j
        self.u_i: complex = 0j

    def compute_output(self, i_ref: complex, i: complex, u_ff: complex = 0j) -> complex:
        """
        Compute the controller output.

        Parameters
        ----------
        i_ref : complex
            Reference signal.
        i : complex
            Feedback signal.
        u_ff : complex, optional
            Feedforward signal, defaults to 0.

        Returns
        -------
        u : complex
            Controller output.

        """
        # Disturbance input estimate
        self.v = self.u_i - (self.k_p - self.k_t) * i + u_ff
        u = self.k_t * (i_ref - i) + self.v
        return u

    def update(self, T_s: float, u: complex, w_c: float) -> None:
        """
        Update the integral state.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u : complex
            Realized (limited) controller output.
        w_c : float
            Angular speed of the reference frame (rad/s).

        """
        self.u_i += T_s * (self.alpha_i + 1j * w_c) * (u - self.v)


# %%
class DiscreteComplexController:
    """
    Direct discrete-time 2DOF complex-vector controller.

    This implements a 2DOF complex-vector controller designed directly in discrete time
    [#Awa2019]_. The controlled system is an integrator in coordinates rotating at the
    angular speed `w_c`. Its hold-equivalent model, including the computational delay of
    one sampling period, is::

        i(k + 1) = Phi*(i(k) + T_s*u(k))
        u(k + 1) = Phi*u_lim(k)

    where `i` is the feedback signal, `u` is the realized output, `u_lim` is the limited
    controller output, and ``Phi = exp(-1j*w_c*T_s)``. The design gives the
    reference-tracking dynamics ``i(k) = (1 - beta)/(z*(z - beta))*i_ref(k)``, where
    ``beta = exp(-alpha_c*T_s)``. The integral action compensates for disturbances. The
    controller compensates for the delays itself, so the PWM must be configured with
    `k_comp=0`.

    Parameters
    ----------
    alpha_c : float
        Reference-tracking bandwidth (rad/s).
    T_s : float
        Sampling period (s).

    References
    ----------
    .. [#Awa2019] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
       saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
       https://doi.org/10.1109/TIA.2019.2919258

    """

    def __init__(self, alpha_c: float, T_s: float) -> None:
        self.T_s = T_s
        self.beta = exp(-alpha_c * T_s).real
        # States
        self.u_i: complex = 0j  # Integral state
        self.u_lim: complex = 0j  # Limited output of the previous sampling period
        self.w_c: float = 0.0  # Angular speed of the coordinates
        # Workspace variables
        self._e: complex = 0j
        self._u: complex = 0j

    def _gains(self, w_c: float) -> tuple[complex, complex, complex, complex]:
        """Gains of the complex-vector design, see (20) and (22) in [#Awa2019]_."""
        T_s, beta = self.T_s, self.beta
        Phi = exp(-1j * w_c * T_s)  # Rotation of the coordinates over T_s
        k_t = (1 - beta) / (Phi**2 * T_s)
        k_i = (1 - beta) * (1 - beta * Phi) / (Phi**2 * T_s**2)
        k_1 = (1 - beta) * (1 + (1 - beta) / Phi + 1 / Phi**2) / T_s
        k_2 = (1 - beta) * (1 + Phi)
        return k_t, k_i, k_1, k_2

    def compute_output(self, i_ref: complex, i: complex) -> complex:
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
        k_t, _, k_1, k_2 = self._gains(self.w_c)
        self._e = i_ref - i
        self._u = k_t * i_ref - k_1 * i - k_2 * self.u_lim + self.u_i
        return self._u

    def update(self, T_s: float, u: complex, w_c: float) -> None:
        """
        Update the states.

        Parameters
        ----------
        T_s : float
            Sampling period (s), which must equal the design value.
        u : complex
            Limited controller output.
        w_c : float
            Angular speed of the coordinates (rad/s).

        """
        k_t, k_i, _, _ = self._gains(self.w_c)
        self.u_i += T_s * k_i * (self._e - (self._u - u) / k_t)
        self.u_lim = u
        self.w_c = w_c


# %%
class RateLimiter:
    """
    Rate limiter.

    Parameters
    ----------
    rate_limit : float, optional
        Rate limit, defaults to `inf`.

    """

    def __init__(self, rate_limit: float = inf) -> None:
        self.rate_limit = rate_limit
        self._old_y = 0.0

    def __call__(self, T_s: float, u: float) -> float:
        """Limit the input signal."""
        # In this implementation, the falling rate limit equals the (negative) rising
        # rate limit. If needed, these limits can be separated with minor modifications
        # in the class.
        rate = (u - self._old_y) / T_s

        if rate > self.rate_limit:
            # Limit rising rate
            y = self._old_y + T_s * self.rate_limit
        elif rate < -self.rate_limit:
            # Limit falling rate
            y = self._old_y - T_s * self.rate_limit
        else:
            y = u

        # Store the limited output
        self._old_y = y

        return y
