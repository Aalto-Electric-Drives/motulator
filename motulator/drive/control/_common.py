"""Common control functions and classes for machine drives."""

from cmath import exp
from math import expm1, inf, pi

import numpy as np

from motulator.common.control._controllers import PIController


# %%
class SpeedController(PIController):
    """
    2DOF PI speed controller.

    This is an interface for a speed controller. The gains are initialized based on the
    desired closed-loop bandwidth and the rotor inertia estimate.

    Parameters
    ----------
    J : float
        Total inertia of the rotor (kgm²).
    alpha_s : float
        Reference-tracking bandwidth (rad/s).
    alpha_i : float, optional
        Integral action bandwidth (rad/s), defaults to `alpha_s`.
    tau_M_max : float, optional
        Maximum motor torque (Nm), defaults to `inf`.

    """

    def __init__(
        self,
        J: float,
        alpha_s: float,
        alpha_i: float | None = None,
        tau_M_max: float = inf,
    ) -> None:
        alpha_i = alpha_s if alpha_i is None else alpha_i
        k_p = (alpha_s + alpha_i) * J
        k_i = alpha_s * alpha_i * J
        k_t = alpha_s * J
        super().__init__(k_p, k_i, k_t, tau_M_max)


# %%
class SpeedObserver:
    """
    Speed observer.

    This observer estimates the mechanical rotor speed based on the mechanical system
    model and the error signal. If the inertia of the mechanical system is provided, the
    load torque is also estimated, avoiding lag in the speed estimate [#Lor1991]_.

    Parameters
    ----------
    k_w : float
        Speed-estimation gain (rad/s or (rad/s)² for induction machines or for
        synchronous machines, respectively).
    k_tau : float
        Load-torque estimation gain (Nm or Nm/s for induction machines or for
        synchronous machines, respectively).
    J : float | None
        Inertia of the mechanical system (kgm²). If None, the load torque estimation is
        not used.

    References
    ----------
    .. [#Lor1991] Lorenz, Van Patten, "High-resolution velocity estimation for
       all-digital, AC servo drives," IEEE Trans. Ind. Appl., 1991,
       https://doi.org/10.1109/28.85485

    """

    def __init__(self, k_w: float, k_tau: float, J: float | None) -> None:
        self.k_w = k_w
        self.k_tau = k_tau
        self.J = J
        self.w_M: float = 0.0
        self.tau_L: float = 0.0

    def compute_output(self) -> tuple[float, float]:
        """Compute outputs."""
        return self.w_M, self.tau_L

    def update(self, T_s: float, eps: float, tau_M: float = 0.0) -> None:
        """
        Update mechanical state estimates.

        Parameters
        ----------
        T_s : float
            Sample time (s).
        eps : float
            Estimation error signal: mechanical speed (rad/s) or mechanical position
            (rad) for induction machines or synchronous machines, respectively.
        tau_M : float, optional
            Electromagnetic torque estimate (Nm).

        """
        if self.J is None:
            d_w_M = self.k_w * eps
            d_tau_L = 0.0
        else:
            d_w_M = (tau_M - self.tau_L) / self.J + self.k_w * eps
            d_tau_L = -self.k_tau * eps

        self.w_M += T_s * d_w_M
        self.tau_L += T_s * d_tau_L


# %%
def hold_equivalent(T_s: float, w_c: float) -> tuple[complex, complex]:
    """
    Hold-equivalent discretization of a vector integrator in rotating coordinates.

    The coefficients discretize ``d(psi)/dt = u + v - 1j*w_c*psi``, where `psi` is in
    coordinates rotating at `w_c`, assuming that `u` is constant in stator coordinates
    and `v` is constant in the rotating coordinates over the sampling period. Then,
    ``psi(k+1) = Phi*(psi(k) + T_s*u(k)) + Gamma*v(k)``, where `u(k)` and `v(k)` are in
    the coordinates at the instant `k`.

    Parameters
    ----------
    T_s : float
        Sampling period (s).
    w_c : float
        Angular speed of the coordinate system (rad/s).

    Returns
    -------
    Phi : complex
        Rotation over the sampling period, ``exp(-1j*w_c*T_s)``.
    Gamma : complex
        Input gain of the rotating-coordinate input, ``(1 - Phi)/(1j*w_c)``.

    """
    # Half-angle form, requiring a single sine-cosine pair
    x = 0.5 * w_c * T_s
    exp_jx = exp(-1j * x)
    return exp_jx * exp_jx, T_s * float(np.sinc(x / pi)) * exp_jx


def matched_pole(alpha: float, T_s: float) -> float:
    """
    Pole for the gain formulas of estimators updated by the forward Euler method.

    The forward Euler method maps the continuous-time pole `s` to the discrete-time
    pole ``1 + s*T_s``. Using the returned value instead of `alpha` in the gain
    formulas of a design with the poles at ``s = -alpha`` places the discrete-time
    poles at ``exp(-alpha*T_s)``. This holds for the constant sampling period `T_s`.

    Parameters
    ----------
    alpha : float
        Continuous-time pole (rad/s).
    T_s : float
        Sampling period (s).

    Returns
    -------
    float
        Matched pole (rad/s), ``(1 - exp(-alpha*T_s))/T_s``.

    """
    return -expm1(-alpha * T_s) / T_s
