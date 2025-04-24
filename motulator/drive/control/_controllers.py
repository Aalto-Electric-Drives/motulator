"""Common control functions and classes for machine drives."""

from math import inf

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
