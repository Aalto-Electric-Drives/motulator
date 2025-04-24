"""Common control functions and classes."""

from math import inf

from motulator.common.control._controllers import PIController


# %%
class DCBusVoltageController(PIController):
    """
    DC-bus voltage PI controller.

    This controller regulates the energy stored in the DC-bus capacitor (scaled square
    of the DC-bus voltage) in order to have a linear closed-loop system [#Hur2001]_.

    Parameters
    ----------
    C_dc : float
        DC-bus capacitance (F).
    alpha_dc : float
        Approximate closed-loop bandwidth (rad/s).
    p_max : float, optional
        Limit for the maximum converter power (W), defaults to `inf`.

    References
    ----------
    .. [#Hur2001] Hur, Jung, Nam, "A fast dynamic DC-link power-balancing scheme for a
       PWM converter-inverter system," IEEE Trans. Ind. Electron., 2001,
       https://doi.org/10.1109/41.937412

    """

    def __init__(self, C_dc: float, alpha_dc: float, p_max: float = inf) -> None:
        k_p = -2 * alpha_dc
        k_i = -(alpha_dc**2)
        k_t = k_p
        super().__init__(k_p, k_i, k_t, p_max)
        self.C_dc = C_dc

    def compute_output(self, y_ref: float, y: float, u_ff: float = 0.0) -> float:
        W_dc_ref = 0.5 * self.C_dc * y_ref**2  # y_ref = u_dc_ref
        W_dc = 0.5 * self.C_dc * y**2  # y = u_dc
        return super().compute_output(W_dc_ref, W_dc, u_ff)


# %%
class CurrentLimiter:
    """
    Limit the amplitude of the input signal.

    Parameters
    ----------
    i_max : float
        Maximum current (A).

    Returns
    -------
    complex
        Limited signal.

    """

    def __init__(self, i_max: float) -> None:
        self.i_max = i_max

    def __call__(self, i: complex) -> complex:
        if abs(i) > self.i_max:
            i = self.i_max * i / abs(i)
        return i
