"""Common control functions and classes."""

from collections.abc import Sequence
from dataclasses import dataclass
from math import copysign, inf, sqrt

from motulator.common.control._controllers import PIController


# %%
@dataclass
class References:
    """Reference signals of the grid converter controllers."""

    T_s: float = 0.0  # Sampling period for the next control step
    p_g: float = 0.0  # Active power reference
    q_g: float = 0.0  # Reactive power reference (not used in grid-forming control)
    v_c: float = 0.0  # Converter voltage magnitude reference (grid-forming control)
    u_c: complex = 0j  # Converter voltage reference
    i_c: complex = 0j  # Converter current reference
    u_dc: float | None = None  # DC-bus voltage reference (set by the control system)
    d_abc: Sequence[float] = (0.0, 0.0, 0.0)  # Duty ratios (set by the control system)
    enabled: bool = True  # Gate enable (set by the control system)


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


# %%
class PowerReferenceLimiter:
    """
    Limit the active-power reference to a realizable level.

    The maximum active power is computed from the voltage reference and the currents,
    prioritizing the reactive current and the negative-sequence current, and it is
    low-pass filtered.

    Parameters
    ----------
    i_d_max : float
        Maximum active current (A), peak value.
    alpha_l : float
        Bandwidth of the low-pass filter (rad/s).
    u_nom : float
        Nominal grid voltage (V), line-to-neutral peak value, which gives the initial
        maximum active power.

    """

    def __init__(self, i_d_max: float, alpha_l: float, u_nom: float) -> None:
        self.i_d_max = i_d_max
        self.alpha_l = alpha_l
        self.p_max = 1.5 * u_nom * i_d_max

    def __call__(self, p_g_ref: float) -> float:
        return copysign(min(abs(p_g_ref), self.p_max), p_g_ref)

    def update(
        self, T_s: float, u_c: complex, i_c: complex, i_cn: complex = 0j
    ) -> None:
        """
        Update the maximum active power.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_c : complex
            Converter voltage reference (V), its positive-sequence component in
            unbalanced conditions.
        i_c : complex
            Converter current (A), its positive-sequence component in unbalanced
            conditions.
        i_cn : complex, optional
            Negative-sequence converter current (A), defaults to 0.

        """
        abs_u_c = max(abs(u_c), 1e-6)
        i_q = (u_c * i_c.conjugate()).imag / abs_u_c
        i_d_lim_sqr = max(max(self.i_d_max - abs(i_cn), 0.0) ** 2 - i_q**2, 0.0)
        p_max = 1.5 * abs_u_c * sqrt(i_d_lim_sqr)
        self.p_max += T_s * self.alpha_l * (p_max - self.p_max)
