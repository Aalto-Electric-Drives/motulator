"""Power-synchronization control for grid converters."""

from cmath import exp
from dataclasses import dataclass, field
from math import pi
from typing import Sequence

from motulator.common.control._base import TimeSeries
from motulator.common.control._pwm import PWM
from motulator.common.utils import wrap
from motulator.grid.control._base import Measurements
from motulator.grid.control._controllers import CurrentLimiter


# %%
@dataclass
class References:
    """Reference signals for power-synchronization control."""

    T_s: float = 0.0
    d_abc: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    u_c: complex = 0j
    i_c: complex = 0j
    v_c: float = 0.0
    p_g: float = 0.0
    w_c: float = 0.0
    u_dc: float | None = None


@dataclass
class Feedbacks:
    """Feedback signals for the control system."""

    u_dc: float = 0.0
    i_c: complex = 0j
    u_c: complex = 0j
    theta_c: float = 0.0
    p_g: float = 0.0


class PowerSynchronizationController:
    """
    Reference-feedforward power-synchronization controller.

    This implements the reference-feedforward power-synchronization control [#Har2020]_.

    Parameters
    ----------
    u_nom : float
        Nominal grid voltage (V), line-to-neutral peak value.
    w_nom : float
        Nominal grid angular frequency (rad/s).
    i_max : float
        Maximum current (A), peak value.
    R : float, optional
        Total series resistance (Ω), defaults to 0.
    R_a : float, optional
        Active resistance (Ω), defaults to 0.25*u_nom/i_max.
    w_b : float, optional
        Low-pass filter bandwidth (rad/s), defaults to 2*pi*5.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    References
    ----------
    .. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward
       power-synchronization control," IEEE Trans. Power Electron., 2020,
       https://doi.org/10.1109/TPEL.2020.2970991

    """

    def __init__(
        self,
        u_nom: float,
        w_nom: float,
        i_max: float,
        R: float = 0.0,
        R_a: float | None = None,
        w_b: float = 2 * pi * 5,
        T_s: float = 125e-6,
    ) -> None:
        self.pwm = PWM()
        self.theta_c: float = 0.0
        self.i_c_flt: complex = 0j
        self.current_limiter = CurrentLimiter(i_max)
        self.w_nom = w_nom
        self.R = R
        self.w_b = w_b
        self.R_a = 0.25 * u_nom / i_max if R_a is None else R_a
        self.k_p_psc = w_nom * self.R_a / (1.5 * u_nom**2)
        self.T_s = T_s

    def get_feedback(self, meas: Measurements) -> Feedbacks:
        """Get the feedback signals."""
        out = Feedbacks(u_dc=meas.u_dc, theta_c=self.theta_c)

        # Transform the measured values into synchronous coordinates
        u_c_ab = self.pwm.get_realized_voltage()
        out.i_c = exp(-1j * out.theta_c) * meas.i_c_ab
        out.u_c = exp(-1j * out.theta_c) * u_c_ab

        # Other feedback signals
        p_loss = 1.5 * self.R * abs(out.i_c) ** 2
        out.p_g = 1.5 * (out.u_c * out.i_c.conjugate()).real - p_loss
        return out

    def compute_output(
        self, p_g_ref: float, v_c_ref: float, fbk: Feedbacks
    ) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s, v_c=v_c_ref, p_g=p_g_ref)

        # Power droop
        ref.w_c = self.w_nom + self.k_p_psc * (ref.p_g - fbk.p_g)

        # Optionally, use of reference feedforward for d-axis current
        ref.i_c = ref.p_g / (1.5 * ref.v_c) + 1j * self.i_c_flt.imag
        # ref.i_c = self.i_c_flt  # Conventional PSC
        ref.i_c = self.current_limiter(ref.i_c)

        # Voltage reference
        ref.u_c = ref.v_c + self.R_a * (ref.i_c - fbk.i_c) + self.R * fbk.i_c
        u_c_ab_ref = exp(1j * fbk.theta_c) * ref.u_c
        ref.d_abc = self.pwm(ref.T_s, u_c_ab_ref, fbk.u_dc, ref.w_c)

        return ref

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update states."""
        self.i_c_flt += ref.T_s * self.w_b * (fbk.i_c - self.i_c_flt)
        self.theta_c += ref.T_s * ref.w_c
        self.theta_c = wrap(self.theta_c)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
