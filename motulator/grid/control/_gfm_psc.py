"""Power-synchronization control for grid converters."""

from cmath import exp
from dataclasses import dataclass
from math import pi, sqrt

from motulator.common.control._base import TimeSeries
from motulator.common.utils._utils import wrap
from motulator.grid.control._base import Measurements
from motulator.grid.control._common import CurrentLimiter, References


# %%
@dataclass
class Feedbacks:
    """Feedback signals for the control system."""

    i_c: complex = 0j
    u_c: complex = 0j
    p_g: float = 0.0
    theta_c: float = 0.0  # Angle of the coordinate system (rad)
    w_c: float = 0.0  # Angular speed of the coordinate system (rad/s)
    u_dc: float = 0.0  # DC-bus voltage (set by the control system)


@dataclass
class PowerSynchronizationControllerCfg:
    """
    Configuration for the reference-feedforward power-synchronization controller.

    Parameters
    ----------
    i_max : float
        Maximum current (A), peak value.
    u_nom : float, optional
        Nominal grid voltage (V), line-to-neutral peak value, defaults to sqrt(2/3)*400.
    w_nom : float, optional
        Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
    R : float, optional
        Total series resistance (Ω), defaults to 0.
    R_a : float | None, optional
        Active resistance (Ω), defaults to ``0.25*u_nom/i_max``.
    w_b : float, optional
        Low-pass filter bandwidth (rad/s), defaults to 2*pi*5.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    """

    i_max: float
    u_nom: float = sqrt(2 / 3) * 400
    w_nom: float = 2 * pi * 50
    R: float = 0.0
    R_a: float | None = None
    w_b: float = 2 * pi * 5
    T_s: float = 125e-6


class PowerSynchronizationController:
    """
    Reference-feedforward power-synchronization controller.

    This implements the reference-feedforward power-synchronization control [#Har2020]_.

    Parameters
    ----------
    cfg : PowerSynchronizationControllerCfg
        Configuration object.

    References
    ----------
    .. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward
       power-synchronization control," IEEE Trans. Power Electron., 2020,
       https://doi.org/10.1109/TPEL.2020.2970991

    """

    grid_forming = True

    def __init__(self, cfg: PowerSynchronizationControllerCfg) -> None:
        self.cfg = cfg
        self.theta_c: float = 0.0
        self.i_c_flt: complex = 0j
        self.current_limiter = CurrentLimiter(cfg.i_max)
        self.R_a = 0.25 * cfg.u_nom / cfg.i_max if cfg.R_a is None else cfg.R_a
        self.k_p_psc = cfg.w_nom * self.R_a / (1.5 * cfg.u_nom**2)

    def get_feedback(self, u_c_ab: complex, meas: Measurements) -> Feedbacks:
        """Get the feedback signals."""
        out = Feedbacks(theta_c=self.theta_c)

        # Transform the measured values into synchronous coordinates
        out.i_c = exp(-1j * out.theta_c) * meas.i_c_ab
        out.u_c = exp(-1j * out.theta_c) * u_c_ab

        # Other feedback signals
        p_loss = 1.5 * self.cfg.R * abs(out.i_c) ** 2
        out.p_g = 1.5 * (out.u_c * out.i_c.conjugate()).real - p_loss
        return out

    def compute_output(
        self, p_g_ref: float, v_c_ref: float, fbk: Feedbacks
    ) -> References:
        """Compute references."""
        ref = References(T_s=self.cfg.T_s, v_c=v_c_ref, p_g=p_g_ref)

        # Power droop
        fbk.w_c = self.cfg.w_nom + self.k_p_psc * (ref.p_g - fbk.p_g)

        # Current reference: feedforward from the power reference in the d-axis and
        # the low-pass-filtered current in the q-axis
        ref.i_c = ref.p_g / (1.5 * ref.v_c) + 1j * self.i_c_flt.imag
        # ref.i_c = self.i_c_flt  # Conventional PSC
        ref.i_c = self.current_limiter(ref.i_c)

        # Voltage reference
        ref.u_c = ref.v_c + self.R_a * (ref.i_c - fbk.i_c) + self.cfg.R * fbk.i_c
        return ref

    def update(self, ref: References, fbk: Feedbacks) -> None:
        """Update states."""
        self.i_c_flt += ref.T_s * self.cfg.w_b * (fbk.i_c - self.i_c_flt)
        self.theta_c += ref.T_s * fbk.w_c
        self.theta_c = wrap(self.theta_c)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
