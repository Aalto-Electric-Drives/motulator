"""Open-loop V/Hz control of induction machine drives."""

from cmath import exp
from collections.abc import Callable
from dataclasses import dataclass

from motulator.common.control._base import TimeSeries
from motulator.common.utils._utils import wrap
from motulator.drive.control._common import References


# %%
@dataclass
class OpenLoopVHzFeedbacks:
    """Feedback signals of the open-loop V/Hz controller."""

    u_dc: float = 0.0  # DC-bus voltage
    i_s: complex = 0j  # Stator current
    # Stator voltage averaged over the previous and ongoing sampling periods, set by
    # the control system
    u_s: complex = 0j
    u_s_zoh: complex = 0j  # Stator voltage averaged over the ongoing sampling period
    w_M: float = 0.0  # Rate-limited speed reference (mechanical rad/s)
    w_s: float = 0.0  # Stator angular frequency
    w_c: float = 0.0  # Angular speed of the coordinate system
    theta_c: float = 0.0  # Coordinate system angle


# %%
class OpenLoopVHzController:
    """
    Open-loop V/Hz controller for induction machine drives.

    The stator voltage reference is given by a V/Hz curve `u_s_ref` as a function of the
    stator angular frequency ``w_s = n_p*w_M_ref``, where `w_M_ref` is the rate-limited
    speed reference, in coordinates rotating at `w_s`. The slip is not compensated for.
    The linear curve ``1j*w_s*psi_s_nom`` gives the stator flux linkage `psi_s_nom` if
    the resistive voltage drop is negligible. With the stator inductance `L_s`, the
    curve ``(R_s/L_s + 1j*w_s)*psi_s_nom`` also includes the resistive voltage drop at
    no load.

    Parameters
    ----------
    n_p : int
        Number of pole pairs.
    u_s_ref : Callable[[float], complex]
        V/Hz curve, i.e., the stator voltage reference (V) as a function of the stator
        angular frequency (rad/s).
    T_s : float, optional
        Sampling period (s), defaults to 250e-6.

    """

    k_comp = 1.5

    def __init__(
        self, n_p: int, u_s_ref: Callable[[float], complex], T_s: float = 250e-6
    ) -> None:
        self.n_p = n_p
        self.u_s_ref = u_s_ref
        self.T_s = T_s
        self.theta_c: float = 0.0  # Angle of the coordinate system

    def get_feedback(
        self, u_s_ab: complex | None, i_s_ab: complex, w_M_ref: float
    ) -> OpenLoopVHzFeedbacks:
        """Get feedback signals, `u_s_ab` None if unknown (blocked gates)."""
        fbk = OpenLoopVHzFeedbacks(w_M=w_M_ref, theta_c=self.theta_c)
        fbk.w_c = fbk.w_s = self.n_p * w_M_ref
        fbk.i_s = exp(-1j * fbk.theta_c) * i_s_ab
        if u_s_ab is not None:
            fbk.u_s_zoh = exp(-1j * fbk.theta_c) * u_s_ab
        return fbk

    def compute_output(self, fbk: OpenLoopVHzFeedbacks) -> References:
        """Compute references."""
        return References(T_s=self.T_s, u_s=self.u_s_ref(fbk.w_s))

    def compute_standby_output(self, fbk: OpenLoopVHzFeedbacks) -> References:
        """Compute references while the gates are blocked."""
        return self.compute_output(fbk)

    def update(self, ref: References, fbk: OpenLoopVHzFeedbacks) -> None:
        """Update the angle of the coordinate system."""
        self.theta_c = wrap(self.theta_c + ref.T_s * fbk.w_c)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series, without changes here."""
