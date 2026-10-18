"""Dual-voltage-forming control."""

from cmath import exp, phase
from dataclasses import dataclass
from math import pi, sqrt

from motulator.common.control._base import TimeSeries
from motulator.grid.control._base import Measurements
from motulator.grid.control._common import CurrentLimiter, References
from motulator.grid.control._gfm_observer import (
    Observer,
    ObserverOutputs,
    transform_to_v_c_coordinates,
)


# %%
@dataclass
class DualVoltageFormingOutputs(ObserverOutputs):
    """Feedback signals for the control system."""

    W_dc: float = 0.0  # DC-bus energy (J) from the measured DC-bus voltage
    p_dis: float = 0.0  # Estimated disturbance power (W), see DisturbancePowerObserver


class DisturbancePowerObserver:
    """
    Disturbance-power observer.

    This observer estimates the disturbance power, i.e., the external power fed to the
    DC bus minus the grid power and the resistive losses, from the energy balance of
    the DC-bus capacitor and the inductor [#Mou2025]_. The estimate is `p_dis = x +
    alpha_o_dc*(W_dc + 0.75*L*|i_c|**2)`, where the auxiliary state `x` is updated by
    `dx/dt = -(1.5/L)*Re{u_g*conj(e_c)} - alpha_o_dc*p_dis` and `e_c` is the realized
    feedback correction, i.e., the realized converter voltage minus the quasi-static
    converter voltage. The derivatives of the energies are thus integrated directly. On
    the first call, the disturbance power is assumed to be zero.

    Parameters
    ----------
    L : float
        Total inductance estimate (H).
    alpha_o_dc : float
        Observer bandwidth (rad/s).

    """

    def __init__(self, L: float, alpha_o_dc: float) -> None:
        self.L = L
        self.alpha_o_dc = alpha_o_dc
        self.state: float | None = None  # Auxiliary state, initialized on first call

    def compute_output(self, W_dc: float, i_c: complex) -> float:
        """Compute the disturbance power estimate (W)."""
        W = W_dc + 0.75 * self.L * abs(i_c) ** 2
        if self.state is None:
            self.state = -self.alpha_o_dc * W
        return self.state + self.alpha_o_dc * W

    def update(self, T_s: float, u_g: complex, e_c: complex, p_dis: float) -> None:
        """Update the auxiliary state."""
        if self.state is not None:
            d_state = -1.5 / self.L * (u_g * e_c.conjugate()).real
            d_state -= self.alpha_o_dc * p_dis
            self.state += T_s * d_state


# %%
@dataclass
class DualVoltageFormingControllerCfg:
    """
    Dual-voltage-forming controller configuration.

    Parameters
    ----------
    i_max : float
        Maximum current (A), peak value.
    L : float
        Total inductance estimate (H).
    C_dc : float
        DC-bus capacitance estimate (F).
    R : float, optional
        Total series resistance estimate (Ω), defaults to 0.
    k_d : float | None, optional
        DC-bus power gain (V/W), defaults to `1/(1.5*i_max)`, i.e., 1 p.u. with the
        base power `1.5*u_nom*i_max`.
    alpha_dc : float, optional
        Approximate closed-loop bandwidth of the DC-bus energy control (rad/s),
        defaults to 2*pi*7.5.
    alpha_o : float, optional
        Observer bandwidth of the quasi-static converter voltage (rad/s), defaults to
        2*pi*100.
    alpha_o_dc : float, optional
        Observer bandwidth of the disturbance power (rad/s), defaults to 2*pi*60.
    k_v : float | None, optional
        Voltage control gain, defaults to `alpha_o/w_nom`.
    alpha_c : float, optional
        Current control bandwidth (rad/s), defaults to 2*pi*400.
    u_nom : float, optional
        Nominal grid voltage (V), line-to-neutral peak value, defaults to
        `sqrt(2/3)*400`.
    w_nom : float, optional
        Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    """

    i_max: float
    L: float
    C_dc: float
    R: float = 0.0
    k_d: float | None = None
    alpha_dc: float = 2 * pi * 7.5
    alpha_o: float = 2 * pi * 100
    alpha_o_dc: float = 2 * pi * 60
    k_v: float | None = None
    alpha_c: float = 2 * pi * 400
    u_nom: float = sqrt(2 / 3) * 400
    w_nom: float = 2 * pi * 50
    T_s: float = 125e-6


class DualVoltageFormingController:
    """
    Dual-voltage-forming controller.

    This controller regulates both the DC-bus voltage and the converter output voltage
    magnitude [#Mou2025]_, extending the disturbance-observer-based grid-forming control
    [#Nur2024b]_. The disturbance observer estimates the quasi-static converter voltage
    as in `ObserverBasedGridFormingController`, and a second observer estimates the
    disturbance power, i.e., the external power fed to the DC bus minus the grid
    power and the resistive losses, from the energy balance of the DC-bus capacitor
    and the inductor. The control law feeds the estimated disturbance power forward and
    corrects the DC-bus energy error, which controls the active power without a
    cascaded DC-bus voltage controller. The disturbance observers provide the integral
    action and the synchronization. Transparent current control is also implemented.

    Parameters
    ----------
    cfg : DualVoltageFormingControllerCfg
        Dual-voltage-forming control configuration.

    Notes
    -----
    The control law equals the grid-forming mode of `ObserverBasedGridFormingController`
    with the power control gain `R_a/(1.5*v_c_ref) = k_d` and the active-power
    reference `p_g_ref = p_dis + p_g - alpha_dc*(W_dc_ref - W_dc)`, where `p_dis + p_g`
    is the estimated external power fed to the DC bus, excluding the resistive losses
    `1.5*R*|i_c|**2`. This equivalent reference is logged in `ref.p_g`.

    References
    ----------
    .. [#Mou2025] Mourouvin, Nurminen, Hinkkanen, Routimo, "Direct DC-bus control for
       grid-forming converters: Toward the concept of dual-voltage-forming converters,"
       IEEE Trans. Power Electron., 2025, https://doi.org/10.1109/TPEL.2025.3537332

    .. [#Nur2024b] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional
       grid-forming converter control based on a disturbance observer," IEEE Trans.
       Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503

    """

    grid_forming = True
    dc_bus_forming = True
    k_comp = 1.5

    def __init__(self, cfg: DualVoltageFormingControllerCfg) -> None:
        self.cfg = cfg
        self.observer = Observer(
            u_nom=cfg.u_nom,
            w_nom=cfg.w_nom,
            L=cfg.L,
            alpha_o=cfg.alpha_o,
            T_s=cfg.T_s,
            R=cfg.R,
        )
        self.power_observer = DisturbancePowerObserver(cfg.L, cfg.alpha_o_dc)
        self.current_limiter = CurrentLimiter(cfg.i_max)
        self.C_dc = cfg.C_dc
        self.k_d = 1 / (1.5 * cfg.i_max) if cfg.k_d is None else cfg.k_d
        self.alpha_dc = cfg.alpha_dc
        self.k_w = cfg.alpha_dc * self.k_d  # DC-bus energy gain
        self.k_v = cfg.alpha_o / cfg.w_nom if cfg.k_v is None else cfg.k_v
        self.k_c = cfg.alpha_c * cfg.L  # Current control gain
        self.T_s: float = cfg.T_s

    def get_feedback(
        self, u_c_ab: complex, meas: Measurements
    ) -> DualVoltageFormingOutputs:
        """Get the feedback signals."""
        fbk = DualVoltageFormingOutputs(
            **vars(self.observer.compute_output(u_c_ab, meas.i_c_ab))
        )
        fbk.W_dc = 0.5 * self.C_dc * meas.u_dc**2
        fbk.p_dis = self.power_observer.compute_output(fbk.W_dc, fbk.i_c)
        return fbk

    def compute_output(
        self, u_dc_ref: float, v_c_ref: float, fbk: DualVoltageFormingOutputs
    ) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s, v_c=v_c_ref, u_dc=u_dc_ref)
        W_dc_ref = 0.5 * self.C_dc * u_dc_ref**2
        # Equivalent active-power reference, for logging only
        ref.p_g = fbk.p_g + fbk.p_dis - self.alpha_dc * (W_dc_ref - fbk.W_dc)

        # Complex gains, aligned with the quasi-static converter voltage
        exp_j_theta = exp(1j * phase(fbk.v_c))
        k_d = exp_j_theta * self.k_d
        k_w = exp_j_theta * self.k_w
        k_v = exp_j_theta * (1.0 - 1j * self.k_v)

        # Feedback correction
        e_c = (
            k_d * fbk.p_dis
            - k_w * (W_dc_ref - fbk.W_dc)
            + k_v * (ref.v_c - abs(fbk.v_c))
        )

        # Transparent current limitation
        ref.i_c = fbk.i_c + e_c / self.k_c
        ref.i_c = self.current_limiter(ref.i_c)
        e_c = self.k_c * (ref.i_c - fbk.i_c)

        # Voltage reference
        ref.u_c = e_c + fbk.v_c + self.observer.R * fbk.i_c
        return ref

    def compute_standby_output(self, fbk: DualVoltageFormingOutputs) -> References:
        """Compute references while the gates are blocked."""
        # The voltage reference equals the estimated quasi-static converter voltage
        ref = References(T_s=self.T_s, p_g=fbk.p_g, v_c=abs(fbk.v_c), i_c=fbk.i_c)
        ref.u_c = fbk.v_c + self.observer.R * fbk.i_c
        return ref

    def update(self, ref: References, fbk: DualVoltageFormingOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s, fbk)
        # The realized feedback correction, including the current and voltage
        # limitations. The realized voltage is averaged over the ongoing sampling
        # period in the observer coordinates, i.e., rotated by half a sampling period.
        u_c = fbk.u_c_zoh * exp(-0.5j * fbk.w_c * ref.T_s)
        e_c = u_c - fbk.v_c - self.observer.R * fbk.i_c
        self.power_observer.update(ref.T_s, fbk.u_g, e_c, fbk.p_dis)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
        transform_to_v_c_coordinates(ts)
