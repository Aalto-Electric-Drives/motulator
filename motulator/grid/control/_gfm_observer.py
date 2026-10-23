"""Disturbance-observer-based grid-forming control."""

from cmath import exp, phase
from dataclasses import dataclass
from math import pi, sqrt

import numpy as np

from motulator.common.control._base import TimeSeries, rotate_coordinates
from motulator.common.utils._utils import wrap
from motulator.grid.control._base import Measurements
from motulator.grid.control._common import (
    CurrentLimiter,
    PowerReferenceLimiter,
    References,
)


# %%
@dataclass
class ObserverStates:
    """States of the observer."""

    theta_c: float = 0.0
    u_g: complex = 0j  # Predicted grid voltage
    i_c: complex = 0j  # Predicted converter current


@dataclass
class ObserverOutputs:
    """Feedback signals for the control system."""

    i_c: complex = 0j
    u_c: complex = 0j  # Converter voltage (set by the control system)
    u_c_zoh: complex = 0j  # Converter voltage of the ongoing sampling period
    v_c: complex = 0j
    u_g: complex = 0j
    p_g: float = 0.0
    q_g: float = 0.0
    theta_c: float = 0.0  # Angle of the coordinate system (rad)
    w_c: float = 0.0  # Angular speed of the coordinate system (rad/s)
    u_dc: float = 0.0  # DC-bus voltage (set by the control system)


class Observer:
    """
    Disturbance observer.

    This observer estimates the grid voltage and the quasi-static converter voltage. It
    is designed directly in discrete time, using the hold-equivalent model in
    coordinates rotating at the nominal grid angular frequency. Over the sampling
    period, the converter voltage is constant in stationary coordinates, while the grid
    voltage and the resistive voltage drop are constant in the observer coordinates. At
    each sampling instant, the grid-voltage estimate is corrected using the prediction
    error of the measured current, after which the current is predicted for the next
    sampling instant, using the realized converter voltage of the ongoing sampling
    period. The observer gain places the pole of the estimation-error dynamics at
    `exp(-alpha_o*T_s)`.

    Parameters
    ----------
    u_nom : float
        Nominal grid voltage (V), line-to-neutral peak value.
    w_nom : float
        Nominal grid angular frequency (rad/s).
    L : float
        Total inductance (H).
    alpha_o : float
        Observer bandwidth (rad/s).
    T_s : float
        Sampling period (s).
    R : float, optional
        Total series resistance (Ω), defaults to 0.

    """

    def __init__(
        self,
        u_nom: float,
        w_nom: float,
        L: float,
        alpha_o: float,
        T_s: float,
        R: float = 0.0,
    ) -> None:
        self.L: float = L
        self.R: float = R
        self.w_g: float = w_nom
        # Coefficients of the hold-equivalent model
        self.phi = exp(-1j * w_nom * T_s)
        self.gamma_c = T_s * self.phi / L
        self.gamma_g = (self.phi - 1) / (1j * w_nom * L)
        self.k_o = (1 - exp(-alpha_o * T_s)) / self.gamma_g  # Observer gain
        self.state = ObserverStates(u_g=u_nom)

    def compute_output(self, u_c_ab: complex, i_c_ab: complex) -> ObserverOutputs:
        """Compute the estimates."""
        out = ObserverOutputs(theta_c=self.state.theta_c, w_c=self.w_g)

        # Transform the measured values into synchronous coordinates
        out.i_c = exp(-1j * out.theta_c) * i_c_ab
        out.u_c_zoh = exp(-1j * out.theta_c) * u_c_ab

        # Correct the predicted grid-voltage estimate
        out.u_g = self.state.u_g + self.k_o * (out.i_c - self.state.i_c)
        out.v_c = out.u_g + 1j * self.w_g * self.L * out.i_c

        # Active and reactive powers
        s_g = 1.5 * out.u_g * out.i_c.conjugate()
        out.p_g, out.q_g = s_g.real, s_g.imag

        return out

    def update(self, T_s: float, out: ObserverOutputs) -> None:
        """Update the states."""
        # Predict the current, the grid voltage being constant in these coordinates
        self.state.i_c = (
            self.phi * out.i_c
            + self.gamma_c * out.u_c_zoh
            + self.gamma_g * (out.u_g + self.R * out.i_c)
        )
        self.state.u_g = out.u_g
        self.state.theta_c = wrap(self.state.theta_c + T_s * self.w_g)


# %%
@dataclass
class ObserverBasedGridFormingControllerCfg:
    """
    Disturbance-observer-based grid-forming controller configuration.

    Parameters
    ----------
    i_max : float
        Maximum current (A), peak value.
    L : float
        Total inductance estimate (H).
    R : float, optional
        Total series resistance estimate (Ω), defaults to 0.
    R_a : float | None, optional
        Active resistance (Ω), defaults to `0.25*u_nom/i_max`.
    k_v : float | None, optional
        Voltage control gain, defaults to `alpha_o/w_nom`.
    alpha_o : float, optional
        Observer bandwidth (rad/s), defaults to 2*pi*50.
    alpha_c : float, optional
        Current control bandwidth (rad/s), defaults to 2*pi*400.
    u_nom : float, optional
        Nominal grid voltage (V), line-to-neutral peak value, defaults to
        `sqrt(2/3)*400`.
    w_nom : float, optional
        Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.
    i_d_max : float, optional
        Maximum active current (A), peak value, for the active-power reference
        limitation. If not given, the active-power reference is not limited. A value
        somewhat below `i_max` is recommended, e.g., `i_d_max = 0.85*i_max`.
    alpha_l : float, optional
        Power-limitation bandwidth (rad/s), defaults to 2*pi*50.

    """

    i_max: float
    L: float
    R: float = 0.0
    R_a: float | None = None
    k_v: float | None = None
    alpha_o: float = 2 * pi * 50
    alpha_c: float = 2 * pi * 400
    u_nom: float = sqrt(2 / 3) * 400
    w_nom: float = 2 * pi * 50
    T_s: float = 125e-6
    i_d_max: float | None = None
    alpha_l: float = 2 * pi * 50


class ObserverBasedGridFormingController:
    """
    Disturbance-observer-based grid-forming controller.

    This implements the RFPSC-type grid-forming mode of the control method described in
    [#Nur2024]_. Transparent current control is also implemented. Optionally, the
    active-power reference is limited to a realizable level, prioritizing the reactive
    current, as described in [#Maa2026]_ (here reduced to balanced conditions). This
    limitation helps to maintain synchronism during grid-voltage sags in weak grids.

    Parameters
    ----------
    cfg : ObserverBasedGridFormingControllerCfg
        Grid-forming control configuration.

    Notes
    -----
    In this implementation, the control system operates in synchronous coordinates
    rotating at the nominal grid angular frequency, and the disturbance observer is
    designed directly in discrete time. For other implementation options, see
    [#Nur2024]_.

    References
    ----------
    .. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming
       converter control based on a disturbance observer," IEEE Trans. Power Electron.,
       2024, https://doi.org/10.1109/TPEL.2024.3433503

    .. [#Maa2026] Määttä, Hinkkanen, Nurminen, Karaca, Mourouvin, Kukkola, Harnefors,
       "Disturbance-observer-based grid-forming control for unbalanced grids," 2026,
       https://arxiv.org/abs/2608.11857

    """

    grid_forming = True
    dc_bus_forming = False
    k_comp = 1.5

    def __init__(self, cfg: ObserverBasedGridFormingControllerCfg) -> None:
        self.cfg = cfg
        self.observer = Observer(
            u_nom=cfg.u_nom,
            w_nom=cfg.w_nom,
            L=cfg.L,
            alpha_o=cfg.alpha_o,
            T_s=cfg.T_s,
            R=cfg.R,
        )
        self.current_limiter = CurrentLimiter(cfg.i_max)
        self.R_a = 0.25 * cfg.u_nom / cfg.i_max if cfg.R_a is None else cfg.R_a
        self.k_v = cfg.alpha_o / cfg.w_nom if cfg.k_v is None else cfg.k_v
        self.k_c = cfg.alpha_c * cfg.L  # Current control gain
        self.T_s: float = cfg.T_s
        self.power_limiter = (
            None
            if cfg.i_d_max is None
            else PowerReferenceLimiter(cfg.i_d_max, cfg.alpha_l, cfg.u_nom)
        )

    def get_feedback(self, u_c_ab: complex, meas: Measurements) -> ObserverOutputs:
        """Get the feedback signals."""
        return self.observer.compute_output(u_c_ab, meas.i_c_ab)

    def compute_output(
        self, p_g_ref: float, v_c_ref: float, fbk: ObserverOutputs
    ) -> References:
        """Compute references."""
        # Limit the active-power reference
        if self.power_limiter is not None:
            p_g_ref = self.power_limiter(p_g_ref)
        ref = References(T_s=self.T_s, p_g=p_g_ref, v_c=v_c_ref)

        # Complex gains for grid-forming mode
        exp_j_theta = exp(1j * phase(fbk.v_c))
        k_p = exp_j_theta * self.R_a / (1.5 * ref.v_c)
        k_v = exp_j_theta * (1.0 - 1j * self.k_v)

        # Feedback correction for grid-forming mode
        e_c = k_p * (ref.p_g - fbk.p_g) + k_v * (ref.v_c - abs(fbk.v_c))

        # Transparent current limitation
        ref.i_c = fbk.i_c + e_c / self.k_c
        ref.i_c = self.current_limiter(ref.i_c)
        e_c = self.k_c * (ref.i_c - fbk.i_c)

        # Voltage reference
        ref.u_c = e_c + fbk.v_c + self.observer.R * fbk.i_c
        return ref

    def compute_standby_output(self, fbk: ObserverOutputs) -> References:
        """Compute references while the gates are blocked."""
        # The voltage reference equals the estimated quasi-static converter voltage
        ref = References(T_s=self.T_s, p_g=fbk.p_g, v_c=abs(fbk.v_c), i_c=fbk.i_c)
        ref.u_c = fbk.v_c + self.observer.R * fbk.i_c
        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s, fbk)
        if self.power_limiter is not None:
            self.power_limiter.update(ref.T_s, ref.u_c, fbk.i_c)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
        transform_to_v_c_coordinates(ts)


def transform_to_v_c_coordinates(ts: TimeSeries) -> None:
    """Transform the controller time series to the coordinates aligned with `v_c`."""
    rotate_coordinates(ts, np.angle(ts.fbk.v_c))
