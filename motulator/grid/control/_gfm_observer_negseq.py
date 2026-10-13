"""Disturbance-observer-based grid-forming control, with negative sequence."""

from cmath import exp, phase
from dataclasses import dataclass
from math import cos, pi, sqrt

import numpy as np

from motulator.common.control._base import TimeSeries
from motulator.common.utils._utils import wrap
from motulator.grid.control._base import Measurements
from motulator.grid.control._common import References


# %%
class CurrentReferenceLimiter:
    """Current reference limiter."""

    def __init__(self, i_max: float) -> None:
        self.i_max = i_max

    def __call__(
        self, ref: "References", i_ref_p: complex, i_ref_n: complex
    ) -> "References":
        phi = phase(i_ref_p) + phase(i_ref_n)
        x = max(cos(phi), cos(2 * pi / 3 + phi), cos(-2 * pi / 3 + phi))
        i_x = sqrt(
            (abs(i_ref_p)) ** 2
            + (abs(i_ref_n)) ** 2
            + 2 * x * abs(i_ref_p) * abs(i_ref_n)
        )
        gamma = self.i_max / max(self.i_max, i_x)
        ref.i_cp = gamma * ref.i_cp
        ref.i_cn = gamma * ref.i_cn

        return ref


class PowerReferenceLimiter:
    """Power reference limiter."""

    def __init__(self, alpha_l: float, i_d_max: float) -> None:
        self.alpha_l = alpha_l
        self.i_d_max = i_d_max
        self.p_flt: float = 0.0

    def __call__(self, p_g_ref: float) -> float:
        p_max = np.sign(p_g_ref) * min(abs(p_g_ref), self.p_flt)
        return p_max

    def update(self, ref: "References", fbk: "ObserverOutputs") -> None:
        # Transform to converter-output-voltage coordinates
        T = np.exp(-1j * np.angle(fbk.v_c))
        u_cp = T * ref.u_cp
        i_cp = T * fbk.i_cp
        i_cn = T * fbk.i_cn

        if abs(ref.u_cp) > 0:
            i_q = (u_cp * i_cp.conjugate()).imag / abs(u_cp)
        else:
            i_q = 0
        i_ref = (self.i_d_max - abs(i_cn)) ** 2 - i_q**2
        i_sat = min(max(i_ref, 0), self.i_d_max**2)
        p_lim = 1.5 * sqrt(i_sat) * abs(u_cp)
        self.p_flt += ref.T_s * self.alpha_l * (p_lim - self.p_flt)


# %%
@dataclass
class ObserverOutputs:
    """Feedback signals for the control system."""

    i_c: complex = 0j
    i_cp: complex = 0j
    i_cn: complex = 0j
    i_f: complex = 0j
    u_c: complex = 0j
    u_cp: complex = 0j
    u_cn: complex = 0j
    v_c: complex = 0j
    v_cp: complex = 0j
    v_cn: complex = 0j
    u_gp: complex = 0j
    u_gn: complex = 0j
    p_g: float = 0.0
    p_g_av: float = 0.0
    p_gp: float = 0.0
    p_gn: float = 0.0
    q_g: float = 0.0
    theta_c: float = 0.0  # Angle of the coordinate system (rad)
    w_c: float = 0.0  # Angular speed of the coordinate system (rad/s)
    u_dc: float = 0.0  # DC-bus voltage (set by the control system)


def calculate_observer_gains(
    alpha_o: float, p: tuple[complex, complex], T_s: float, w_g: float, L: float
) -> tuple[complex, complex]:
    """Calculate observer gains based on given continuous-time pole locations."""

    # Calculate discrete-time poles from specified continuous-time ones
    p_z1 = exp(p[0] * alpha_o * T_s)
    p_z2 = exp(p[1] * alpha_o * T_s)
    # Calculate observer gains using pole locations
    k_o1 = -(L * w_g * exp(T_s * w_g * 3j) * (p_z1 - 1) * (p_z2 - 1) * 1j) / (
        (exp(T_s * w_g * 1j) - 1) ** 2 * (exp(T_s * w_g * 1j) + 1)
    )
    k_o2 = (
        L
        * w_g
        * (p_z1 * exp(T_s * w_g * 2j) - 1)
        * (p_z2 * exp(T_s * w_g * 2j) - 1)
        * 1j
    ) / ((exp(T_s * w_g * 1j) - 1) ** 2 * (exp(T_s * w_g * 1j) + 1))

    return (k_o1, k_o2)


@dataclass
class ObserverStates:
    """State estimates."""

    theta_c: float = 0.0
    u_gp: complex = 0j
    u_gn: complex = 0j
    i_f: complex = 0j
    # Current and voltages from the previous time step, required only for the
    # discrete-time observer implementation
    i_c_old: complex = 0j
    u_gp_old: complex = 0j
    u_gn_old: complex = 0j
    u_c: complex = 0j
    u_c_old: complex = 0j


class Observer:
    """
    Disturbance observer.

    This implements a disturbance observer, which estimates the positive- and negative-
    sequence grid voltages. Direct discrete-time design is employed, based on the
    zero-order-hold-equivalent system model.

    Parameters
    ----------
    u_nom : float
        Nominal grid voltage (V), line-to-neutral peak value.
    w_nom : float
        Nominal grid angular frequency (rad/s).
    L : float
        Total inductance (H).
    alpha_o : float, optional
        Observer gain (rad/s), defaults to 2*pi*50.
    p : tuple[complex, complex]
        Continuous-time observer poles (p.u.).
    T_s : float
        Sampling period (s).

    """

    def __init__(
        self,
        u_nom: float,
        w_nom: float,
        L: float,
        alpha_o: float,
        p: tuple[complex, complex],
        T_s: float,
    ) -> None:
        self.L: float = L
        self.w_g: float = w_nom
        self.u_cp: complex = 0j
        self.u_cn: complex = 0j
        self.gamma_c = T_s * exp(-1j * w_nom * T_s) / L
        self.gamma_gp = (exp(-1j * w_nom * T_s) - 1) / (1j * w_nom * L)
        self.gamma_gn = (exp(-2j * w_nom * T_s) - exp(-1j * w_nom * T_s)) / (
            1j * w_nom * L
        )
        self.k_op, self.k_on = calculate_observer_gains(alpha_o, p, T_s, w_nom, L)
        self.state = ObserverStates(u_gp=u_nom, u_gp_old=u_nom)

    def compute_output(
        self, u_c_ab: complex, i_c_ab: complex, u_g_ab: complex
    ) -> ObserverOutputs:
        """Compute the estimates."""
        out = ObserverOutputs(theta_c=self.state.theta_c, i_f=self.state.i_f)

        # Transform the measured values into synchronous coordinates
        out.i_c = exp(-1j * out.theta_c) * i_c_ab
        out.u_c = exp(-1j * out.theta_c) * u_c_ab

        # Extract the current and voltage sequence components
        out.i_cp = out.i_c / (1 + 1j) + 1j * out.i_f
        out.i_cn = out.i_c / (1 - 1j) - 1j * out.i_f
        out.u_cp = self.u_cp
        out.u_cn = self.u_cn

        # Estimates for the quasi-static converter voltage and the grid voltage
        out.u_gp = self.state.u_gp
        out.u_gn = self.state.u_gn
        out.v_cp = out.u_gp + 1j * self.w_g * self.L * out.i_cp
        out.v_cn = out.u_gn - 1j * self.w_g * self.L * out.i_cn
        out.v_c = out.v_cp + out.v_cn
        out.w_c = self.w_g

        # Instantaneous active and reactive powers
        s_g = 1.5 * (out.u_gp + out.u_gn) * out.i_c.conjugate()
        out.p_g, out.q_g = s_g.real, s_g.imag

        out.p_g_av = (
            1.5
            * (out.v_cp * out.i_cp.conjugate() + out.v_cn * out.i_cn.conjugate()).real
        )
        out.p_gp = 1.5 * (out.v_cp * out.i_cp.conjugate()).real
        out.p_gn = 1.5 * (out.v_cn * out.i_cn.conjugate()).real

        return out

    def update(self, ref: "References", out: ObserverOutputs) -> None:
        """Update the states."""
        e_o = (
            out.i_c
            - exp(-1j * self.w_g * ref.T_s) * self.state.i_c_old
            - self.gamma_c * self.state.u_c_old
            - self.gamma_gp * self.state.u_gp_old
            - self.gamma_gn * self.state.u_gn_old
        )
        self.state.u_gp = out.u_gp + self.k_op * e_o
        self.state.u_gn = exp(-2j * self.w_g * ref.T_s) * out.u_gn + self.k_on * e_o
        self.state.i_f += ref.T_s * self.w_g * (out.i_c - out.i_f - 1j * out.i_f)
        self.state.theta_c += ref.T_s * self.w_g
        self.state.theta_c = wrap(self.state.theta_c)

        # Save feedbacks for next time step
        self.state.i_c_old = out.i_c
        self.state.u_c_old = self.state.u_c
        self.state.u_c = exp(-1j * self.w_g * ref.T_s) * ref.u_c
        self.state.u_gp_old = out.u_gp
        self.state.u_gn_old = out.u_gn


# %%
@dataclass
class SequenceSeparatedObserverBasedGridFormingControllerCfg:
    """
    Sequence-separated disturbance-observer-based grid-forming controller configuration.

    Parameters
    ----------
    i_max : float
        Maximum current (A), peak value.
    i_d_max : float
        Maximum active current (A), peak value.
    L : float
        Total inductance estimate (H).
    R : float, optional
        Total series resistance estimate (Ω), defaults to 0.
    R_a : float | None, optional
        Active resistance (Ω), defaults to `0.2*u_nom/i_max`.
    k_v : float | None, optional
        Voltage control gain, defaults to `alpha_o/w_nom`.
    k_n : float, optional
        Negative-sequence voltage gain, defaults to 2.
    alpha_c : float, optional
        Current control bandwidth (rad/s), defaults to 2*pi*300.
    alpha_l : float, optional
        Power reference limiter bandwidth (rad/s), defaults to 2*pi*50.
    alpha_o : float, optional
        Observer gain (rad/s), defaults to 2*pi*50.
    u_nom : float, optional
        Nominal grid voltage (V), line-to-neutral peak value, defaults to
        `sqrt(2/3)*400`.
    w_nom : float, optional
        Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
    p : tuple[complex, complex], optional
        Continuous-time observer poles (p.u.). The continuous-time poles are mapped to
        discrete-time and are used for calculating the observer gains. Defaults to
        (-1 - 1j, -0.58 - 1j).
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    """

    i_max: float
    i_d_max: float
    L: float
    R: float = 0.0
    R_a: float | None = None
    k_v: float | None = None
    k_n: float = 2
    alpha_c: float = 2 * pi * 300
    alpha_l: float = 2 * pi * 50
    alpha_o: float = 2 * pi * 50
    u_nom: float = sqrt(2 / 3) * 400
    w_nom: float = 2 * pi * 50
    p: tuple[complex, complex] = (-1 - 1j, -0.58 - 1j)
    T_s: float = 125e-6


class SequenceSeparatedObserverBasedGridFormingController:
    """
    Sequence-separated disturbance-observer-based grid-forming controller.

    This implements the disturbance-observer-based control method described in
    [#Maa2026]_. The control method is designed to take into account negative sequence
    grid voltage.

    Parameters
    ----------
    cfg : ObserverBasedGridFormingControllerWithNegativeSequenceCfg
        Grid-forming control configuration.

    """

    grid_forming = True

    def __init__(
        self, cfg: SequenceSeparatedObserverBasedGridFormingControllerCfg
    ) -> None:
        self.observer = Observer(
            u_nom=cfg.u_nom,
            w_nom=cfg.w_nom,
            L=cfg.L,
            alpha_o=cfg.alpha_o,
            p=cfg.p,
            T_s=cfg.T_s,
        )
        self.power_limiter = PowerReferenceLimiter(cfg.alpha_l, cfg.i_d_max)
        self.current_limiter = CurrentReferenceLimiter(cfg.i_max)
        self.R = cfg.R
        self.R_a = 0.2 * cfg.u_nom / cfg.i_max if cfg.R_a is None else cfg.R_a
        self.k_v = cfg.alpha_o / cfg.w_nom if cfg.k_v is None else cfg.k_v
        self.k_n = cfg.k_n
        self.k_c = cfg.alpha_c * cfg.L  # Current control gain
        self.T_s = cfg.T_s

    def get_feedback(self, u_c_ab: complex, meas: Measurements) -> ObserverOutputs:
        """Get the feedback signals."""
        fbk = self.observer.compute_output(u_c_ab, meas.i_c_ab, meas.u_g_ab)
        return fbk

    def compute_output(
        self, p_g_ref: float, v_c_ref: float, fbk: ObserverOutputs
    ) -> References:
        """Compute references."""
        # Limit the active-power reference
        p_g_ref = self.power_limiter(p_g_ref)
        ref = References(T_s=self.T_s, p_g=p_g_ref, v_c=v_c_ref)

        # Complex gains for grid-forming mode
        exp_j_theta = fbk.v_cp / abs(fbk.v_cp)
        k_p = exp_j_theta * (1 + 1j) * self.R_a / (1.5 * ref.v_c)
        k_v = exp_j_theta * self.k_v
        k_n = 1j * self.k_n

        # Feedback correction for grid-forming mode
        p_loss = 1.5 * self.R * abs(fbk.i_cp) ** 2
        e_cp = k_p * (ref.p_g + p_loss - fbk.p_gp) + k_v * (ref.v_c - abs(fbk.v_cp))
        e_cn = -k_n * fbk.v_cn

        # Transparent current limitation
        ref.i_cp = fbk.i_cp + e_cp / self.k_c
        ref.i_cn = fbk.i_cn + e_cn / self.k_c
        i_cp_ab_ref = exp(1j * fbk.theta_c) * ref.i_cp
        i_cn_ab_ref = exp(1j * fbk.theta_c) * ref.i_cn
        ref = self.current_limiter(ref, i_cp_ab_ref, i_cn_ab_ref)
        ref.i_c = ref.i_cp + ref.i_cn

        # Voltage reference
        ref.u_cp = fbk.v_cp + self.k_c * (ref.i_cp - fbk.i_cp)
        ref.u_cn = fbk.v_cn + self.k_c * (ref.i_cn - fbk.i_cn)
        self.observer.u_cp = ref.u_cp
        self.observer.u_cn = ref.u_cn

        # Discrete-time delay compensation
        ref.u_c = (
            exp(1.5j * self.observer.w_g * ref.T_s) * ref.u_cp
            + exp(-1.5j * self.observer.w_g * ref.T_s) * ref.u_cn
        )

        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref, fbk)
        self.power_limiter.update(ref, fbk)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
        # Transform the complex signals to the coordinates aligned with v_c
        T = np.exp(-1j * np.angle(ts.fbk.v_c))
        ts.fbk.u_c = T * ts.fbk.u_c
        ts.ref.u_c = T * ts.ref.u_c
        ts.ref.u_cp = T * ts.ref.u_cp
        ts.ref.u_cn = T * ts.ref.u_cn
        ts.fbk.v_c = T * ts.fbk.v_c
        ts.fbk.v_cp = T * ts.fbk.v_cp
        ts.fbk.v_cn = T * ts.fbk.v_cn
        ts.fbk.i_c = T * ts.fbk.i_c
        ts.fbk.i_cp = T * ts.fbk.i_cp
        ts.fbk.i_cn = T * ts.fbk.i_cn
        ts.fbk.i_f = T * ts.fbk.i_f
        ts.ref.i_c = T * ts.ref.i_c
        ts.ref.i_cp = T * ts.ref.i_cp
        ts.ref.i_cn = T * ts.ref.i_cn
        ts.fbk.u_gp = T * ts.fbk.u_gp
        ts.fbk.u_gn = T * ts.fbk.u_gn
