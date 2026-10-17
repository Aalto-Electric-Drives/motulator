"""Disturbance-observer-based grid-forming control with sequence separation."""

from cmath import exp, phase
from dataclasses import dataclass
from math import cos, pi, sqrt

import numpy as np

from motulator.common.control._base import TimeSeries
from motulator.common.utils._utils import wrap
from motulator.grid.control._base import Measurements
from motulator.grid.control._common import PowerReferenceLimiter, References


# %%
@dataclass
class SequenceReferences(References):
    """Reference signals, including the sequence components."""

    u_cp: complex = 0j  # Positive-sequence converter-voltage reference
    u_cn: complex = 0j  # Negative-sequence converter-voltage reference
    i_cp: complex = 0j  # Positive-sequence converter-current reference
    i_cn: complex = 0j  # Negative-sequence converter-current reference


class SequenceCurrentLimiter:
    """
    Limit the peak phase current of the sequence current references.

    Both sequence components are scaled by the same factor, which preserves the
    elliptical trajectory of the current vector.

    Parameters
    ----------
    i_max : float
        Maximum current (A), peak value.

    """

    def __init__(self, i_max: float) -> None:
        self.i_max = i_max

    def __call__(
        self, i_cp: complex, i_cn: complex, theta_c: float
    ) -> tuple[complex, complex]:
        # Peak phase current, using the angles in stationary coordinates
        phi = phase(i_cp) + phase(i_cn) + 2 * theta_c
        x = max(cos(phi + 2 * pi * n / 3) for n in (-1, 0, 1))
        abs_i_cp, abs_i_cn = abs(i_cp), abs(i_cn)
        i_ph_max = sqrt(abs_i_cp**2 + abs_i_cn**2 + 2 * x * abs_i_cp * abs_i_cn)
        gamma = self.i_max / max(self.i_max, i_ph_max)
        return gamma * i_cp, gamma * i_cn


# %%
@dataclass
class SequenceObserverOutputs:
    """Feedback signals for the control system."""

    i_c: complex = 0j
    i_cp: complex = 0j
    i_cn: complex = 0j
    i_f: complex = 0j  # Filtered current for the sequence separation
    u_c: complex = 0j  # Converter voltage (set by the control system)
    u_c_zoh: complex = 0j  # Converter voltage of the ongoing sampling period
    v_c: complex = 0j
    v_cp: complex = 0j
    v_cn: complex = 0j
    u_gp: complex = 0j
    u_gn: complex = 0j
    p_g: float = 0.0
    p_gp: float = 0.0
    p_gn: float = 0.0
    q_g: float = 0.0
    theta_c: float = 0.0  # Angle of the coordinate system (rad)
    w_c: float = 0.0  # Angular speed of the coordinate system (rad/s)
    u_dc: float = 0.0  # DC-bus voltage (set by the control system)


@dataclass
class SequenceObserverStates:
    """States of the observer."""

    theta_c: float = 0.0
    u_gp: complex = 0j  # Predicted positive-sequence grid voltage
    u_gn: complex = 0j  # Predicted negative-sequence grid voltage
    i_c: complex = 0j  # Predicted converter current
    i_f: complex = 0j  # Filtered current


class SequenceObserver:
    """
    Disturbance observer for the grid-voltage sequence components.

    This observer estimates the positive- and negative-sequence grid voltages. It is
    designed directly in discrete time, using the hold-equivalent model in coordinates
    rotating at the nominal grid angular frequency. At each sampling instant, the
    estimates are corrected using the prediction error of the measured current, after
    which the current and the estimates are predicted for the next sampling instant,
    using the realized converter voltage of the ongoing sampling period.
    The observer gains place the poles of the estimation-error dynamics at
    `exp(p*alpha_o*T_s)`.

    The converter current is separated into its quasi-static sequence components using
    a complex first-order filter. Its discrete-time coefficients are selected such
    that the separation is exact for both sequences in the periodic steady state.

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
    p : tuple[complex, complex]
        Continuous-time observer poles, normalized by `alpha_o`.
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
        # Coefficients of the hold-equivalent model
        self.phi = exp(-1j * w_nom * T_s)
        self.phi_n = self.phi**2  # State transition of the negative sequence
        self.gamma_c = T_s * self.phi / L
        self.gamma_p = (self.phi - 1) / (1j * w_nom * L)
        self.gamma_n = self.phi * self.gamma_p
        # Observer gains
        z_1, z_2 = exp(p[0] * alpha_o * T_s), exp(p[1] * alpha_o * T_s)
        self.k_p = (1 - z_1) * (1 - z_2) / ((1 - self.phi_n) * self.gamma_p)
        self.k_n = (
            (self.phi_n - z_1) * (self.phi_n - z_2) / ((self.phi_n - 1) * self.gamma_n)
        )
        # Coefficients of the sequence-separation filter
        self.a_f = 2j * self.phi_n / (1 + 1j - (1 - 1j) * self.phi_n)
        self.b_f = (1 - self.a_f) / (1 + 1j)
        self.state = SequenceObserverStates(u_gp=u_nom)

    def compute_output(
        self, u_c_ab: complex, i_c_ab: complex
    ) -> SequenceObserverOutputs:
        """Compute the estimates."""
        out = SequenceObserverOutputs(theta_c=self.state.theta_c, w_c=self.w_g)

        # Transform the measured values into synchronous coordinates
        out.i_c = exp(-1j * out.theta_c) * i_c_ab
        out.u_c_zoh = exp(-1j * out.theta_c) * u_c_ab

        # Correct the predicted grid-voltage estimates
        e_o = out.i_c - self.state.i_c
        out.u_gp = self.state.u_gp + self.k_p * e_o
        out.u_gn = self.state.u_gn + self.k_n * e_o

        # Quasi-static sequence components of the current and the converter voltage
        out.i_f = self.a_f * self.state.i_f + self.b_f * out.i_c
        out.i_cp = out.i_c / (1 + 1j) + 1j * out.i_f
        out.i_cn = out.i_c / (1 - 1j) - 1j * out.i_f
        out.v_cp = out.u_gp + 1j * self.w_g * self.L * out.i_cp
        out.v_cn = out.u_gn - 1j * self.w_g * self.L * out.i_cn
        out.v_c = out.v_cp + out.v_cn

        # Instantaneous powers and the active powers of the sequences
        s_g = 1.5 * (out.u_gp + out.u_gn) * out.i_c.conjugate()
        out.p_g, out.q_g = s_g.real, s_g.imag
        out.p_gp = 1.5 * (out.u_gp * out.i_cp.conjugate()).real
        out.p_gn = 1.5 * (out.u_gn * out.i_cn.conjugate()).real

        return out

    def update(self, T_s: float, out: SequenceObserverOutputs) -> None:
        """Update the states."""
        # Predict the current and the grid-voltage sequences
        self.state.i_c = (
            self.phi * out.i_c
            + self.gamma_c * out.u_c_zoh
            + self.gamma_p * out.u_gp
            + self.gamma_n * out.u_gn
        )
        self.state.u_gp = out.u_gp
        self.state.u_gn = self.phi_n * out.u_gn
        self.state.i_f = out.i_f
        self.state.theta_c = wrap(self.state.theta_c + T_s * self.w_g)


# %%
@dataclass
class SequenceObserverBasedGridFormingControllerCfg:
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
        Observer bandwidth (rad/s), defaults to 2*pi*50.
    u_nom : float, optional
        Nominal grid voltage (V), line-to-neutral peak value, defaults to
        `sqrt(2/3)*400`.
    w_nom : float, optional
        Nominal grid angular frequency (rad/s), defaults to 2*pi*50.
    p : tuple[complex, complex], optional
        Continuous-time observer poles, normalized by `alpha_o`, defaults to
        (-1 - 1j, -0.58 - 1j). The poles of the discrete-time observer are
        `exp(p*alpha_o*T_s)`.
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


class SequenceObserverBasedGridFormingController:
    """
    Sequence-separated disturbance-observer-based grid-forming controller.

    This implements the grid-forming mode of the control method described in
    [#Maa2026b]_, with transparent current control. The positive-sequence active power
    and converter-voltage magnitude are controlled, while the negative-sequence
    converter voltage is suppressed. The disturbance observer estimates the positive-
    and negative-sequence grid voltages. The active-power reference is limited to a
    realizable level, prioritizing the reactive and negative-sequence currents, and
    the sequence current references are limited such that the peak phase current does
    not exceed `i_max`.

    Parameters
    ----------
    cfg : SequenceObserverBasedGridFormingControllerCfg
        Grid-forming control configuration.

    Notes
    -----
    The control system operates in synchronous coordinates rotating at the nominal grid
    angular frequency. The controller compensates for the computational delay of one
    sampling period and the zero-order-hold delay of half a sampling period itself:
    the angle of the positive-sequence voltage reference is advanced by
    `1.5*w_nom*T_s`, and that of the negative sequence, which rotates in the opposite
    direction, is retarded by the same angle. Therefore, the control system does not
    advance the angle of the voltage reference (`k_comp = 0`).

    References
    ----------
    .. [#Maa2026b] Määttä, Hinkkanen, Nurminen, Karaca, Mourouvin, Kukkola, Harnefors,
       "Disturbance-observer-based grid-forming control for unbalanced grids," 2026,
       https://arxiv.org/abs/2608.11857

    """

    grid_forming = True
    dc_bus_forming = False
    k_comp = 0.0

    def __init__(self, cfg: SequenceObserverBasedGridFormingControllerCfg) -> None:
        self.observer = SequenceObserver(
            u_nom=cfg.u_nom,
            w_nom=cfg.w_nom,
            L=cfg.L,
            alpha_o=cfg.alpha_o,
            p=cfg.p,
            T_s=cfg.T_s,
        )
        self.power_limiter = PowerReferenceLimiter(cfg.i_d_max, cfg.alpha_l, cfg.u_nom)
        self.current_limiter = SequenceCurrentLimiter(cfg.i_max)
        self.R = cfg.R
        self.R_a = 0.2 * cfg.u_nom / cfg.i_max if cfg.R_a is None else cfg.R_a
        self.k_v = cfg.alpha_o / cfg.w_nom if cfg.k_v is None else cfg.k_v
        self.k_n = cfg.k_n
        self.k_c = cfg.alpha_c * cfg.L  # Current control gain
        self.T_s = cfg.T_s
        # Angle advance of the positive sequence for the delay compensation
        self.exp_j_delay = exp(1.5j * cfg.w_nom * cfg.T_s)

    def get_feedback(
        self, u_c_ab: complex, meas: Measurements
    ) -> SequenceObserverOutputs:
        """Get the feedback signals."""
        return self.observer.compute_output(u_c_ab, meas.i_c_ab)

    def compute_output(
        self, p_g_ref: float, v_c_ref: float, fbk: SequenceObserverOutputs
    ) -> SequenceReferences:
        """Compute references."""
        # Limit the active-power reference
        p_g_ref = self.power_limiter(p_g_ref)
        ref = SequenceReferences(T_s=self.T_s, p_g=p_g_ref, v_c=v_c_ref)

        # Complex gains for grid-forming mode
        exp_j_theta = exp(1j * phase(fbk.v_cp))
        k_p = exp_j_theta * (1 + 1j) * self.R_a / (1.5 * ref.v_c)
        k_v = exp_j_theta * self.k_v

        # Feedback correction for grid-forming mode
        p_loss = 1.5 * self.R * abs(fbk.i_cp) ** 2
        e_cp = k_p * (ref.p_g + p_loss - fbk.p_gp) + k_v * (ref.v_c - abs(fbk.v_cp))
        e_cn = -1j * self.k_n * fbk.v_cn

        # Transparent current limitation
        ref.i_cp, ref.i_cn = self.current_limiter(
            fbk.i_cp + e_cp / self.k_c, fbk.i_cn + e_cn / self.k_c, fbk.theta_c
        )
        ref.i_c = ref.i_cp + ref.i_cn

        # Voltage references, compensating for the delays of both sequences
        ref.u_cp = fbk.v_cp + self.k_c * (ref.i_cp - fbk.i_cp)
        ref.u_cn = fbk.v_cn + self.k_c * (ref.i_cn - fbk.i_cn)
        ref.u_c = self.exp_j_delay * ref.u_cp + self.exp_j_delay.conjugate() * ref.u_cn

        return ref

    def compute_standby_output(
        self, fbk: SequenceObserverOutputs
    ) -> SequenceReferences:
        """Compute references while the gates are blocked."""
        # The voltage references equal the estimated quasi-static converter voltages
        ref = SequenceReferences(
            T_s=self.T_s,
            p_g=fbk.p_g,
            v_c=abs(fbk.v_cp),
            i_c=fbk.i_c,
            i_cp=fbk.i_cp,
            i_cn=fbk.i_cn,
            u_cp=fbk.v_cp,
            u_cn=fbk.v_cn,
        )
        ref.u_c = self.exp_j_delay * ref.u_cp + self.exp_j_delay.conjugate() * ref.u_cn
        return ref

    def update(self, ref: SequenceReferences, fbk: SequenceObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s, fbk)
        self.power_limiter.update(ref.T_s, ref.u_cp, fbk.i_cp, fbk.i_cn)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
        # Transform the complex signals to the coordinates aligned with v_cp
        T = np.exp(-1j * np.angle(ts.fbk.v_cp))
        ts.ref.u_c = T * ts.ref.u_c
        ts.ref.u_cp = T * ts.ref.u_cp
        ts.ref.u_cn = T * ts.ref.u_cn
        ts.ref.i_c = T * ts.ref.i_c
        ts.ref.i_cp = T * ts.ref.i_cp
        ts.ref.i_cn = T * ts.ref.i_cn
        ts.fbk.u_c = T * ts.fbk.u_c
        ts.fbk.u_c_zoh = T * ts.fbk.u_c_zoh
        ts.fbk.i_c = T * ts.fbk.i_c
        ts.fbk.i_cp = T * ts.fbk.i_cp
        ts.fbk.i_cn = T * ts.fbk.i_cn
        ts.fbk.i_f = T * ts.fbk.i_f
        ts.fbk.v_c = T * ts.fbk.v_c
        ts.fbk.v_cp = T * ts.fbk.v_cp
        ts.fbk.v_cn = T * ts.fbk.v_cn
        ts.fbk.u_gp = T * ts.fbk.u_gp
        ts.fbk.u_gn = T * ts.fbk.u_gn
