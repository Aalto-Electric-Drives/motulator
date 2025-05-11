"""Disturbance-observer-based grid-forming control."""

from cmath import exp
from dataclasses import dataclass, field
from math import pi, sqrt
from typing import Sequence

import numpy as np

from motulator.common.control._base import TimeSeries
from motulator.common.control._pwm import PWM
from motulator.common.utils import wrap
from motulator.grid.control._base import Measurements
from motulator.grid.control._controllers import CurrentLimiter


# %%
@dataclass
class ObserverStates:
    """State estimates."""

    theta_c: float = 0.0
    u_gp: complex = 0j


@dataclass
class ObserverOutputs:
    """Feedback signals for the control system."""

    u_dc: float = 0.0
    i_c: complex = 0j
    u_c: complex = 0j
    v_c: complex = 0j
    u_g: complex = 0j
    theta_c: float = 0.0
    p_g: float = 0.0
    q_g: float = 0.0


class Observer:
    """
    Disturbance observer.

    This implements a disturbance observer, which estimates the quasi-static converter
    output voltage. Coordinates rotating at the nominal grid angular frequency are used.

    Parameters
    ----------
    u_nom : float
        Nominal grid voltage (V), line-to-neutral peak value.
    w_nom : float
        Nominal grid angular frequency (rad/s).
    alpha_o : float
        Observer gain (rad/s).
    L : float
        Total inductance (H).
    R : float, optional
        Total series resistance (Ω), defaults to 0.

    """

    def __init__(
        self, u_nom: float, w_nom: float, alpha_o: float, L: float, R: float = 0
    ) -> None:
        self.alpha_o: float = alpha_o
        self.L: float = L
        self.R: float = R
        self.w_g: float = w_nom
        self.state = ObserverStates(theta_c=0, u_gp=u_nom)

    def compute_output(self, meas: Measurements, u_c_ab: complex) -> ObserverOutputs:
        """Compute the estimates."""
        out = ObserverOutputs(theta_c=self.state.theta_c)

        # Transform the measured values into synchronous coordinates
        out.i_c = exp(-1j * out.theta_c) * meas.i_c_ab
        out.u_c = exp(-1j * out.theta_c) * u_c_ab

        # Estimates for the quasi-static converter voltage and the grid voltage
        u_gp = self.state.u_gp
        out.v_c = u_gp - (self.alpha_o - 1j * self.w_g) * self.L * out.i_c
        out.u_g = u_gp - self.alpha_o * self.L * out.i_c

        # Active and reactive powers
        s_g = 1.5 * out.u_g * out.i_c.conjugate()
        out.p_g, out.q_g = s_g.real, s_g.imag

        return out

    def update(self, T_s: float, out: ObserverOutputs) -> None:
        """Update the states."""
        self.state.u_gp += T_s * self.alpha_o * (out.u_c - out.v_c - self.R * out.i_c)
        self.state.theta_c += T_s * self.w_g
        self.state.theta_c = wrap(self.state.theta_c)


# %%
@dataclass
class References:
    """Reference signals for observer-based grid-forming control."""

    T_s: float = 0.0
    d_abc: Sequence[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    p_g: float = 0.0
    u_c: complex = 0j
    i_c: complex = 0j
    v_c: float = 0.0
    u_dc: float | None = None


class ObserverBasedGridFormingController:
    """
    Disturbance-observer-based grid-forming controller.

    This implements the RFPSC-type grid-forming mode of the control method described in
    [#Nur2024]_. Transparent current control is also implemented.

    Parameters
    ----------
    i_max : float
        Maximum current (A), peak value.
    L : float
        Total inductance (H).
    R : float, optional
        Total series resistance (Ω), defaults to 0.
    R_a : float, optional
        Active resistance (Ω), defaults to `0.25*u_nom/i_max`.
    k_v : float, optional
        Voltage control gain, defaults to `alpha_o/w_nom`.
    alpha_o : float, optional
        Observer gain (rad/s), defaults to 2*pi*50.
    alpha_c : float, optional
        Current control bandwidth (rad/s), defaults to 2*pi*400.
    u_nom : float, optional
        Nominal grid voltage (V), line-to-neutral peak value, defaults to
        `sqrt(2/3)*400`.
    T_s : float, optional
        Sampling period (s), defaults to 125e-6.

    Notes
    -----
    In this implementation, the control system operates in synchronous coordinates
    rotating at the nominal grid angular frequency. For other implementation options,
    see [#Nur2024]_.

    References
    ----------
    .. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming
       converter control based on a disturbance observer," IEEE Trans. Power Electron.,
       2024, https://doi.org/10.1109/TPEL.2024.3433503

    """

    def __init__(
        self,
        i_max: float,
        L: float,
        R: float = 0,
        R_a: float | None = None,
        k_v: float | None = None,
        alpha_o: float = 2 * pi * 50,
        alpha_c: float = 2 * pi * 400,
        u_nom: float = sqrt(2 / 3) * 400,
        w_nom: float = 2 * pi * 50,
        T_s: float = 125e-6,
    ) -> None:
        self.pwm = PWM()
        self.observer = Observer(u_nom=u_nom, w_nom=w_nom, L=L, R=R, alpha_o=alpha_o)
        self.current_limiter = CurrentLimiter(i_max)
        # Initialize gains
        self.R_a = 0.25 * u_nom / i_max if R_a is None else R_a
        self.k_v = alpha_o / w_nom if k_v is None else k_v
        self.k_c = alpha_c * L  # Current control gain
        self.T_s: float = T_s

    def get_feedback(self, meas: Measurements) -> ObserverOutputs:
        """Get the feedback signals."""
        u_c_ab = self.pwm.get_realized_voltage()
        fbk = self.observer.compute_output(meas, u_c_ab)
        fbk.u_dc = meas.u_dc
        return fbk

    def compute_output(
        self, p_g_ref: float, v_c_ref: float, fbk: ObserverOutputs
    ) -> References:
        """Compute references."""
        ref = References(T_s=self.T_s, p_g=p_g_ref, v_c=v_c_ref)

        # Complex gains for grid-forming mode
        exp_j_theta = fbk.v_c / abs(fbk.v_c) if abs(fbk.v_c) > 0 else 1
        k_p = exp_j_theta * self.R_a / (1.5 * ref.v_c)
        k_v = exp_j_theta * (1 - 1j * self.k_v)

        # Feedback correction for grid-forming mode
        e_c = k_p * (ref.p_g - fbk.p_g) + k_v * (ref.v_c - abs(fbk.v_c))

        # Transparent current limitation
        ref.i_c = fbk.i_c + e_c / self.k_c
        ref.i_c = self.current_limiter(ref.i_c)
        e_c = self.k_c * (ref.i_c - fbk.i_c)

        # Voltage reference
        ref.u_c = e_c + fbk.v_c + self.observer.R * fbk.i_c
        u_c_ref_ab = exp(1j * fbk.theta_c) * ref.u_c
        ref.d_abc = self.pwm(ref.T_s, u_c_ref_ab, fbk.u_dc, self.observer.w_g)

        return ref

    def update(self, ref: References, fbk: ObserverOutputs) -> None:
        """Update states."""
        self.observer.update(ref.T_s, fbk)

    def post_process(self, ts: TimeSeries) -> None:
        """Post-process controller time series."""
        # Convert quantities to converter-output-voltage coordinates
        T = np.where(
            np.abs(ts.fbk.v_c) > 0, np.conj(ts.fbk.v_c) / np.abs(ts.fbk.v_c), 1
        )
        ts.ref.u_c = T * ts.ref.u_c
        ts.fbk.i_c = T * ts.fbk.i_c
        ts.ref.i_c = T * ts.ref.i_c
