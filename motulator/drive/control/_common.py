"""Common control functions and classes for machine drives."""

from cmath import exp
from collections.abc import Sequence
from dataclasses import dataclass
from math import expm1, inf, pi
from typing import Protocol

import numpy as np

from motulator.common.control._controllers import PIController
from motulator.drive.utils._parameters import (
    InductionMachineInvGammaPars,
    InductionMachinePars,
    SaturatedSynchronousMachinePars,
    SynchronousMachinePars,
)


# %%
@dataclass
class References:
    """Reference signals of the drive controllers."""

    T_s: float = 0.0  # Sampling period for the next control step
    tau_M: float = 0.0  # Torque reference
    psi_s: float = 0.0  # Stator flux magnitude reference (not used in IM CVC)
    i_s: complex = 0j  # Stator current reference (only used in CVC)
    u_s: complex = 0j  # Stator voltage reference
    u_s_lim: complex = 0j  # Limited voltage reference (set by the control system)
    w_M: float | None = None  # Speed reference (set by the control system)
    d_abc: Sequence[float] = (0.0, 0.0, 0.0)  # Duty ratios (set by the control system)
    enabled: bool = True  # Effective gate enable, including the model interlock
    flying_start: bool = False  # PWM priming or torque takeover ramp is in progress


# %%
class FluxTorqueFeedbacks(Protocol):
    """Protocol defining the feedback signals of the flux and torque controller."""

    i_s: complex  # Stator current
    u_s: complex  # Stator voltage
    psi_s: complex  # Stator flux linkage estimate
    tau_M: float  # Electromagnetic torque estimate


@dataclass
class Gains:
    """Control gains of the flux and torque controller."""

    alpha_psi: float
    alpha_tau: float
    alpha_i: float


class FluxTorqueControllerBase:
    """
    Base class of the flux and torque controllers.

    The stator flux magnitude and the electromagnetic torque are controlled in the
    directions that decouple the two channels. The directions depend on the auxiliary
    current, which the machine-specific subclasses compute in `compute_output`.

    Parameters
    ----------
    par : SynchronousMachinePars | SaturatedSynchronousMachinePars | \
        InductionMachineInvGammaPars | InductionMachinePars
        Machine model parameters.
    alpha_psi : float
        Flux-control bandwidth (rad/s).
    alpha_tau : float
        Torque-control bandwidth (rad/s).
    alpha_i : float, optional
        Integral action bandwidth (rad/s), defaults to 0.

    """

    def __init__(
        self,
        par: SynchronousMachinePars
        | SaturatedSynchronousMachinePars
        | InductionMachineInvGammaPars
        | InductionMachinePars,
        alpha_psi: float,
        alpha_tau: float,
        alpha_i: float = 0.0,
    ) -> None:
        self.par = par
        self.gain = Gains(alpha_psi, alpha_tau, alpha_i)
        # Integral states
        self.x_psi: float = 0.0
        self.x_tau: float = 0.0
        # Workspace variables for the update method
        self._i_a: complex = 0j
        self._u_ff: complex = 0j

    def initialize(self, fbk: FluxTorqueFeedbacks) -> None:
        """Initialize integral action at the measured flux and torque."""
        self.x_psi = self.gain.alpha_i * abs(fbk.psi_s)
        self.x_tau = self.gain.alpha_i * fbk.tau_M

    def _compute_voltage_ref(
        self,
        psi_s_ref: float,
        tau_M_ref: float,
        fbk: FluxTorqueFeedbacks,
        i_a: complex,
        w: float,
    ) -> complex:
        """
        Compute the voltage reference.

        Parameters
        ----------
        psi_s_ref : float
            Stator flux magnitude reference (Vs).
        tau_M_ref : float
            Torque reference (Nm).
        fbk : FluxTorqueFeedbacks
            Feedback signals.
        i_a : complex
            Auxiliary current (A). If zero, only the flux is controlled.
        w : float
            Angular speed (rad/s) of the back-emf term ``1j*w*psi_s``.

        Returns
        -------
        complex
            Voltage reference (V).

        """
        par, gain = self.par, self.gain

        # Torque-production factor and directions
        c_tau = 1.5 * par.n_p * (i_a * fbk.psi_s.conjugate()).real
        t_psi = 1.5 * par.n_p * abs(fbk.psi_s) * i_a / c_tau if c_tau > 0 else 1.0
        t_tau = 1j * fbk.psi_s / c_tau if c_tau > 0 else 0j

        # Error signals
        e_psi = psi_s_ref - abs(fbk.psi_s)
        e_tau = tau_M_ref - fbk.tau_M
        e_u = gain.alpha_psi * e_psi * t_psi + gain.alpha_tau * e_tau * t_tau
        u_i = self.x_psi * t_psi + self.x_tau * t_tau
        e_v = u_i - gain.alpha_i * (abs(fbk.psi_s) * t_psi + fbk.tau_M * t_tau)

        # Voltage reference, consisting of the feedforward and integral action (u_ff)
        # and the proportional action (e_u)
        u_ff = par.R_s * fbk.i_s + 1j * w * fbk.psi_s + e_v
        self._i_a, self._u_ff = i_a, u_ff
        return u_ff + e_u

    def update(self, T_s: float, fbk: FluxTorqueFeedbacks) -> None:
        """Update the integral states."""
        par, gain = self.par, self.gain
        # Error signal and gains
        u_err = fbk.u_s - self._u_ff
        k_psi = gain.alpha_i / abs(fbk.psi_s) if abs(fbk.psi_s) > 0 else 0.0
        k_tau = 1.5 * par.n_p * gain.alpha_i
        # Update the integral states
        self.x_psi += T_s * k_psi * (fbk.psi_s * u_err.conjugate()).real
        self.x_tau += T_s * k_tau * (1j * self._i_a * u_err.conjugate()).real


# %%
class SpeedController(PIController):
    """
    2DOF PI speed controller.

    This is an interface for a speed controller. The gains are initialized based on the
    desired closed-loop bandwidth and the rotor inertia estimate.

    Parameters
    ----------
    J : float
        Total inertia of the rotor (kgm²).
    alpha_s : float
        Reference-tracking bandwidth (rad/s).
    alpha_i : float, optional
        Integral action bandwidth (rad/s), defaults to `alpha_s`.
    tau_M_max : float, optional
        Maximum motor torque (Nm), defaults to `inf`.

    """

    def __init__(
        self,
        J: float,
        alpha_s: float,
        alpha_i: float | None = None,
        tau_M_max: float = inf,
    ) -> None:
        alpha_i = alpha_s if alpha_i is None else alpha_i
        k_p = (alpha_s + alpha_i) * J
        k_i = alpha_s * alpha_i * J
        k_t = alpha_s * J
        super().__init__(k_p, k_i, k_t, tau_M_max)


# %%
class SpeedObserver:
    """
    Speed observer.

    This observer estimates the mechanical rotor speed based on the mechanical system
    model and the error signal. If the inertia of the mechanical system is provided, the
    load torque is also estimated, avoiding lag in the speed estimate [#Lor1991]_.

    Parameters
    ----------
    k_w : float
        Speed-estimation gain (rad/s or (rad/s)² for induction machines or for
        synchronous machines, respectively).
    k_tau : float
        Load-torque estimation gain (Nm or Nm/s for induction machines or for
        synchronous machines, respectively).
    J : float | None
        Inertia of the mechanical system (kgm²). If None, the load torque estimation is
        not used.

    References
    ----------
    .. [#Lor1991] Lorenz, Van Patten, "High-resolution velocity estimation for
       all-digital, AC servo drives," IEEE Trans. Ind. Appl., 1991,
       https://doi.org/10.1109/28.85485

    """

    def __init__(self, k_w: float, k_tau: float, J: float | None) -> None:
        self.k_w = k_w
        self.k_tau = k_tau
        self.J = J
        self.w_M: float = 0.0
        self.tau_L: float = 0.0

    def compute_output(self) -> tuple[float, float]:
        """Compute outputs."""
        return self.w_M, self.tau_L

    def update(self, T_s: float, eps: float, tau_M: float = 0.0) -> None:
        """
        Update mechanical state estimates.

        Parameters
        ----------
        T_s : float
            Sample time (s).
        eps : float
            Estimation error signal: mechanical speed (rad/s) or mechanical position
            (rad) for induction machines or synchronous machines, respectively.
        tau_M : float, optional
            Electromagnetic torque estimate (Nm).

        """
        if self.J is None:
            d_w_M = self.k_w * eps
            d_tau_L = 0.0
        else:
            d_w_M = (tau_M - self.tau_L) / self.J + self.k_w * eps
            d_tau_L = -self.k_tau * eps

        self.w_M += T_s * d_w_M
        self.tau_L += T_s * d_tau_L


# %%
def hold_equivalent(T_s: float, w_c: float) -> tuple[complex, complex]:
    """
    Hold-equivalent discretization of a vector integrator in rotating coordinates.

    The coefficients discretize ``d(psi)/dt = u + v - 1j*w_c*psi``, where `psi` is in
    coordinates rotating at `w_c`, assuming that `u` is constant in stator coordinates
    and `v` is constant in the rotating coordinates over the sampling period. Then,
    ``psi(k+1) = Phi*(psi(k) + T_s*u(k)) + Gamma*v(k)``, where `u(k)` and `v(k)` are in
    the coordinates at the instant `k`.

    Parameters
    ----------
    T_s : float
        Sampling period (s).
    w_c : float
        Angular speed of the coordinate system (rad/s).

    Returns
    -------
    Phi : complex
        Rotation over the sampling period, ``exp(-1j*w_c*T_s)``.
    Gamma : complex
        Input gain of the rotating-coordinate input, ``(1 - Phi)/(1j*w_c)``.

    """
    # Half-angle form, requiring a single sine-cosine pair
    x = 0.5 * w_c * T_s
    exp_jx = exp(-1j * x)
    return exp_jx * exp_jx, T_s * float(np.sinc(x / pi)) * exp_jx


def matched_pole(alpha: float, T_s: float) -> float:
    """
    Pole for the gain formulas of estimators updated by the forward Euler method.

    The forward Euler method maps the continuous-time pole `s` to the discrete-time
    pole ``1 + s*T_s``. Using the returned value instead of `alpha` in the gain
    formulas of a design with the poles at ``s = -alpha`` places the discrete-time
    poles at ``exp(-alpha*T_s)``. This holds for the constant sampling period `T_s`.

    Parameters
    ----------
    alpha : float
        Continuous-time pole (rad/s).
    T_s : float
        Sampling period (s).

    Returns
    -------
    float
        Matched pole (rad/s), ``(1 - exp(-alpha*T_s))/T_s``.

    """
    return -expm1(-alpha * T_s) / T_s
