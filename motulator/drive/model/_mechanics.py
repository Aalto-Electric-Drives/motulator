"""Continuous-time models for mechanical subsystems."""

from collections.abc import Callable
from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np

from motulator.common.model import Subsystem, SubsystemTimeSeries
from motulator.common.utils._utils import empty_array, get_value


# %%
@dataclass
class MechanicsInputs:
    """Input variables."""

    tau_M: float = 0.0
    tau_L: float | Callable[[float], float] = 0.0


@dataclass
class MechanicsOutputs:
    """Output variables for interconnection."""

    theta_M: float  # Mechanical rotor angle (rad)
    w_M: float


@dataclass
class MechanicalSystemStates:
    """State variables."""

    theta_M: complex = 0j  # Imaginary part always zero
    w_M: complex = 0j


class MechanicalSystem(Subsystem):
    """
    Stiff mechanical system.

    Parameters
    ----------
    J : float
        Total moment of inertia (kgm²).
    B_L : float | Callable[[float], float], optional
        Friction coefficient (Nm/(rad/s)) that can be constant, corresponding to viscous
        friction, or an arbitrary function of the rotor speed. For example, choosing
        ``B_L = lambda w_M: k*abs(w_M)`` gives the quadratic load torque
        ``k*abs(w_M)*w_M``. Defaults to 0.

    """

    angles = frozenset({"theta_M"})

    def __init__(self, J: float, B_L: float | Callable[[float], float] = 0.0) -> None:
        self.inp: MechanicsInputs = MechanicsInputs()
        self.state: MechanicalSystemStates = MechanicalSystemStates()
        self.out: MechanicsOutputs = MechanicsOutputs(
            theta_M=self.state.theta_M.real, w_M=self.state.w_M.real
        )
        self.J = J
        self.B_L = B_L

    def set_external_load_torque(self, tau_L: float | Callable[[float], float]) -> None:
        """Set external load torque (Nm), constant or a function of time."""
        self.inp.tau_L = tau_L

    def _compute_total_load_torque(self, w_M: Any, tau_L: Any) -> Any:
        """Total load torque (Nm), including the friction."""
        return get_value(self.B_L, w_M) * w_M + tau_L

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.theta_M = self.state.theta_M.real
        self.out.w_M = self.state.w_M.real

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        state, inp = self.state, self.inp
        tau_L = get_value(inp.tau_L, t)
        tau_L_tot = self._compute_total_load_torque(state.w_M.real, tau_L)
        d_theta_M = state.w_M
        d_w_M = (inp.tau_M - tau_L_tot) / self.J
        return [d_theta_M, d_w_M]

    def create_time_series(self, t: np.ndarray) -> SubsystemTimeSeries:
        """Create the time series."""
        return MechanicalSystemTimeSeries(t, self)


@dataclass
class MechanicalSystemTimeSeries(SubsystemTimeSeries):
    """Continuous time series for mechanical systems."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[MechanicalSystem]
    # States
    w_M: np.ndarray = field(default_factory=empty_array)
    theta_M: np.ndarray = field(default_factory=empty_array)
    # Inputs
    tau_M: np.ndarray = field(default_factory=empty_array)
    tau_L: np.ndarray = field(default_factory=empty_array)
    # Derived
    tau_L_tot: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: MechanicalSystem) -> None:
        self.w_M = np.real(np.array(subsystem._history["w_M"]))
        theta_M = np.real(np.array(subsystem._history["theta_M"]))
        self.theta_M = np.angle(np.exp(1j * theta_M))
        # External load torque, replaced by the connected one if any
        self.tau_L = np.array([get_value(subsystem.inp.tau_L, t_k) for t_k in t])

    def compute_derived_signals(
        self, t: np.ndarray, subsystem: MechanicalSystem
    ) -> None:
        """Compute the total load torque from the speed and the load torque."""
        self.tau_L_tot = np.array(
            [
                subsystem._compute_total_load_torque(w_M_k, tau_L_k)
                for w_M_k, tau_L_k in zip(self.w_M, self.tau_L, strict=True)
            ]
        )


# %%
@dataclass
class TwoMassMechanicalSystemStates:
    """State variables."""

    theta_M: complex = 0j  # Imaginary part always zero
    w_M: complex = 0j
    w_L: complex = 0j  # Imaginary part always zero
    theta_ML: complex = 0j  # Imaginary part always zero


class TwoMassMechanicalSystem(Subsystem):
    """
    Two-mass mechanical system.

    Parameters
    ----------
    J_M : float
        Motor moment of inertia (kgm²).
    J_L : float
        Load moment of inertia (kgm²).
    K_S : float
        Shaft torsional stiffness (Nm/rad).
    C_S : float
        Shaft torsional damping (Nm/(rad/s)).
    B_L : float | Callable[[float], float], optional
        Friction coefficient (Nm/(rad/s)) that can be constant, corresponding to viscous
        friction, or an arbitrary function of the load speed. For example, choosing
        ``B_L = lambda w_L: k*abs(w_L)`` gives the quadratic load torque
        ``k*abs(w_L)*w_L``. Defaults to 0.

    """

    angles = frozenset({"theta_M"})

    def __init__(
        self,
        J_M: float,
        J_L: float,
        K_S: float,
        C_S: float,
        B_L: float | Callable[[float], float] = 0.0,
    ) -> None:
        self.J_M = J_M
        self.J_L = J_L
        self.K_S = K_S
        self.C_S = C_S
        self.B_L = B_L
        self.state: TwoMassMechanicalSystemStates = TwoMassMechanicalSystemStates()
        self.inp: MechanicsInputs = MechanicsInputs()
        self.out: MechanicsOutputs = MechanicsOutputs(
            theta_M=self.state.theta_M.real, w_M=self.state.w_M.real
        )

    def set_external_load_torque(self, tau_L: float | Callable[[float], float]) -> None:
        """Set external load torque (Nm), constant or a function of time."""
        self.inp.tau_L = tau_L

    def _compute_shaft_torque(self, theta_ML: Any, w_M: Any, w_L: Any) -> Any:
        """Shaft torque (Nm)."""
        return self.K_S * theta_ML + self.C_S * (w_M - w_L)

    def _compute_total_load_torque(self, w_L: Any, tau_L: Any) -> Any:
        """Total load torque (Nm), including the friction."""
        return get_value(self.B_L, w_L) * w_L + tau_L

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.theta_M = self.state.theta_M.real
        self.out.w_M = self.state.w_M.real

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        state, inp = self.state, self.inp
        tau_S = self._compute_shaft_torque(state.theta_ML, state.w_M, state.w_L)
        tau_L = get_value(inp.tau_L, t)
        tau_L_tot = self._compute_total_load_torque(state.w_L.real, tau_L)
        d_theta_M = state.w_M
        d_w_M = (inp.tau_M - tau_S) / self.J_M
        d_w_L = (tau_S - tau_L_tot) / self.J_L
        d_theta_ML = state.w_M - state.w_L
        return [d_theta_M, d_w_M, d_w_L, d_theta_ML]

    def create_time_series(self, t: np.ndarray) -> SubsystemTimeSeries:
        """Create the time series."""
        return TwoMassMechanicalSystemTimeSeries(t, self)


@dataclass
class TwoMassMechanicalSystemTimeSeries(SubsystemTimeSeries):
    """Continuous time series for mechanical systems."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[TwoMassMechanicalSystem]
    # States
    w_M: np.ndarray = field(default_factory=empty_array)
    theta_M: np.ndarray = field(default_factory=empty_array)
    w_L: np.ndarray = field(default_factory=empty_array)
    theta_ML: np.ndarray = field(default_factory=empty_array)
    # Inputs
    tau_M: np.ndarray = field(default_factory=empty_array)
    tau_L: np.ndarray = field(default_factory=empty_array)
    # Derived
    tau_S: np.ndarray = field(default_factory=empty_array)
    tau_L_tot: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: TwoMassMechanicalSystem) -> None:
        self.w_M = np.real(np.array(subsystem._history["w_M"]))
        theta_M = np.real(np.array(subsystem._history["theta_M"]))
        self.theta_M = np.angle(np.exp(1j * theta_M))
        self.w_L = np.real(np.array(subsystem._history["w_L"]))
        self.theta_ML = np.real(np.array(subsystem._history["theta_ML"]))
        self.tau_S = subsystem._compute_shaft_torque(self.theta_ML, self.w_M, self.w_L)
        # External load torque, replaced by the connected one if any
        self.tau_L = np.array([get_value(subsystem.inp.tau_L, t_k) for t_k in t])

    def compute_derived_signals(
        self, t: np.ndarray, subsystem: TwoMassMechanicalSystem
    ) -> None:
        """Compute the total load torque from the load speed and the load torque."""
        self.tau_L_tot = np.array(
            [
                subsystem._compute_total_load_torque(w_L_k, tau_L_k)
                for w_L_k, tau_L_k in zip(self.w_L, self.tau_L, strict=True)
            ]
        )


# %%
@dataclass
class ExternalRotorSpeedStates:
    """State variables."""

    theta_M: complex = 0j  # Imaginary part always zero


class ExternalRotorSpeed(Subsystem):
    """
    Externally given rotor speed.

    The speed is given externally instead of being determined by the torque dynamics,
    and the rotor angle is integrated from the speed. The electromagnetic torque is an
    input, as in the other mechanical systems, but it does not affect the speed.

    Parameters
    ----------
    w_M : float | Callable[[float], float], optional
        Rotor speed (rad/s), constant or a function of time, defaults to 0. It can be
        changed later through the attribute `w_M`.

    """

    angles = frozenset({"theta_M"})

    def __init__(self, w_M: float | Callable[[float], float] = 0.0) -> None:
        self.inp: MechanicsInputs = MechanicsInputs()
        self.state: ExternalRotorSpeedStates = ExternalRotorSpeedStates()
        self.out: MechanicsOutputs = MechanicsOutputs(theta_M=0.0, w_M=0.0)
        self.w_M = w_M
        self.set_outputs(0.0)

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        # External rotor speed is fed directly to the output
        self.out.w_M = get_value(self.w_M, t)
        self.out.theta_M = self.state.theta_M.real

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        return [get_value(self.w_M, t)]

    def create_time_series(self, t: np.ndarray) -> SubsystemTimeSeries:
        """Create the time series."""
        return ExternalRotorSpeedTimeSeries(t, self)


@dataclass
class ExternalRotorSpeedTimeSeries(SubsystemTimeSeries):
    """Continuous time series for mechanical systems."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[ExternalRotorSpeed]
    # State
    theta_M: np.ndarray = field(default_factory=empty_array)
    # External rotor speed
    w_M: np.ndarray = field(default_factory=empty_array)
    # Input
    tau_M: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: ExternalRotorSpeed) -> None:
        self.w_M = np.array([get_value(subsystem.w_M, t_k) for t_k in t])
        theta_M = np.real(np.array(subsystem._history["theta_M"]))
        self.theta_M = np.angle(np.exp(1j * theta_M))
