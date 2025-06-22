"""Continuous-time models for mechanical subsystems."""

from cmath import phase
from dataclasses import InitVar, dataclass, field
from typing import Any, Callable

import numpy as np

from motulator.common.model import Subsystem, SubsystemTimeSeries
from motulator.common.utils._utils import empty_array, get_value


# %%
@dataclass
class Inputs:
    """Input variables."""

    tau_M: float | None = None
    tau_L: Callable[[float], float] = lambda t: 0.0 * t


@dataclass
class Outputs:
    """Output variables for interconnection."""

    exp_j_theta_M: complex
    w_M: float


@dataclass
class States:
    """State variables for mechanical systems."""

    exp_j_theta_M: complex = complex(1)
    w_M: complex = 0j


@dataclass
class StateHistory:
    """State history."""

    exp_j_theta_M: list[complex] = field(default_factory=list)
    w_M: list[complex] = field(default_factory=list)


class MechanicalSystem(Subsystem):
    """
    Stiff mechanical system.

    Parameters
    ----------
    J : float
        Total moment of inertia (kgm²).
    B_L : float | Callable[[float], float]
        Friction coefficient (Nm/(rad/s)) that can be constant, corresponding to viscous
        friction, or an arbitrary function of the rotor speed. For example, choosing
        ``B_L = lambda w_M: k*abs(w_M)`` gives the quadratic load torque ``k*w_M**2``.
        The default is ``B_L = 0``.

    """

    def __init__(self, J: float, B_L: float | Callable[[float], float] = 0.0) -> None:
        self.inp: Inputs = Inputs()
        self.state: States = States()
        self.out: Outputs = Outputs(
            exp_j_theta_M=self.state.exp_j_theta_M, w_M=self.state.w_M.real
        )
        self._history: StateHistory = StateHistory()
        self.J = J
        self.B_L = B_L

    def set_external_load_torque(self, tau_L: Callable[[float], float]) -> None:
        """Set external load torque (Nm)."""
        self.inp.tau_L = tau_L

    def set_external_rotor_speed(self, _) -> None:
        """Set external rotor speed (rad/s)."""
        raise NotImplementedError

    def compute_total_load_torque(self, t: Any, state: Any) -> Any:
        """Total load torque (Nm)."""
        B_L = get_value(self.B_L, state.w_M)
        tau_L_tot = B_L * state.w_M.real + self.inp.tau_L(t)
        return tau_L_tot

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.exp_j_theta_M = self.state.exp_j_theta_M
        self.out.w_M = self.state.w_M.real

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        state, inp = self.state, self.inp
        tau_L_tot = self.compute_total_load_torque(t, state)
        d_exp_j_theta_M = 1j * state.w_M * state.exp_j_theta_M
        d_w_M = (inp.tau_M - tau_L_tot) / self.J
        return [d_exp_j_theta_M, d_w_M]

    def meas_position(self) -> float:
        """Measure mechanical rotor angle (rad)."""
        return phase(self.out.exp_j_theta_M)

    def meas_speed(self) -> float:
        """Measure mechanical rotor speed (rad/s)."""
        return self.out.w_M

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "MechanicalSystemTimeSeries"]:
        """Create time series from state list."""
        return "mechanics", MechanicalSystemTimeSeries(t, self)


@dataclass
class MechanicalSystemTimeSeries(SubsystemTimeSeries):
    """Continuous time series for mechanical systems."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[MechanicalSystem]
    # States
    w_M: np.ndarray = field(default_factory=empty_array)
    exp_j_theta_M: np.ndarray = field(default_factory=empty_array)
    # Outputs
    tau_L_tot: np.ndarray = field(default_factory=empty_array)
    # Derived
    theta_M: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: MechanicalSystem) -> None:
        self.w_M = np.real(np.array(subsystem._history.w_M))
        self.exp_j_theta_M = np.array(subsystem._history.exp_j_theta_M)
        self.tau_L_tot = subsystem.compute_total_load_torque(t, self)
        self.theta_M = np.angle(self.exp_j_theta_M)


# %%
@dataclass
class TwoMassMechanicalSystemStates:
    """State variables."""

    exp_j_theta_M: complex = complex(1)
    w_M: complex = 0j
    w_L: complex = 0j  # Imaginary part always zero
    theta_ML: complex = 0j  # Imaginary part always zero


@dataclass
class TwoMassMechanicalSystemStateHistory:
    """Temporary storage for system states."""

    exp_j_theta_M: list[complex] = field(default_factory=list)
    w_M: list[complex] = field(default_factory=list)
    w_L: list[complex] = field(default_factory=list)
    theta_ML: list[complex] = field(default_factory=list)


class TwoMassMechanicalSystem(Subsystem):
    """
    Two-mass mechanical subsystem.

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
    B_L : float | Callable[[float], float]
        Friction coefficient (Nm/(rad/s)) that can be constant, corresponding to viscous
        friction, or an arbitrary function of the load speed. For example, choosing
        ``B_L = lambda w_L: k*abs(w_L)`` leads to quadratic load torque ``k*w_L**2``.
        The default is ``B_L = 0``.

    """

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
        self.inp: Inputs = Inputs()
        self.out: Outputs = Outputs(
            exp_j_theta_M=self.state.exp_j_theta_M, w_M=self.state.w_M.real
        )
        self._history: TwoMassMechanicalSystemStateHistory = (
            TwoMassMechanicalSystemStateHistory()
        )

    def set_external_load_torque(self, tau_L: Callable[[float], float]) -> None:
        """Set external load torque (Nm)."""
        self.inp.tau_L = tau_L

    def set_external_rotor_speed(self, _) -> None:
        """Set external rotor speed (rad/s)."""
        raise NotImplementedError

    def compute_torques(self, t: Any, state: Any) -> tuple[Any, Any]:
        """Compute shaft and load torques (Nm)."""
        B_L = get_value(self.B_L, state.w_L)
        tau_S = self.K_S * state.theta_ML + self.C_S * (state.w_M - state.w_L)
        tau_L_tot = B_L * state.w_L + self.inp.tau_L(t)
        return tau_S, tau_L_tot

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        self.out.exp_j_theta_M = self.state.exp_j_theta_M
        self.out.w_M = self.state.w_M.real

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        state, inp = self.state, self.inp
        tau_S, tau_L_tot = self.compute_torques(t, state)
        d_exp_j_theta_M = 1j * state.w_M * state.exp_j_theta_M
        d_w_M = (inp.tau_M - tau_S) / self.J_M
        d_w_L = (tau_S - tau_L_tot) / self.J_L
        d_theta_ML = state.w_M - state.w_L
        return [d_exp_j_theta_M, d_w_M, d_w_L, d_theta_ML]

    def meas_position(self) -> float:
        """Measure mechanical rotor angle (rad)."""
        return phase(self.out.exp_j_theta_M)

    def meas_speed(self) -> float:
        """Measure mechanical rotor speed (rad/s)."""
        return self.out.w_M

    def meas_load_speed(self) -> float:
        """Measure the load speed."""
        return self.state.w_L.real

    def meas_load_position(self) -> float:
        """Measure the load angle."""
        theta_L = phase(self.state.exp_j_theta_M) - self.state.theta_ML.real
        return theta_L

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "TwoMassMechanicalSystemTimeSeries"]:
        """Create time series from state list."""
        return "mechanics", TwoMassMechanicalSystemTimeSeries(t, self)


@dataclass
class TwoMassMechanicalSystemTimeSeries(SubsystemTimeSeries):
    """Continuous time series for mechanical systems."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[TwoMassMechanicalSystem]
    # States
    w_M: np.ndarray = field(default_factory=empty_array)
    exp_j_theta_M: np.ndarray = field(default_factory=empty_array)
    w_L: np.ndarray = field(default_factory=empty_array)
    theta_ML: np.ndarray = field(default_factory=empty_array)
    # Outputs
    tau_S: np.ndarray = field(default_factory=empty_array)
    tau_L_tot: np.ndarray = field(default_factory=empty_array)
    # Derived
    theta_M: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: TwoMassMechanicalSystem) -> None:
        self.w_M = np.real(np.array(subsystem._history.w_M))
        self.exp_j_theta_M = np.array(subsystem._history.exp_j_theta_M)
        self.w_L = np.real(np.array(subsystem._history.w_L))
        self.theta_ML = np.real(np.array(subsystem._history.theta_ML))
        self.tau_S, self.tau_L_tot = subsystem.compute_torques(t, self)
        self.theta_M = np.angle(self.exp_j_theta_M)


# %%
@dataclass
class ExternalRotorSpeedStates:
    """State variables."""

    exp_j_theta_M: complex = complex(1)


@dataclass
class ExternalRotorSpeedStateHistory:
    """State history."""

    exp_j_theta_M: list[complex] = field(default_factory=list)


class ExternalRotorSpeed(Subsystem):
    """
    Integrate rotor angle from externally given rotor speed.

    This class maintains the same interface as other mechanical systems but the speed is
    determined by an external function rather than by torque dynamics.

    """

    def __init__(self) -> None:
        self.inp: Inputs = Inputs()
        self.state: ExternalRotorSpeedStates = ExternalRotorSpeedStates()
        self.out: Outputs = Outputs(exp_j_theta_M=self.state.exp_j_theta_M, w_M=0.0)
        self._history: ExternalRotorSpeedStateHistory = ExternalRotorSpeedStateHistory()
        self.w_M = lambda t: 0.0  # External input

    def set_external_rotor_speed(self, w_M: Callable[[float], float]) -> None:
        """Set external rotor speed (rad/s)."""
        self.w_M = w_M

    def set_external_load_torque(self, tau_L: Callable[[float], float]) -> None:
        """Set external load torque (Nm)."""
        raise NotImplementedError

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        # External rotor speed is fed directly to the output
        self.out.w_M = self.w_M(t)
        self.out.exp_j_theta_M = self.state.exp_j_theta_M

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        d_exp_j_theta_M = 1j * self.out.w_M * self.state.exp_j_theta_M
        return [d_exp_j_theta_M]

    def meas_position(self) -> float:
        """Measure mechanical rotor angle (rad)."""
        return phase(self.out.exp_j_theta_M)

    def meas_speed(self) -> float:
        """Measure mechanical rotor speed (rad/s)."""
        return self.out.w_M

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "ExternalRotorSpeedTimeSeries"]:
        """Create time series from state list."""
        return "mechanics", ExternalRotorSpeedTimeSeries(t, self)


@dataclass
class ExternalRotorSpeedTimeSeries(SubsystemTimeSeries):
    """Continuous time series for mechanical systems."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[ExternalRotorSpeed]
    # State
    exp_j_theta_M: np.ndarray = field(default_factory=empty_array)
    # Input
    w_M: np.ndarray = field(default_factory=empty_array)
    # Derived
    theta_M: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: ExternalRotorSpeed) -> None:
        self.w_M = np.array([subsystem.w_M(t) for t in t])
        self.exp_j_theta_M = np.array(subsystem._history.exp_j_theta_M)
        self.theta_M = np.angle(self.exp_j_theta_M)
