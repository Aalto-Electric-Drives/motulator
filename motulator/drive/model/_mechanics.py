"""Continuous-time models for mechanical subsystems."""

from collections.abc import Callable
from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np

from motulator.common.model import Subsystem, SubsystemTimeSeries
from motulator.common.utils._utils import empty_array, get_value, wrap


# %%
@dataclass
class Inputs:
    """Input variables."""

    tau_M: float = 0.0
    tau_L: float | Callable[[float], float] = 0.0


@dataclass
class Outputs:
    """Output variables for interconnection."""

    theta_M: float  # Mechanical rotor angle (rad)
    w_M: float


@dataclass
class States:
    """State variables for mechanical systems."""

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
        ``B_L = lambda w_M: k*abs(w_M)`` gives the quadratic load torque ``k*w_M**2``.
        Defaults to 0.

    """

    def __init__(self, J: float, B_L: float | Callable[[float], float] = 0.0) -> None:
        self.inp: Inputs = Inputs()
        self.state: States = States()
        self.out: Outputs = Outputs(
            theta_M=self.state.theta_M.real, w_M=self.state.w_M.real
        )
        self.J = J
        self.B_L = B_L

    def set_external_load_torque(self, tau_L: float | Callable[[float], float]) -> None:
        """Set external load torque (Nm), constant or a function of time."""
        self.inp.tau_L = tau_L

    def set_external_rotor_speed(self, _) -> None:
        """Set external rotor speed (rad/s), constant or a function of time."""
        raise NotImplementedError

    def _compute_total_load_torque(self, w_M: Any, tau_L: Any) -> Any:
        """Total load torque (Nm), including the friction."""
        return get_value(self.B_L, w_M) * w_M + tau_L

    def set_states(self, state_list: list[complex], index: int) -> int:
        """Set states, keeping the rotor angle within [-pi, pi)."""
        index = super().set_states(state_list, index)
        self.state.theta_M = wrap(self.state.theta_M.real)
        return index

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

    def meas_position(self) -> float:
        """Measure mechanical rotor angle (rad)."""
        return wrap(self.out.theta_M)

    def meas_speed(self) -> float:
        """Measure mechanical rotor speed (rad/s)."""
        return self.out.w_M

    def create_time_series(self, t: np.ndarray) -> "MechanicalSystemTimeSeries":
        """Create time series from state list."""
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
    exp_j_theta_M: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: MechanicalSystem) -> None:
        self.w_M = np.real(np.array(subsystem._history["w_M"]))
        theta_M = np.real(np.array(subsystem._history["theta_M"]))
        self.exp_j_theta_M = np.exp(1j * theta_M)
        self.theta_M = np.angle(self.exp_j_theta_M)
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
    B_L : float | Callable[[float], float], optional
        Friction coefficient (Nm/(rad/s)) that can be constant, corresponding to viscous
        friction, or an arbitrary function of the load speed. For example, choosing
        ``B_L = lambda w_L: k*abs(w_L)`` leads to quadratic load torque ``k*w_L**2``.
        Defaults to 0.

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
            theta_M=self.state.theta_M.real, w_M=self.state.w_M.real
        )

    def set_external_load_torque(self, tau_L: float | Callable[[float], float]) -> None:
        """Set external load torque (Nm), constant or a function of time."""
        self.inp.tau_L = tau_L

    def set_external_rotor_speed(self, _) -> None:
        """Set external rotor speed (rad/s), constant or a function of time."""
        raise NotImplementedError

    def _compute_shaft_torque(self, theta_ML: Any, w_M: Any, w_L: Any) -> Any:
        """Shaft torque (Nm)."""
        return self.K_S * theta_ML + self.C_S * (w_M - w_L)

    def _compute_total_load_torque(self, w_L: Any, tau_L: Any) -> Any:
        """Total load torque (Nm), including the friction."""
        return get_value(self.B_L, w_L) * w_L + tau_L

    def set_states(self, state_list: list[complex], index: int) -> int:
        """Set states, keeping the rotor angle within [-pi, pi)."""
        index = super().set_states(state_list, index)
        self.state.theta_M = wrap(self.state.theta_M.real)
        return index

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

    def meas_position(self) -> float:
        """Measure mechanical rotor angle (rad)."""
        return wrap(self.out.theta_M)

    def meas_speed(self) -> float:
        """Measure mechanical rotor speed (rad/s)."""
        return self.out.w_M

    def meas_load_speed(self) -> float:
        """Measure the load speed."""
        return self.state.w_L.real

    def meas_load_position(self) -> float:
        """Measure the load angle."""
        return wrap(self.out.theta_M - self.state.theta_ML.real)

    def create_time_series(self, t: np.ndarray) -> "TwoMassMechanicalSystemTimeSeries":
        """Create time series from state list."""
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
    exp_j_theta_M: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: TwoMassMechanicalSystem) -> None:
        self.w_M = np.real(np.array(subsystem._history["w_M"]))
        theta_M = np.real(np.array(subsystem._history["theta_M"]))
        self.exp_j_theta_M = np.exp(1j * theta_M)
        self.theta_M = np.angle(self.exp_j_theta_M)
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
    Integrate rotor angle from externally given rotor speed.

    This class maintains the same interface as other mechanical systems but the speed is
    determined by an external function rather than by torque dynamics.

    """

    def __init__(self) -> None:
        self.inp: Inputs = Inputs()
        self.state: ExternalRotorSpeedStates = ExternalRotorSpeedStates()
        self.out: Outputs = Outputs(theta_M=self.state.theta_M.real, w_M=0.0)
        self.w_M: float | Callable[[float], float] = 0.0  # External input

    def set_external_rotor_speed(self, w_M: float | Callable[[float], float]) -> None:
        """Set external rotor speed (rad/s), constant or a function of time."""
        self.w_M = w_M

    def set_external_load_torque(self, tau_L: float | Callable[[float], float]) -> None:
        """Set external load torque (Nm), constant or a function of time."""
        raise NotImplementedError

    def set_states(self, state_list: list[complex], index: int) -> int:
        """Set states, keeping the rotor angle within [-pi, pi)."""
        index = super().set_states(state_list, index)
        self.state.theta_M = wrap(self.state.theta_M.real)
        return index

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        # External rotor speed is fed directly to the output
        self.out.w_M = get_value(self.w_M, t)
        self.out.theta_M = self.state.theta_M.real

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        d_theta_M = self.out.w_M
        return [d_theta_M]

    def meas_position(self) -> float:
        """Measure mechanical rotor angle (rad)."""
        return wrap(self.out.theta_M)

    def meas_speed(self) -> float:
        """Measure mechanical rotor speed (rad/s)."""
        return self.out.w_M

    def create_time_series(self, t: np.ndarray) -> "ExternalRotorSpeedTimeSeries":
        """Create time series from state list."""
        return ExternalRotorSpeedTimeSeries(t, self)


@dataclass
class ExternalRotorSpeedTimeSeries(SubsystemTimeSeries):
    """Continuous time series for mechanical systems."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[ExternalRotorSpeed]
    # State
    theta_M: np.ndarray = field(default_factory=empty_array)
    # Input
    w_M: np.ndarray = field(default_factory=empty_array)
    # Derived
    exp_j_theta_M: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: ExternalRotorSpeed) -> None:
        self.w_M = np.array([get_value(subsystem.w_M, t_k) for t_k in t])
        theta_M = np.real(np.array(subsystem._history["theta_M"]))
        self.exp_j_theta_M = np.exp(1j * theta_M)
        self.theta_M = np.angle(self.exp_j_theta_M)
