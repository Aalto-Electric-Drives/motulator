"""Three-phase voltage source model."""

from collections.abc import Callable
from dataclasses import InitVar, dataclass, field
from math import pi, sqrt
from typing import Any

import numpy as np

from motulator.common.model import Subsystem, SubsystemTimeSeries
from motulator.common.utils._utils import empty_array, get_value


@dataclass
class States:
    """State variables."""

    exp_j_theta_g: complex = complex(1)


@dataclass
class Outputs:
    """Output variables."""

    e_g_ab: complex = complex(1)
    exp_j_theta_g: complex = complex(1)


@dataclass
class StateHistory:
    """State history."""

    exp_j_theta_g: list[complex] = field(default_factory=list)


class ThreePhaseSource(Subsystem):
    """
    Three-phase source model.

    The frequency, phase shift, and magnitude can be given either as constants or
    functions of time. An unbalanced source can be modeled by specifying a negative-
    sequence component. The zero-sequence component is not included in this model.

    Parameters
    ----------
    w_g : float | Callable[[float], float], optional
        Angular frequency (rad/s), defaults to 2*pi*50.
    e_g : float | Callable[[float], float], optional
        Peak-valued magnitude of positive-sequence component, defaults to sqrt(2/3)*400.
    phi : float | Callable[[float], float], optional
        Phase shift (rad) of positive-sequence component, defaults to 0.
    e_g_neg : float | Callable[[float], float], optional
        Peak-valued magnitude of negative-sequence component, defaults to 0.
    phi_neg : float | Callable[[float], float], optional
        Phase shift (rad) of negative-sequence component, defaults to 0.

    Notes
    -----
    This model is typically used to represent a voltage source, but it can be configured
    to represent, e.g., a current source as well.

    """

    def __init__(
        self,
        w_g: float | Callable[[float], float] = 2 * pi * 50,
        e_g: float | Callable[[float], float] = sqrt(2 / 3) * 400,
        phi: float | Callable[[float], float] = 0.0,
        e_g_neg: float | Callable[[float], float] = 0.0,
        phi_neg: float | Callable[[float], float] = 0.0,
    ) -> None:
        self.w_g = w_g
        self.e_g = e_g
        self.phi = phi
        self.e_g_neg = e_g_neg
        self.phi_neg = phi_neg
        self.inp = None
        self.state: States = States()
        self.out: Outputs = Outputs()
        self._history: StateHistory = StateHistory()

    def generate_space_vector(self, t: Any, exp_j_theta_g: Any) -> Any:
        """Generate the space vector in stationary coordinates."""
        e_g = get_value(self.e_g, t)
        phi = get_value(self.phi, t)
        e_g_neg = get_value(self.e_g_neg, t)
        phi_neg = get_value(self.phi_neg, t)
        # Space vector in stationary coordinates
        e_g_ab = e_g * exp_j_theta_g * np.exp(1j * phi)
        # Add possible negative sequence component
        e_g_ab += e_g_neg * np.conj(exp_j_theta_g * np.exp(1j * phi_neg))
        return e_g_ab

    def set_outputs(self, t) -> None:
        """Set output variables."""
        self.out.e_g_ab = self.generate_space_vector(t, self.state.exp_j_theta_g)
        self.out.exp_j_theta_g = self.state.exp_j_theta_g

    def rhs(self, t) -> list[complex]:
        """Compute the state derivative."""
        w_g = get_value(self.w_g, t)
        d_exp_j_theta_g = 1j * w_g * self.state.exp_j_theta_g
        return [d_exp_j_theta_g]

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "ThreePhaseSourceTimeSeries"]:
        """Create time series from state list."""
        return "ac_source", ThreePhaseSourceTimeSeries(t, self)


@dataclass
class ThreePhaseSourceTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[ThreePhaseSource]
    exp_j_theta_g: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: ThreePhaseSource) -> None:
        """Compute output time series from the states."""
        self.exp_j_theta_g = np.array(subsystem._history.exp_j_theta_g)
        self.w_g = np.vectorize(get_value)(subsystem.w_g, t)
        self.theta_g = np.angle(self.exp_j_theta_g)
        self.e_g_ab = subsystem.generate_space_vector(t, self.exp_j_theta_g)


@dataclass
class SignalInjectionStates:
    """State variables."""

    exp_j_theta_g: complex = complex(1)
    exp_j_theta_e: complex = complex(1)


@dataclass
class SignalInjectionStateHistory:
    """State history."""

    exp_j_theta_g: list[complex] = field(default_factory=list)
    exp_j_theta_e: list[complex] = field(default_factory=list)


class ThreePhaseSourceWithSignalInjection(Subsystem):
    """
    Three-phase source model with additional signal injection in dq-coordinates.

    This class implements a three-phase voltage source model with signal injection
    capabilities, intended for use in converter output admittance identification. The
    excitation signal is added to the output voltage of the source. The zero-sequence
    component is not included in this model.

    Parameters
    ----------
    w_g : float, optional
        Angular frequency (rad/s), defaults to 2*pi*50.
    e_g : float, optional
        Peak-valued magnitude of positive-sequence component, defaults to sqrt(2/3)*400.
    phi : float, optional
        Phase shift (rad) of positive-sequence component, defaults to 0.
    u_ed : float, optional
        Magnitude of d-axis excitation signal (V), defaults to 0.
    u_eq : float, optional
        Magnitude of q-axis excitation signal (V), defaults to 0.

    Notes
    -----
    This model is typically used to represent a voltage source, but it can be configured
    to represent, e.g., a current source as well.

    """

    def __init__(
        self,
        w_g: float = 2 * pi * 50,
        e_g: float = sqrt(2 / 3) * 400,
        phi: float = 0.0,
        u_ed: float = 0.0,
        u_eq: float = 0.0,
        f_e: float = 0.0,
    ) -> None:
        self.w_g = w_g
        self.e_g = e_g
        self.phi = phi
        self.u_ed = u_ed
        self.u_eq = u_eq
        self.f_e = f_e
        self.inp = None
        self.state: SignalInjectionStates = SignalInjectionStates()
        self.out: Outputs = Outputs()
        self._history: SignalInjectionStateHistory = SignalInjectionStateHistory()

    def generate_space_vector(
        self, t: Any, exp_j_theta_g: Any, exp_j_theta_e: Any
    ) -> Any:
        """Generate the space vector in stationary coordinates."""
        e_g = get_value(self.e_g, t)
        phi = get_value(self.phi, t)

        # Add excitation signal in synchronous coordinates
        e_g += (self.u_ed + 1j * self.u_eq) * np.real(exp_j_theta_e)

        # Space vector in stationary coordinates
        e_g_ab = e_g * exp_j_theta_g * np.exp(1j * phi)
        return e_g_ab

    def set_outputs(self, t) -> None:
        """Set output variables."""
        self.out.e_g_ab = self.generate_space_vector(
            t, self.state.exp_j_theta_g, self.state.exp_j_theta_e
        )
        self.out.exp_j_theta_g = self.state.exp_j_theta_g

    def rhs(self, t) -> list[complex]:
        """Compute the state derivative."""
        w_g = get_value(self.w_g, t)
        d_exp_j_theta_g = 1j * w_g * self.state.exp_j_theta_g
        d_exp_j_theta_e = 2 * np.pi * 1j * self.f_e * self.state.exp_j_theta_e
        return [d_exp_j_theta_g, d_exp_j_theta_e]

    def create_time_series(
        self, t: np.ndarray
    ) -> tuple[str, "SignalInjectionTimeSeries"]:
        """Create time series from state list."""
        return "ac_source", SignalInjectionTimeSeries(t, self)


@dataclass
class SignalInjectionTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[ThreePhaseSourceWithSignalInjection]
    exp_j_theta_g: np.ndarray = field(default_factory=empty_array)

    def __post_init__(
        self, t: np.ndarray, subsystem: ThreePhaseSourceWithSignalInjection
    ) -> None:
        """Compute output time series from the states."""
        self.exp_j_theta_g = np.array(subsystem._history.exp_j_theta_g)
        self.exp_j_theta_e = np.array(subsystem._history.exp_j_theta_e)
        self.w_g = np.vectorize(get_value)(subsystem.w_g, t)
        self.theta_g = np.angle(self.exp_j_theta_g)
        self.e_g_ab = subsystem.generate_space_vector(
            t, self.exp_j_theta_g, self.exp_j_theta_e
        )
