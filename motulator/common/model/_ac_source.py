"""Three-phase voltage source model."""

from collections.abc import Callable
from dataclasses import InitVar, dataclass, field
from math import pi, sqrt
from typing import Any

import numpy as np

from motulator.common.model._base import Subsystem, SubsystemTimeSeries
from motulator.common.utils._utils import empty_array, get_value, wrap


@dataclass
class States:
    """State variables."""

    theta_g: complex = 0j  # Angle (rad), imaginary part always zero


@dataclass
class Outputs:
    """Output variables."""

    e_g_ab: complex = complex(1)
    exp_j_theta_g: complex = complex(1)


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

    def set_states(self, state_list: list[complex], index: int) -> int:
        """Set the state, keeping the angle within [-pi, pi)."""
        # The angle is wrapped only in the state of the model, which the next
        # integration interval starts from, so the solver sees no discontinuity
        index = super().set_states(state_list, index)
        self.state.theta_g = wrap(self.state.theta_g.real)
        return index

    def set_outputs(self, t) -> None:
        """Set output variables."""
        exp_j_theta_g = np.exp(1j * self.state.theta_g.real)
        self.out.e_g_ab = self.generate_space_vector(t, exp_j_theta_g)
        self.out.exp_j_theta_g = exp_j_theta_g

    def rhs(self, t) -> list[complex]:
        """Compute the state derivative."""
        return [get_value(self.w_g, t)]

    def create_time_series(self, t: np.ndarray) -> "ThreePhaseSourceTimeSeries":
        """Create time series from state list."""
        return ThreePhaseSourceTimeSeries(t, self)


@dataclass
class ThreePhaseSourceTimeSeries(SubsystemTimeSeries):
    """Continuous time series."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[ThreePhaseSource]
    exp_j_theta_g: np.ndarray = field(default_factory=empty_array)
    w_g: np.ndarray = field(default_factory=empty_array)
    theta_g: np.ndarray = field(default_factory=empty_array)
    e_g_ab: np.ndarray = field(default_factory=empty_array)

    def __post_init__(self, t: np.ndarray, subsystem: ThreePhaseSource) -> None:
        """Compute output time series from the states."""
        theta_g = np.real(np.array(subsystem._history["theta_g"]))
        self.exp_j_theta_g = np.exp(1j * theta_g)
        self.theta_g = np.angle(self.exp_j_theta_g)
        self.w_g = np.vectorize(get_value)(subsystem.w_g, t)
        self.e_g_ab = subsystem.generate_space_vector(t, self.exp_j_theta_g)
