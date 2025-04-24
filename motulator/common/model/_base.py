"""Base classes for models."""

from dataclasses import InitVar, dataclass, field
from typing import Any, Protocol

import numpy as np
from scipy.integrate._ivp.ivp import OdeResult

from motulator.common.model._pwm import ZOH, CarrierComparison


# %%
class SubsystemInputs(Protocol):
    """Protocol for subsystem inputs."""


class SubsystemOutputs(Protocol):
    """Protocol for subsystem outputs."""


class SubsystemStates(Protocol):
    """Protocol for subsystem states."""


class SubsystemStateHistory(Protocol):
    """Protocol for subsystem state histories."""


class Subsystem[
    Inp: SubsystemInputs,
    Out: SubsystemOutputs,
    State: SubsystemStates,
    History: SubsystemStateHistory,
](Protocol):
    """
    Protocol defining the interface for all subsystems.

    This class defines the interface for all subsystems. It is a generic class that can
    be used with different input, output, state, and history types. The class provides
    methods for setting states, outputs, and computing state derivatives. It also
    provides methods for extending the state history and creating time-series
    representations of the subsystem.

    """

    inp: Inp | None
    out: Out
    state: State | None
    _history: History | None

    def set_states(self, state_list: list[complex], index: int) -> int:
        """Set states from the state list."""
        if self.state is not None:
            for attr in vars(self.state):
                setattr(self.state, attr, state_list[index])
                index += 1
        return index

    def set_outputs(self, t: float) -> None:
        """Set output variables."""
        ...

    def rhs(self, t: float) -> list[complex]:
        """Compute state derivatives."""
        ...

    def extend_state_history(self, sol_y: list[list[float]], index: int) -> int:
        """Extend the state history with values from solver output."""
        if self._history is not None:
            for attr, value in vars(self._history).items():
                if isinstance(value, list):
                    value.extend(sol_y[index])
                else:
                    setattr(self._history, attr, list(sol_y[index]))
                index += 1
        return index

    def create_time_series(self, t: np.ndarray) -> tuple[str, "SubsystemTimeSeries"]:
        """Create time-series representation of this subsystem."""
        ...


# %%
@dataclass
class ModelStateHistory:
    """Temporary storage."""

    t: list[float] = field(default_factory=list)


class Model:
    """
    Base class for continuous-time system models.

    This class defines the interface for continuous-time system models. It provides
    methods for setting initial values, computing state derivatives, interconnecting
    subsystems, and saving simulation results. The class also provides methods for
    setting ZOH inputs and computing outputs. The model can be configured to use either
    PWM or ZOH for the carrier comparison. The class also provides a method for saving
    the simulation results, which includes the time history and the ZOH inputs. The
    class is designed to be subclassed for specific applications.

    """

    def __init__(self, pwm: bool = False, delay: int = 0) -> None:
        self.t0: float = 0.0
        self.delay = Delay(delay)
        self.pwm = CarrierComparison() if pwm else ZOH()
        self.subsystems: list[Subsystem] = []
        self.connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]] = {}
        self.zoh_connections: dict[tuple[Subsystem, str], str] = {}
        self.zoh_inputs: dict[str, Any] = {}
        self._history = ModelStateHistory()

    def get_initial_values(self) -> list[complex]:
        """Get initial values of all subsystems before the solver."""
        state0: list[complex] = []
        for subsystem in self.subsystems:
            if subsystem.state is not None:
                state0.extend(vars(subsystem.state).values())
        return state0

    def set_zoh_input(self, name: str, value: Any) -> None:
        """Set a specific ZOH input value."""
        self.zoh_inputs[name] = value
        # Update any subsystem that uses this input
        for (target, target_attr), input_name in self.zoh_connections.items():
            if input_name == name:
                setattr(target.inp, target_attr, value)

    def set_states(self, state_list: list[complex]) -> None:
        """Set states in all subsystems."""
        index = 0
        for subsystem in self.subsystems:
            index = subsystem.set_states(state_list, index)

    def set_outputs(self, t: float) -> None:
        """Compute output variables."""
        for subsystem in self.subsystems:
            subsystem.set_outputs(t)

    def interconnect(self) -> None:
        """Connect subsystem inputs and outputs."""
        for (target, target_attr), (src, src_attr) in self.connections.items():
            setattr(target.inp, target_attr, getattr(src.out, src_attr))

    def rhs(self, t: float, state_list: list[complex]) -> list[complex]:
        """Compute complete state derivative list for the solver."""
        self.set_states(state_list)
        self.set_outputs(t)
        self.interconnect()
        rhs_list: list[complex] = []
        for subsystem in self.subsystems:
            if derivatives := subsystem.rhs(t):
                rhs_list.extend(derivatives)
        return rhs_list

    def save(self, sol: OdeResult) -> None:
        """Save solution with all ZOH inputs."""
        self._history.t.extend(sol.t)
        # Save ZOH inputs for the current time span
        for name, value in self.zoh_inputs.items():
            if not hasattr(self._history, name):
                setattr(self._history, name, [])
            getattr(self._history, name).extend([value] * len(sol.t))
        # Save states
        index = 0
        for subsystem in self.subsystems:
            index = subsystem.extend_state_history(sol.y, index)


# %%
class SubsystemTimeSeries[S: Subsystem](Protocol):
    """Base class for subsystem time series."""

    t: np.ndarray

    def compute_zoh_input_derived_signals(self, t: np.ndarray, subsystem: S) -> None:
        """Compute additional time series using subsystem's ZOH inputs."""
        pass

    def compute_input_derived_signals(self, t: np.ndarray, subsystem: S) -> None:
        """Compute additional time series using subsystem's regular inputs."""
        pass


@dataclass
class ModelTimeSeries:
    """Container for simulation result time series."""

    _history: InitVar[ModelStateHistory]
    subsystems: InitVar[list[Subsystem] | None] = None
    connections: InitVar[dict[tuple[Subsystem, str], tuple[Subsystem, str]] | None] = (
        None
    )
    zoh_connections: InitVar[dict[tuple[Subsystem, str], str] | None] = None
    t: np.ndarray = field(default_factory=lambda: np.array([]), init=False)

    def __post_init__(self, history, subsystems, connections, zoh_connections) -> None:
        self.t = np.array(history.t)
        # Process ZOH inputs
        for attr_name, value in vars(history).items():
            if attr_name != "t" and not attr_name.startswith("_"):
                setattr(self, attr_name, np.array(value))
        # Process subsystems
        zoh_connections = zoh_connections or {}
        if subsystems is not None and connections is not None:
            self.build_subsystem_time_series(subsystems, connections, zoh_connections)

    def __getattr__(self, name: str) -> Any:
        """Support type checking for dynamic attributes."""
        # This helps type checkers understand dynamic attributes
        error_msg = f"'{self.__class__.__name__}' has no attribute '{name}'"
        raise AttributeError(error_msg)

    def build_subsystem_time_series(
        self, subsystems: list, connections: dict, zoh_connections: dict
    ) -> None:
        """Build time series for all subsystems."""
        ts_objects = self._create_time_series(subsystems)
        self._add_zoh_input_signals(ts_objects, zoh_connections)
        self._compute_zoh_input_derived_signals(subsystems, ts_objects)
        self._add_input_signals(ts_objects, connections)
        self._compute_input_derived_signals(subsystems, ts_objects)

    def _create_time_series(
        self, subsystems: list
    ) -> dict[Subsystem, SubsystemTimeSeries]:
        """
        Create time series objects using subsystem methods.

        This method converts the state history lists of each subsystem to the
        corresponding Numpy arrays. These time series arrays are stored as attributes of
        the ModelTimeSeries object.

        """
        ts_objects = {}
        for subsystem in subsystems:
            attr_name, ts = subsystem.create_time_series(self.t)
            setattr(self, attr_name, ts)
            ts_objects[subsystem] = ts
        return ts_objects

    def _add_zoh_input_signals(self, ts_objects: dict, zoh_connections: dict) -> None:
        """
        Add ZOH inputs' time series to subsystems.

        This methods adds ZOH input time series (where available) to each subsystem
        based on the ZOH connections dictionary. This helps plotting and analysis of the
        simulation results, since each subsystem time series contains all the necessary
        data. Typical ZOH inputs are the switching states of the converters.

        """
        if not zoh_connections:
            return
        for (target, target_attr), input_name in zoh_connections.items():
            if (target_ts := ts_objects.get(target)) is not None:
                setattr(target_ts, target_attr, np.array(getattr(self, input_name)))

    def _compute_zoh_input_derived_signals(
        self, subsystems: list, ts_objects: dict
    ) -> None:
        """
        Compute additional time series using the added ZOH inputs.

        Additional time series can be computed using the added ZOH inputs and the
        internal states of the subsystems. For example, the converter output voltage can
        be computed using the switching states (ZOH signals) and the DC-bus voltage
        (internal continuous-time signal).

        """
        for subsystem in subsystems:
            if (ts := ts_objects.get(subsystem)) is not None:
                ts.compute_zoh_input_derived_signals(self.t, subsystem)

    def _add_input_signals(self, ts_objects: dict, connections: dict) -> None:
        """
        Add continuous-time inputs' time series to subsystems.

        This methods adds continuous-time inputs' time series to each subsystem based on
        the connections dictionary. This helps plotting and analysis of the simulation
        results, since each subsystem time series contains all the necessary data.

        """
        for (target, target_attr), (src, src_attr) in connections.items():
            target_ts = ts_objects.get(target)
            source_ts = ts_objects.get(src)
            if target_ts is not None and source_ts is not None:
                data = getattr(source_ts, src_attr)
                setattr(target_ts, target_attr, np.array(data))

    def _compute_input_derived_signals(
        self, subsystems: list, ts_objects: dict
    ) -> None:
        """
        Compute additional time series using the added continuous-time inputs.

        Additional outputs containing direct feedthrough can be computed at this stage.

        """
        for subsystem in subsystems:
            if (ts := ts_objects.get(subsystem)) is not None:
                ts.compute_input_derived_signals(self.t, subsystem)


# %%
class Delay:
    """
    Computational delay modeled as a ring buffer.

    Parameters
    ----------
    length : int, optional
        Length of the buffer in samples, defaults to 1.
    elem : int, optional
        Number of elements in each sample, defaults to 3.

    """

    def __init__(self, length: int = 1, elem: int = 3) -> None:
        self.data = [elem * [0] for _ in range(length)]  # Creates zero lists

    def __call__(self, u: Any) -> Any:
        """
        Delay the input.

        Parameters
        ----------
        u : array_like, shape (elem,)
            Input array.

        Returns
        -------
        array_like, shape (elem,)
            Output array.

        """
        # Add the latest value to the end of the list
        self.data.append(u)
        # Pop the first element and return it
        return self.data.pop(0)
