"""Composite of models sharing subsystems."""

from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np
from scipy.integrate._ivp.ivp import OdeResult

from motulator.common.model._base import Model, ModelTimeSeries, Subsystem, System
from motulator.common.model._converter import DCBus


# %%
class CompositeModel(System):
    """
    Composite of models sharing subsystems.

    The models are coupled through shared subsystem objects, e.g., a DC bus given to
    the models of a grid converter and a drive. Each model keeps its converter, PWM
    model, and computational delay, and it is driven by its own control system in the
    simulation, see `Simulation`. The subsystems are evaluated in the order of the
    models, a shared subsystem at its first occurrence. The order is not resolved
    automatically: a shared subsystem whose outputs other subsystems depend on
    directly must come first in each model, as `Model` places the DC bus.

    Parameters
    ----------
    models : dict[str, Model]
        Models by name, each with its own converter. The names are used for the
        control systems and the results, so they must be valid attribute names (not
        starting with an underscore, and not `t`).

    """

    def __init__(self, models: dict[str, Model]) -> None:
        subsystems: list[Subsystem] = []
        connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]] = {}
        if len({id(mdl.converter) for mdl in models.values()}) < len(models):
            raise ValueError("The models must have distinct converters")
        for name, mdl in models.items():
            if not name.isidentifier() or name.startswith("_") or name == "t":
                raise ValueError(f"Invalid model name {name!r}")
            subsystems.extend(s for s in mdl.subsystems if s not in subsystems)
            for key, value in mdl.connections.items():
                if key in connections and connections[key] != value:
                    raise ValueError(
                        f"Conflicting connections to {type(key[0]).__name__}.{key[1]}"
                    )
                connections[key] = value
        # Each converter fed from a DC bus must get its voltage
        for dc_bus in subsystems:
            if isinstance(dc_bus, DCBus):
                for converter in dc_bus.converters:
                    if connections.get((converter, "u_dc")) != (dc_bus, "u_dc"):
                        raise ValueError(
                            "The DC bus must be given to the model of each converter"
                        )
        super().__init__(subsystems, connections)
        self.models = models

    def save(self, sol: OdeResult) -> None:
        """Save the solution and the switching states of the models."""
        super().save(sol)
        for mdl in self.models.values():
            mdl.t0 = self.t0
            mdl._history.t.extend(sol.t)
            mdl._history.q_c_ab.extend([mdl.converter.inp.q_c_ab] * len(sol.t))


# %%
@dataclass
class CompositeModelTimeSeries:
    """
    Time series of the simulation results of a composite model.

    The time series of each model is stored as an attribute by the model name (e.g.,
    `drive`), see `ModelTimeSeries`.

    """

    mdl: InitVar[CompositeModel]
    t: np.ndarray = field(default_factory=lambda: np.array([]), init=False)

    def __post_init__(self, mdl: CompositeModel) -> None:
        self.t = np.array(mdl._history.t)
        for name, sub_mdl in mdl.models.items():
            setattr(self, name, ModelTimeSeries(sub_mdl))

    def __getattr__(self, name: str) -> Any:
        """Support type checking for dynamic attributes."""
        error_msg = f"'{self.__class__.__name__}' has no attribute '{name}'"
        raise AttributeError(error_msg)
