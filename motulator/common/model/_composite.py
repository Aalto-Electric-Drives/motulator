"""Composite of models sharing subsystems."""

from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np
from scipy.integrate._ivp.ivp import OdeResult

from motulator.common.model._base import Model, ModelTimeSeries, Subsystem


# %%
class CompositeModel(Model[None]):
    """
    Composite of models sharing subsystems.

    The models are coupled through shared subsystem objects, e.g., a DC bus given to
    the models of a grid converter and a drive. Each model keeps its converter, PWM
    model, and computational delay, and the model of a controlled converter is driven
    by its own control system in the simulation, see `Simulation`. The subsystems are
    evaluated in the order of the models, a shared subsystem at its first occurrence.
    The order is not resolved automatically: a shared subsystem whose outputs other
    subsystems depend on directly must come first in each model, as `Model` places the
    DC bus.

    Parameters
    ----------
    models : dict[str, Model]
        Models by name. The names are used for the control systems and the results, so
        they must be valid attribute names (not starting with an underscore, and not
        `t`).

    """

    def __init__(self, models: dict[str, Model]) -> None:
        subsystems: list[Subsystem] = []
        connections: dict[tuple[Subsystem, str], tuple[Subsystem, str]] = {}
        converters = [m.converter for m in models.values() if m.converter is not None]
        if len({id(c) for c in converters}) < len(converters):
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
        super().__init__(None, subsystems, connections)
        self.models = models
        self.converters = converters

    def save(self, sol: OdeResult) -> None:
        """Save the solution and the time histories of the models."""
        super().save(sol)
        for mdl in self.models.values():
            mdl._t.extend(sol.t)


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
        self.t = np.array(mdl._t)
        for name, sub_mdl in mdl.models.items():
            setattr(self, name, ModelTimeSeries(sub_mdl))

    def __getattr__(self, name: str) -> Any:
        """Support type checking for dynamic attributes."""
        error_msg = f"'{self.__class__.__name__}' has no attribute '{name}'"
        raise AttributeError(error_msg)
