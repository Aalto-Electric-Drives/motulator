"""Composite of models sharing subsystems."""

from dataclasses import InitVar, dataclass, field
from typing import Any

import numpy as np
from scipy.integrate._ivp.ivp import OdeResult

from motulator.common.model._base import (
    Model,
    ModelTimeSeries,
    Subsystem,
    Wiring,
    check_name,
)


# %%
class CompositeModel(Model[None]):
    """
    Composite of models sharing subsystems.

    The models are coupled through shared subsystem objects, e.g., a DC bus given to
    the models of a grid converter and a drive. Each model keeps its converter, whose
    modulator includes the PWM model and the computational delay, and the model of a
    controlled converter is driven by its own control system in the simulation, see
    `Simulation`. The subsystems are evaluated in the order of the models, a shared
    subsystem once, at its first occurrence, whatever its names in the models (the
    composite names them `model.subsystem` after their first occurrence). The order is
    not resolved automatically: a shared subsystem whose outputs other subsystems
    depend on directly must come first in each model, as `Model` places the DC bus.

    Parameters
    ----------
    models : dict[str, Model]
        Models by name. The names are used for the control systems and the results, so
        they must be valid attribute names (not starting with an underscore, and not
        `t`).

    """

    def __init__(self, models: dict[str, Model]) -> None:
        subsystems: dict[str, Subsystem] = {}
        wiring: Wiring = {}
        converters = [m.converter for m in models.values() if m.converter is not None]
        if len({id(c) for c in converters}) < len(converters):
            raise ValueError("The models must have distinct converters")
        for name, mdl in models.items():
            check_name(name)
            for sub_name, subsystem in mdl.subsystems.items():
                if subsystem not in subsystems.values():
                    subsystems[f"{name}.{sub_name}"] = subsystem
            for src, targets in mdl._outgoing.items():
                for target, inp, out in targets:
                    if (target, inp) in wiring and wiring[target, inp] != (src, out):
                        raise ValueError(
                            f"Conflicting connections to {type(target).__name__}.{inp}"
                        )
                    wiring[target, inp] = (src, out)
        self.converter = None
        self.converters = converters
        self.dc_bus = None
        self.models = models
        self._wire(subsystems, wiring)

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
