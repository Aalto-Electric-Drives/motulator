"""Continuous-time grid converter system model."""

from motulator.common.model._base import Model
from motulator.common.model._converter import (
    CapacitiveDCBusConverter,
    VoltageSourceConverter,
)
from motulator.grid.model._ac_filter import LCLFilter, LFilter
from motulator.grid.model._ac_source import (
    ThreePhaseSource,
    ThreePhaseSourceWithSignalInjection,
)


# %%
class GridConverterSystem(Model):
    """
    Continuous-time model for grid-converter systems.

    Parameters
    ----------
    converter : VoltageSourceConverter | CapacitiveDCBusConverter
        Converter model.
    ac_filter : LFilter | LCLFilter
        AC filter model.
    ac_source : ThreePhaseSource | ThreePhaseSourceWithSignalInjection
        Three-phase voltage source.
    pwm : bool, optional
        Enable PWM model, defaults to False.
    delay : int, optional
        Computational delay (samples), defaults to 1.

    """

    def __init__(
        self,
        converter: VoltageSourceConverter | CapacitiveDCBusConverter,
        ac_filter: LFilter | LCLFilter,
        ac_source: ThreePhaseSource | ThreePhaseSourceWithSignalInjection,
        pwm: bool = False,
        delay: int = 1,
    ) -> None:
        self.ac_filter = ac_filter
        self.ac_source = ac_source
        connections = {
            (converter, "i_c_ab"): (ac_filter, "i_c_ab"),
            (ac_filter, "u_c_ab"): (converter, "u_c_ab"),
            (ac_filter, "e_g_ab"): (ac_source, "e_g_ab"),
        }
        subsystems = [converter, ac_filter, ac_source]
        super().__init__(converter, subsystems, connections, pwm, delay)
