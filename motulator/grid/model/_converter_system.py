"""Continuous-time grid converter system model."""

from motulator.common.model._base import Model
from motulator.common.model._converter import (
    CapacitiveDCBusConverter,
    VoltageSourceConverter,
)
from motulator.grid.model._ac_filter import LCLFilter, LFilter
from motulator.grid.model._ac_source import ThreePhaseSource


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
    ac_source : ThreePhaseSource
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
        ac_source: ThreePhaseSource,
        pwm: bool = False,
        delay: int = 1,
    ) -> None:
        super().__init__(pwm, delay)

        # Create subsystems
        self.converter = converter
        self.ac_filter = ac_filter
        self.ac_source = ac_source

        # Store references for interconnection
        self.subsystems = [self.converter, self.ac_filter, self.ac_source]

        # Direct connections without LC filter
        self.connections = {
            (self.converter, "i_c_ab"): (self.ac_filter, "i_c_ab"),
            (self.ac_filter, "u_c_ab"): (self.converter, "u_c_ab"),
            (self.ac_filter, "e_g_ab"): (self.ac_source, "e_g_ab"),
        }

        # Define ZOH inputs separately
        self.zoh_connections = {(self.converter, "q_c_ab"): "sw_state"}
