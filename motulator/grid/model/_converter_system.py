"""Continuous-time grid converter system model."""

from motulator.common.model._ac_source import ThreePhaseSource
from motulator.common.model._base import Model, Subsystem
from motulator.common.model._converter import DCBus, DiodeBridge
from motulator.grid.model._ac_filter import LCLFilter, LFilter


# %%
class GridConverterSystem[C: DiodeBridge](Model[C]):
    """
    Continuous-time model for grid-converter systems.

    The converter is connected to the grid through the AC filter.

    Parameters
    ----------
    converter : DiodeBridge | VoltageSourceConverter
        Converter model.
    ac_filter : LFilter | LCLFilter
        AC filter model.
    ac_source : ThreePhaseSource
        Three-phase voltage source.
    dc_bus : DCBus, optional
        DC bus feeding the converter, which can be shared with other models (e.g., with
        a drive), see `CompositeModel`. Defaults to None, in which case the converter
        has a constant DC-bus voltage.
    dc_load : Subsystem, optional
        Load connected to the DC bus, e.g., a battery or an electrolyzer. Its input
        `u_dc` is the DC-bus voltage and its output `i_dc` the current fed to the DC
        bus (negative when drawn), which replaces the external current of the DC bus.
        Defaults to None.

    """

    def __init__(
        self,
        converter: C,
        ac_filter: LFilter | LCLFilter,
        ac_source: ThreePhaseSource,
        dc_bus: DCBus | None = None,
        dc_load: Subsystem | None = None,
    ) -> None:
        self.ac_filter = ac_filter
        self.ac_source = ac_source
        self.dc_load = dc_load
        connections = {
            (converter, "i_c_ab"): (ac_filter, "i_c_ab"),
            (ac_filter, "u_c_ab"): (converter, "u_c_ab"),
            (ac_filter, "e_g_ab"): (ac_source, "e_g_ab"),
        }
        subsystems = [converter, ac_filter, ac_source]
        if dc_load is not None:
            if dc_bus is None:
                raise ValueError("A DC load requires a DC bus")
            subsystems.append(dc_load)
            connections |= {
                (dc_load, "u_dc"): (dc_bus, "u_dc"),
                (dc_bus, "i_dc"): (dc_load, "i_dc"),
            }
        converter.ac_load = ac_filter
        super().__init__(converter, subsystems, connections, dc_bus=dc_bus)
