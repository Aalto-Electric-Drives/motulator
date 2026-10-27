"""Continuous-time grid converter system model."""

from motulator.common.model._ac_source import ThreePhaseSource
from motulator.common.model._base import Model
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
    pwm : bool, optional
        Enable the PWM model of a voltage-source converter, defaults to False.
    delay : int, optional
        Computational delay in half carrier periods (sampling periods with double
        update), defaults to 1. With single update, use 2 for a delay of one
        sampling period, see `Model`.
    single_update : bool, optional
        Sample and update the duty ratios once per carrier period instead of twice,
        defaults to False, see `Model`.

    """

    def __init__(
        self,
        converter: C,
        ac_filter: LFilter | LCLFilter,
        ac_source: ThreePhaseSource,
        dc_bus: DCBus | None = None,
        pwm: bool = False,
        delay: int = 1,
        single_update: bool = False,
    ) -> None:
        self.ac_filter = ac_filter
        self.ac_source = ac_source
        connections = {
            (converter, "i_c_ab"): (ac_filter, "i_c_ab"),
            (ac_filter, "u_c_ab"): (converter, "u_c_ab"),
            (ac_filter, "e_g_ab"): (ac_source, "e_g_ab"),
        }
        subsystems = [converter, ac_filter, ac_source]
        converter.ac_load = ac_filter
        super().__init__(
            converter,
            subsystems,
            connections,
            dc_bus=dc_bus,
            pwm=pwm,
            delay=delay,
            single_update=single_update,
        )
