"""Continuous-time grid converter system model."""

from motulator.common.model._base import Model
from motulator.common.model._converter import DCBus, VoltageSourceConverter
from motulator.grid.model._ac_filter import LCLFilter, LFilter
from motulator.grid.model._ac_source import ThreePhaseSource


# %%
class GridConverterSystem(Model[VoltageSourceConverter]):
    """
    Continuous-time model for grid-converter systems.

    Parameters
    ----------
    converter : VoltageSourceConverter
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
        Enable PWM model, defaults to False.
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
        converter: VoltageSourceConverter,
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
        super().__init__(
            converter,
            subsystems,
            connections,
            dc_bus=dc_bus,
            pwm=pwm,
            delay=delay,
            single_update=single_update,
        )
