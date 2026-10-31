"""Continuous-time grid converter system model."""

from dataclasses import dataclass, field

from motulator.common.model._ac_source import ThreePhaseSource
from motulator.common.model._base import Model, Subsystem
from motulator.common.model._converter import DCBus, DiodeBridge
from motulator.common.model._sensor import Sensor
from motulator.common.utils._utils import complex2abc, complex2line
from motulator.grid.model._ac_filter import LCLFilter, LFilter


# %%
@dataclass
class GridConverterSensors:
    """
    Sensors of a grid converter system, ideal by default, see `Sensor`.

    Parameters
    ----------
    i_c : Sensor, optional
        Converter phase-current sensors. Their conversion defaults to `complex2abc`.
    u_g : Sensor, optional
        Sensors of the line-to-line voltages `u_ab` and `u_bc` at the point of common
        coupling. Their conversion defaults to `complex2line`.
    u_dc : Sensor, optional
        DC-bus voltage sensor.

    """

    i_c: Sensor = field(default_factory=Sensor)
    u_g: Sensor = field(default_factory=Sensor)
    u_dc: Sensor = field(default_factory=Sensor)

    def __post_init__(self) -> None:
        if self.i_c.convert is None:
            self.i_c.convert = complex2abc
        if self.u_g.convert is None:
            self.u_g.convert = complex2line


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
        Load connected to the DC bus, e.g., a battery or an electrolyzer, named
        `dc_load` in the results. Its input `u_dc` is the DC-bus voltage and its output
        `i_dc` the current fed to the DC bus (negative when drawn), which replaces the
        external current of the DC bus. Defaults to None.
    sensors : GridConverterSensors, optional
        Sensors, which the control system samples, defaults to ideal sensors. They are
        the subsystems `i_c_sensor`, `u_g_sensor`, and `u_dc_sensor` of the model,
        connected to the signals they measure.

    """

    def __init__(
        self,
        converter: C,
        ac_filter: LFilter | LCLFilter,
        ac_source: ThreePhaseSource,
        dc_bus: DCBus | None = None,
        dc_load: Subsystem | None = None,
        sensors: GridConverterSensors | None = None,
    ) -> None:
        self.ac_filter = ac_filter
        self.ac_source = ac_source
        self.dc_load = dc_load
        self.sensors = GridConverterSensors() if sensors is None else sensors
        connections = {
            ("converter", "i_c_ab"): ("ac_filter", "i_c_ab"),
            ("ac_filter", "u_c_ab"): ("converter", "u_c_ab"),
            ("ac_filter", "e_g_ab"): ("ac_source", "e_g_ab"),
            ("i_c_sensor", "x"): ("ac_filter", "i_c_ab"),
            ("u_g_sensor", "x"): ("ac_filter", "u_g_ab"),
            ("u_dc_sensor", "x"): ("converter", "u_dc"),
        }
        subsystems: dict[str, Subsystem] = {
            "converter": converter,
            "ac_filter": ac_filter,
            "ac_source": ac_source,
            "i_c_sensor": self.sensors.i_c,
            "u_g_sensor": self.sensors.u_g,
            "u_dc_sensor": self.sensors.u_dc,
        }
        if dc_load is not None:
            if dc_bus is None:
                raise ValueError("A DC load requires a DC bus")
            subsystems["dc_load"] = dc_load
            connections |= {
                ("dc_load", "u_dc"): ("dc_bus", "u_dc"),
                ("dc_bus", "i_dc"): ("dc_load", "i_dc"),
            }
        converter.ac_load = ac_filter
        super().__init__(converter, subsystems, connections, dc_bus=dc_bus)
