"""Continuous-time models for electric machine drives and grid-connected machines."""

from dataclasses import dataclass, field

from motulator.common.model._ac_source import ThreePhaseSource
from motulator.common.model._base import Model, Subsystem
from motulator.common.model._converter import DCBus, VoltageSourceConverter
from motulator.common.model._sensor import Sensor
from motulator.common.utils._utils import complex2abc
from motulator.drive.model._lc_filter import LCFilter
from motulator.drive.model._machine import InductionMachine, SynchronousMachine
from motulator.drive.model._mechanics import (
    ExternalRotorSpeed,
    MechanicalSystem,
    TwoMassMechanicalSystem,
)


# %%
@dataclass
class DriveSensors:
    """
    Sensors of a drive, ideal by default, see `Sensor`.

    Parameters
    ----------
    i_c : Sensor, optional
        Converter phase-current sensors. Their conversion defaults to `complex2abc`.
    u_dc : Sensor, optional
        DC-bus voltage sensor.
    w_M : Sensor, optional
        Rotor speed sensor (mechanical rad/s), defaults to None, i.e., no sensor.
        Sensored control of an induction machine uses the speed sensor.
    theta_M : Sensor, optional
        Rotor position sensor (mechanical rad), defaults to None, i.e., no sensor.
        Sensored control of a synchronous machine uses the position sensor. The sensor
        cannot have a bandwidth, since the angle is wrapped.

    """

    i_c: Sensor = field(default_factory=Sensor)
    u_dc: Sensor = field(default_factory=Sensor)
    w_M: Sensor | None = None
    theta_M: Sensor | None = None

    def __post_init__(self) -> None:
        if self.i_c.convert is None:
            self.i_c.convert = complex2abc


# %%
class Drive(Model[VoltageSourceConverter]):
    """
    Continuous-time system model for a machine drive.

    Parameters
    ----------
    machine : InductionMachine | SynchronousMachine
        Electric machine model.
    mechanics : MechanicalSystem | TwoMassMechanicalSystem | ExternalRotorSpeed
        Mechanical system model.
    converter : VoltageSourceConverter
        Converter model.
    lc_filter : LCFilter, optional
        LC filter model. If not given, a direct connection between the converter and
        machine is used.
    dc_bus : DCBus, optional
        DC bus feeding the converter, which can be shared with other models (e.g., with
        a grid converter system with a diode bridge or an active front end), see
        `CompositeModel`. Defaults to None, in which case the converter has a constant
        DC-bus voltage.
    sensors : DriveSensors, optional
        Sensors, which the control system samples, defaults to ideal phase-current and
        DC-bus voltage sensors. They are the subsystems `i_c_sensor`, `u_dc_sensor`,
        and, if given, `w_M_sensor` and `theta_M_sensor` of the model, connected to the
        signals they measure.

    """

    def __init__(
        self,
        machine: InductionMachine | SynchronousMachine,
        mechanics: MechanicalSystem | TwoMassMechanicalSystem | ExternalRotorSpeed,
        converter: VoltageSourceConverter,
        lc_filter: LCFilter | None = None,
        dc_bus: DCBus | None = None,
        sensors: DriveSensors | None = None,
    ) -> None:
        self.machine = machine
        self.mechanics = mechanics
        self.lc_filter = lc_filter
        self.sensors = DriveSensors() if sensors is None else sensors
        subsystems: dict[str, Subsystem] = {"converter": converter}
        connections = {
            ("machine", "w_M"): ("mechanics", "w_M"),
            ("mechanics", "tau_M"): ("machine", "tau_M"),
        }
        if lc_filter is None:
            connections |= {
                ("converter", "i_c_ab"): ("machine", "i_s_ab"),
                ("machine", "u_s_ab"): ("converter", "u_c_ab"),
                ("i_c_sensor", "x"): ("machine", "i_s_ab"),
            }
            converter.ac_load = machine
        else:
            subsystems["lc_filter"] = lc_filter
            connections |= {
                ("converter", "i_c_ab"): ("lc_filter", "i_c_ab"),
                ("lc_filter", "i_f_ab"): ("machine", "i_s_ab"),
                ("lc_filter", "u_c_ab"): ("converter", "u_c_ab"),
                ("machine", "u_s_ab"): ("lc_filter", "u_f_ab"),
                ("i_c_sensor", "x"): ("lc_filter", "i_c_ab"),
            }
            converter.ac_load = lc_filter
        subsystems |= {
            "machine": machine,
            "mechanics": mechanics,
            "i_c_sensor": self.sensors.i_c,
            "u_dc_sensor": self.sensors.u_dc,
        }
        connections[("u_dc_sensor", "x")] = ("converter", "u_dc")
        if self.sensors.w_M is not None:
            subsystems["w_M_sensor"] = self.sensors.w_M
            connections[("w_M_sensor", "x")] = ("mechanics", "w_M")
        if self.sensors.theta_M is not None:
            if self.sensors.theta_M.state is not None:
                raise ValueError("The position sensor cannot have a bandwidth")
            subsystems["theta_M_sensor"] = self.sensors.theta_M
            connections[("theta_M_sensor", "x")] = ("mechanics", "theta_M")
        super().__init__(converter, subsystems, connections, dc_bus=dc_bus)


# %%
class GridConnectedMachine(Model[None]):
    """
    Continuous-time system model for a machine connected directly to the grid.

    The stator terminals of the machine are connected to a three-phase voltage source,
    e.g., to model the direct-on-line start of an induction machine. A series grid
    impedance can be lumped into the machine parameters, e.g., into the stator
    resistance and the leakage inductance of the inverse-Γ model of an induction
    machine.

    Parameters
    ----------
    machine : InductionMachine | SynchronousMachine
        Electric machine model.
    mechanics : MechanicalSystem | TwoMassMechanicalSystem | ExternalRotorSpeed
        Mechanical system model.
    ac_source : ThreePhaseSource
        Three-phase voltage source.

    """

    def __init__(
        self,
        machine: InductionMachine | SynchronousMachine,
        mechanics: MechanicalSystem | TwoMassMechanicalSystem | ExternalRotorSpeed,
        ac_source: ThreePhaseSource,
    ) -> None:
        self.machine = machine
        self.mechanics = mechanics
        self.ac_source = ac_source
        connections = {
            ("machine", "u_s_ab"): ("ac_source", "e_g_ab"),
            ("machine", "w_M"): ("mechanics", "w_M"),
            ("mechanics", "tau_M"): ("machine", "tau_M"),
        }
        subsystems: dict[str, Subsystem] = {
            "ac_source": ac_source,
            "machine": machine,
            "mechanics": mechanics,
        }
        super().__init__(None, subsystems, connections)
