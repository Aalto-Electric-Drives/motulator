"""Continuous-time model for electric machine drives."""

from motulator.common.model._base import Model
from motulator.common.model._converter import FrequencyConverter, VoltageSourceConverter
from motulator.drive.model._lc_filter import LCFilter
from motulator.drive.model._machine import InductionMachine, SynchronousMachine
from motulator.drive.model._mechanics import (
    ExternalRotorSpeed,
    MechanicalSystem,
    TwoMassMechanicalSystem,
)


# %%
class Drive(Model[VoltageSourceConverter | FrequencyConverter]):
    """
    Continuous-time system model for a machine drive.

    Parameters
    ----------
    machine : InductionMachine | SynchronousMachine
        Electric machine model.
    mechanics : MechanicalSystem | TwoMassMechanicalSystem | ExternalRotorSpeed
        Mechanical system model.
    converter : VoltageSourceConverter | FrequencyConverter
        Converter model.
    lc_filter : LCFilter, optional
        LC filter model. If not given, a direct connection between the converter and
        machine is used.
    pwm : bool, optional
        Enable PWM model, defaults to False.
    delay : int, optional
        Computational delay (samples), defaults to 1.

    """

    def __init__(
        self,
        machine: InductionMachine | SynchronousMachine,
        mechanics: MechanicalSystem | TwoMassMechanicalSystem | ExternalRotorSpeed,
        converter: VoltageSourceConverter | FrequencyConverter,
        lc_filter: LCFilter | None = None,
        pwm: bool = False,
        delay: int = 1,
    ) -> None:
        self.machine = machine
        self.mechanics = mechanics
        self.lc_filter = lc_filter
        connections = {
            (machine, "w_M"): (mechanics, "w_M"),
            (mechanics, "tau_M"): (machine, "tau_M"),
        }
        if lc_filter is None:
            subsystems = [converter, machine, mechanics]
            connections |= {
                (converter, "i_c_ab"): (machine, "i_s_ab"),
                (machine, "u_s_ab"): (converter, "u_c_ab"),
            }
        else:
            # The machine comes after the filter, since its current can depend directly
            # on the filter voltage
            subsystems = [converter, lc_filter, machine, mechanics]
            connections |= {
                (converter, "i_c_ab"): (lc_filter, "i_c_ab"),
                (lc_filter, "i_f_ab"): (machine, "i_s_ab"),
                (lc_filter, "u_c_ab"): (converter, "u_c_ab"),
                (machine, "u_s_ab"): (lc_filter, "u_f_ab"),
            }
        super().__init__(converter, subsystems, connections, pwm, delay)
