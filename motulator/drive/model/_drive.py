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
class Drive(Model):
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
        super().__init__(pwm, delay)

        # Create subsystems
        self.machine = machine
        self.mechanics = mechanics
        self.converter = converter
        if lc_filter is not None:
            self.lc_filter = lc_filter
        else:
            self.lc_filter = None

        # Store references for interconnection
        self.subsystems = [self.converter, self.machine, self.mechanics]

        # Define connections based on presence of LC filter
        if self.lc_filter is None:
            # Direct connections without LC filter
            self.connections = {
                (self.converter, "i_c_ab"): (self.machine, "i_s_ab"),
                (self.machine, "u_s_ab"): (self.converter, "u_c_ab"),
                (self.machine, "w_M"): (self.mechanics, "w_M"),
                (self.mechanics, "tau_M"): (self.machine, "tau_M"),
            }
        else:
            # Connections with LC filter
            self.subsystems.append(self.lc_filter)
            self.connections = {
                (self.converter, "i_c_ab"): (self.lc_filter, "i_c_ab"),
                (self.lc_filter, "i_f_ab"): (self.machine, "i_s_ab"),
                (self.lc_filter, "u_c_ab"): (self.converter, "u_c_ab"),
                (self.machine, "u_s_ab"): (self.lc_filter, "u_f_ab"),
                (self.machine, "w_M"): (self.mechanics, "w_M"),
                (self.mechanics, "tau_M"): (self.machine, "tau_M"),
            }

        # Define ZOH inputs separately
        self.zoh_connections = {(self.converter, "q_c_ab"): "sw_state"}
