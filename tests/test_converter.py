"""
Tests for the defaults of the converter protocol.

A converter without diodes and without a DC-bus model implements only the subsystem
interface and the gate control, which the simulation needs from it.

"""

from dataclasses import InitVar, dataclass, field
from math import inf, pi
from typing import Any

import numpy as np
import pytest

from motulator.common.model import (
    ZOH,
    ControlledConverter,
    Converter,
    Model,
    Simulation,
    SubsystemTimeSeries,
    ThreePhaseSource,
)
from motulator.common.utils import abc2complex
from motulator.grid import control, model


@dataclass
class Inputs:
    """Input variables."""

    i_c_ab: complex = 0j
    q_c_ab: complex = 0j
    enabled: bool = True


@dataclass
class Outputs:
    """Output variables."""

    u_c_ab: complex
    u_dc: float


class IdealConverter(Converter, ControlledConverter):
    """Converter with ideal switches, no diodes, and a constant DC-bus voltage."""

    def __init__(self, u_dc: float) -> None:
        self.inp: Inputs = Inputs()
        self.out: Outputs = Outputs(u_c_ab=0j, u_dc=u_dc)
        self.state = None
        self.modulator = ZOH()

    def modulate(self, T_s: float, command: Any) -> tuple[np.ndarray, np.ndarray]:
        return self.modulator(T_s, command)

    def set_gates(self, gates: np.ndarray) -> None:
        self.inp.q_c_ab = abc2complex(gates[0])

    def set_outputs(self, t: float) -> None:
        self.out.u_c_ab = self.inp.q_c_ab * self.out.u_dc

    def rhs(self, t: float) -> list[complex]:
        return []

    def meas_dc_voltage(self) -> float:
        return self.out.u_dc

    def init_state_history(self) -> None:
        super().init_state_history()
        self._history["q_c_ab"] = []

    def extend_history(self, n: int, u_c_ab: Any) -> None:
        self._history["q_c_ab"].extend([self.inp.q_c_ab] * n)

    def create_time_series(self, t: np.ndarray) -> tuple[str, "ConverterTimeSeries"]:
        return "converter", ConverterTimeSeries(t, self)


@dataclass
class ConverterTimeSeries(SubsystemTimeSeries):
    """Time series of the ideal converter."""

    t: InitVar[np.ndarray]
    subsystem: InitVar[IdealConverter]
    q_c_ab: np.ndarray = field(default_factory=lambda: np.array([]))
    u_c_ab: np.ndarray = field(default_factory=lambda: np.array([]))

    def __post_init__(self, t: np.ndarray, subsystem: IdealConverter) -> None:
        self.q_c_ab = np.array(subsystem._history["q_c_ab"])
        self.u_c_ab = self.q_c_ab * subsystem.out.u_dc


class GridSystem(Model[IdealConverter]):
    """Ideal converter connected to the grid through an L filter."""

    def __init__(
        self,
        converter: IdealConverter,
        ac_filter: model.LFilter,
        ac_source: ThreePhaseSource,
    ) -> None:
        self.ac_filter = ac_filter
        self.ac_source = ac_source
        connections = {
            (converter, "i_c_ab"): (ac_filter, "i_c_ab"),
            (ac_filter, "u_c_ab"): (converter, "u_c_ab"),
            (ac_filter, "e_g_ab"): (ac_source, "e_g_ab"),
        }
        super().__init__(converter, [converter, ac_filter, ac_source], connections)


def test_converter_without_diodes_and_dc_bus() -> None:
    """The simulation needs only the subsystem interface and the gate control."""
    converter = IdealConverter(u_dc=650.0)
    assert converter.ac_load is None and not converter.blocked
    assert converter.margin == inf
    with pytest.raises(NotImplementedError):
        converter.set_enable(False)
    with pytest.raises(TypeError):  # A DC bus would draw the current
        converter.compute_internal_dc_current(converter.inp)

    mdl = GridSystem(
        converter,
        model.LFilter(L_f=10e-3),
        ThreePhaseSource(w_g=2 * pi * 50, e_g=np.sqrt(2 / 3) * 400),
    )
    cfg = control.CurrentVectorControllerCfg(i_max=20.0, L=10e-3, T_s=100e-6)
    ctrl = control.GridConverterControlSystem(control.CurrentVectorController(cfg))
    ctrl.set_power_ref(5e3)
    ctrl.set_reactive_power_ref(0)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.04)
    assert res.success
    # Converter power to the grid, averaged over the last 10 ms
    p_c = 1.5 * np.real(res.mdl.converter.u_c_ab * np.conj(res.mdl.ac_filter.i_c_ab))
    assert np.mean(p_c[res.mdl.t > 0.03]) == pytest.approx(5e3, rel=0.05)
