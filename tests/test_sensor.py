"""Tests for the sensor model."""

from math import pi

import numpy as np
import pytest

import motulator.drive.control.im as im_control
import motulator.drive.control.sm as sm_control
from motulator.common.model import (
    Model,
    Sensor,
    Simulation,
    SolverCfg,
    ThreePhaseSource,
)
from motulator.common.utils import abc2complex, complex2abc
from motulator.drive import model


def test_bandwidth() -> None:
    """A sensor with a bandwidth follows a sinusoidal signal according to the
    first-order frequency response, wherever it is listed, and a sensor without
    dynamics passes the signal through."""
    w_g, alpha, e_g = 2 * pi * 50, 2 * pi * 500, 100.0
    subsystems = {
        "filtered": Sensor(alpha=alpha),
        "source": ThreePhaseSource(w_g=w_g, e_g=e_g),
        "direct": Sensor(),
    }
    connections = {
        ("filtered", "x"): ("source", "e_g_ab"),
        ("direct", "x"): ("source", "e_g_ab"),
    }
    mdl = Model(None, subsystems, connections)
    cfg = SolverCfg(rtol=1e-6, atol=1e-8)
    res = Simulation(mdl, show_progress=False, cfg=cfg).simulate(t_stop=0.05)

    e_g_ab = e_g * np.exp(1j * w_g * res.mdl.t[-1])
    assert res.mdl.direct.y[-1] == pytest.approx(e_g_ab)
    H = 1 / (1 + 1j * w_g / alpha)  # Differs from unity by 0.1
    assert res.mdl.filtered.y[-1] == pytest.approx(H * e_g_ab, rel=1e-3)


def test_invalid_sensors() -> None:
    """A nonpositive bandwidth and a position sensor with a bandwidth are rejected."""
    for alpha in (0.0, -100.0, float("nan")):
        with pytest.raises(ValueError):
            Sensor(alpha=alpha)
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    with pytest.raises(ValueError):
        model.Drive(
            model.SynchronousMachine(par),
            model.MechanicalSystem(J=0.015),
            model.VoltageSourceConverter(u_dc=540),
            sensors=model.DriveSensors(theta_M=Sensor(alpha=1e3)),
        )


def test_converter_current_is_measured_with_lc_filter() -> None:
    """With an LC filter, the current sensors of both control systems measure the
    converter current, not the stator current."""
    par = model.InductionMachineInvGammaPars(
        n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
    )
    lc_filter = model.LCFilter(L_f=8e-3, C_f=9.9e-6, R_f=0.1)
    lc_filter.state.i_c_ab = 2 + 0j
    mdl = model.Drive(
        model.InductionMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=540),
        lc_filter,
    )
    mdl.set_outputs(0.0)
    assert mdl.machine.out.i_s_ab == 0  # The stator current differs
    cfg = im_control.ObserverBasedVHzControllerCfg(psi_s_nom=1.04, i_s_max=10.6)
    vhz = im_control.VHzControlSystem(im_control.ObserverBasedVHzController(par, cfg))
    vector_cfg = im_control.CurrentVectorControllerCfg(psi_s_nom=1.04, i_s_max=10.6)
    vector = im_control.VectorControlSystem(
        im_control.CurrentVectorController(par, vector_cfg)
    )
    for ctrl in (vhz, vector):
        assert ctrl.get_measurement(mdl).i_c_ab == pytest.approx(2 + 0j)


def test_sample() -> None:
    """The sampled measurement has the channels, the gain, the offset, and the noise."""
    rng = np.random.default_rng(1)
    sensor = Sensor(
        gain=[1.02, 1, 1], offset=[0, 0.1, 0], std=0.5, rng=rng, convert=complex2abc
    )
    sensor.inp.x = 1 + 0j
    sensor.set_outputs(0.0)
    samples = np.array([sensor.sample() for _ in range(2000)])
    np.testing.assert_allclose(np.mean(samples, axis=0), [1.02, -0.4, -0.5], atol=0.05)
    np.testing.assert_allclose(np.std(samples, axis=0), 0.5, rtol=0.1)
    # Parameters changed after construction take effect, also in an ideal sensor
    sensor = Sensor()
    sensor.inp.x = 10.0
    sensor.set_outputs(0.0)
    sensor.gain = 2.0
    assert sensor.sample() == pytest.approx(20.0)
    sensor.gain = None
    assert sensor.sample() == pytest.approx(10.0)


def test_real_signal_stays_real() -> None:
    """A filtered real signal is real, also through a cascade of sensors."""
    speed = model.ExternalRotorSpeed()
    speed.set_external_rotor_speed(10.0)
    subsystems = {
        "speed": speed,
        "first": Sensor(alpha=1e3, y0=10.0),
        "second": Sensor(alpha=1e3, y0=10.0),
    }
    connections = {("first", "x"): ("speed", "w_M"), ("second", "x"): ("first", "y")}
    mdl = Model(None, subsystems, connections)
    res = Simulation(mdl, show_progress=False).simulate(t_stop=0.01)
    assert not np.iscomplexobj(res.mdl.first.y)
    assert not np.iscomplexobj(res.mdl.second.y)
    assert res.mdl.second.y[-1] == pytest.approx(10.0)


def test_drive_sensors() -> None:
    """The control system measures the filtered currents with their offset and the
    filtered DC-bus voltage from its initial value, and the sampled speed and position
    are those of the sensors."""
    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    offset, gain, u_dc = [0.1, 0.0, 0.0], 1.05, 540.0
    sensors = model.DriveSensors(
        i_c=Sensor(alpha=2 * pi * 9e3, offset=offset),
        u_dc=Sensor(alpha=2 * pi * 1e3, y0=u_dc),
        w_M=Sensor(gain=gain),
        theta_M=Sensor(),
    )
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.MechanicalSystem(J=0.015),
        model.VoltageSourceConverter(u_dc=u_dc),
        sensors=sensors,
    )
    est_par = sm_control.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    cfg = sm_control.FluxVectorControllerCfg(i_s_max=6.5, sensorless=False)
    ctrl = sm_control.VectorControlSystem(
        sm_control.FluxVectorController(est_par, cfg),
        sm_control.SpeedController(J=0.015, alpha_s=2 * pi * 4),
    )
    w_M_ref = 50.0
    ctrl.set_speed_ref(w_M_ref)
    res = Simulation(mdl, ctrl, show_progress=False).simulate(t_stop=0.4)
    assert res.success

    # The measured current is the filtered current with the offset
    t, y = res.mdl.t, res.mdl.i_c_sensor.y
    i_c_meas_ab = np.interp(res.ctrl.t, t, y.real) + 1j * np.interp(
        res.ctrl.t, t, y.imag
    )
    i_s_ab_ctrl = np.exp(1j * res.ctrl.fbk.theta_c) * res.ctrl.fbk.i_s
    expected = i_c_meas_ab + abc2complex(offset)
    assert i_s_ab_ctrl == pytest.approx(expected, abs=1e-3 * np.max(np.abs(y)))
    assert np.max(np.abs(y - res.mdl.machine.i_s_ab)) > 0  # The filter lags
    # The filtered DC-bus voltage starts from its initial value, as a real signal
    np.testing.assert_allclose(res.mdl.u_dc_sensor.y, u_dc)
    assert not np.iscomplexobj(res.mdl.u_dc_sensor.y)
    # The sensors sampled at the end of the simulation
    meas = ctrl.get_measurement(mdl)
    assert meas.u_dc == pytest.approx(u_dc)
    assert meas.w_M == pytest.approx(gain * mdl.mechanics.out.w_M)
    assert meas.theta_M == pytest.approx(mdl.mechanics.out.theta_M)
    assert mdl.mechanics.out.w_M == pytest.approx(w_M_ref, rel=0.01)
