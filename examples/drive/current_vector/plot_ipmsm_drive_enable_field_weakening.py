"""
IPMSM drive enable in field weakening
=====================================

This example accelerates a 2.2-kW interior permanent-magnet synchronous machine
(IPMSM) into field weakening, disables the drive while it is above base speed,
and re-enables it with sensored flying-start before it leaves the high-speed
region. Shaft speed follows the mechanical system's torque balance.

Field weakening requires negative d-axis current to reduce stator flux. With the
gates blocked, active control cannot maintain that current; the antiparallel
diodes can conduct and return energy to the fixed DC source. Passive currents
can exceed the active controller's current limit. Flying-start synchronizes the
observer and regulators, primes the PWM delay, and ramps torque back in while
the reference generator restores field weakening.

"""

# %%
from math import pi, sqrt

import matplotlib.pyplot as plt
import numpy as np

import motulator.drive.control.sm as control
from motulator.common.model import SolverCfg
from motulator.drive import model, utils

# %%
# Compute base values from the nominal machine ratings. The rated mechanical
# speed is about 157 rad/s; the high-speed reference is about 314 rad/s.

nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = utils.BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the machine, mechanical inertia, and fixed DC-bus voltage.

par = model.SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# The existing reference generator automatically transitions from MTPA to field
# weakening. Reserve voltage headroom with k_u, and limit peak stator current.

cfg = control.CurrentVectorControllerCfg(i_s_max=10, k_u=0.9, sensorless=False)
ctrl = control.VectorControlSystem(
    control.CurrentVectorController(par, cfg),
    speed_ctrl=control.SpeedController(J=mechanics.J, alpha_s=2 * pi * 4),
    flying_start=control.FlyingStartCfg(T_ramp=0.05),
)

# %%
# Accelerate to 2 p.u. and keep the speed reference there. Block the gates for
# 20 ms after reaching field weakening. This short interval allows passive
# braking while keeping the shaft above base speed at re-enabling.

ctrl.set_speed_ref(
    lambda t: base.w_M * np.interp(t, [0, 0.1, 0.3, 0.5, 1.1], [0, 0, 0.5, 0.5, 2])
)
mechanics.set_external_load_torque(lambda t: 0.25 * nom.tau * (t >= 0.2))
t_disable, t_enable = 1.40, 1.42
ctrl.set_enable(lambda t: t < t_disable or t >= t_enable)

# %%
# Simulate and plot speed and torque in per-unit values.

# Resolve zero-current diode transitions while the gates are blocked.
solver_cfg = SolverCfg(rtol=1e-8, atol=1e-10)
res = model.Simulation(mdl, ctrl, cfg=solver_cfg).simulate(t_stop=2.0)
utils.plot(res, base, subplots=["speed", "torque"])

# %%
# Zoom into disabling and flying-start in the field-weakening region. Plot the
# actual terminal voltage, including passive diode operation, and the DC current:
# negative DC current feeds the DC source. Voltage and current reference limits
# apply to active control and cannot constrain passive diode currents.

t, t_ctrl = res.mdl.t, res.ctrl.t
fig, axes = plt.subplots(6, 1, sharex=True, figsize=(8, 12), layout="constrained")
axes[0].step(t_ctrl, res.ctrl.ref.enabled, where="post", label="Gates enabled")
axes[0].step(
    t_ctrl,
    res.ctrl.ref.flying_start,
    where="post",
    linestyle="--",
    label="Flying-start",
)
axes[0].set_ylabel("State")
axes[0].set_yticks([0, 1])
axes[1].plot(t, res.mdl.machine.i_s_dq.real, label=r"$i_\mathrm{d}$")
axes[1].plot(t, res.mdl.machine.i_s_dq.imag, label=r"$i_\mathrm{q}$")
axes[1].plot(t_ctrl, res.ctrl.ref.i_s.real, "--", label=r"$i_\mathrm{d}^\mathrm{ref}$")
axes[1].set_ylabel("Current (A)")
axes[2].plot(t, np.abs(res.mdl.machine.psi_s_dq), label="Stator flux")
axes[2].plot(t_ctrl, res.ctrl.ref.psi_s, "--", label="Flux reference")
axes[2].axhline(par.psi_f, linestyle=":", label="PM flux")
axes[2].set_ylabel("Flux linkage (Vs)")
axes[3].plot(t, np.abs(res.mdl.converter.u_c_ab), label="Terminal voltage magnitude")
axes[3].axhline(cfg.k_u * 540 / sqrt(3), linestyle="--", label="Active voltage target")
axes[3].axhline(540 / sqrt(3), linestyle=":", label="Linear PWM limit")
axes[3].set_ylabel("Voltage (V)")
axes[4].plot(t, np.abs(res.mdl.machine.i_s_dq), label="Stator current magnitude")
axes[4].axhline(cfg.i_s_max, linestyle="--", label="Active current reference limit")
axes[4].set_ylabel("Current (A)")
axes[5].plot(t, res.mdl.converter.i_dc_int, label="Converter DC current")
axes[5].set_ylabel("DC current (A)")
axes[5].set_xlabel("Time (s)")
axes[5].set_xlim(t_disable - 0.1, t_enable + 0.2)
for ax in axes:
    ax.axvspan(t_disable, t_enable, color="gray", alpha=0.15)
    ax.grid()
    ax.legend()
plt.show()
