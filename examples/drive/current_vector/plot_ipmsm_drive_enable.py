"""
IPMSM drive enable and sensored flying-start
==========================================

The control gate-enable command is defined in the same way as an external reference: a
constant or a function of simulation time. The speed controller accelerates the
mechanical system from rest. With the gates blocked, stored kinetic energy feeds the DC
source through the antiparallel diodes. As the shaft slows, the diodes stop
conducting and the machine operates in open circuit. Sensored flying-start then
synchronizes the observer and regulators before re-enabling the gates and ramping
in the torque command.

"""

# %%
import matplotlib.pyplot as plt

import motulator.drive.control.sm as control
from motulator.common.model import SolverCfg
from motulator.drive import model, utils

# %%
# Configure the machine, shaft inertia and friction, and constant DC source. The
# shaft speed is determined by electromagnetic and friction torques.

par = model.SynchronousMachinePars(3, 3.6, 0.036, 0.051, 0.545)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.015, B_L=0.02)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

cfg = control.CurrentVectorControllerCfg(i_s_max=10, sensorless=False)
ctrl = control.VectorControlSystem(
    control.CurrentVectorController(par, cfg),
    speed_ctrl=control.SpeedController(J=mechanics.J, alpha_s=20),
    flying_start=control.FlyingStartCfg(T_ramp=0.05),
)

# %%
# Define references and drive state. The state command bypasses the PWM delay and
# is evaluated at control sampling instants. Power regulators stop integrating
# while blocked. Flying-start uses measured shaft speed and position, primes the
# delayed PWM, initializes regulators, and ramps in torque at each takeover.
# The independent model-side command, mdl.set_enable(...), can also block gates.

# Accelerate before gate blocking, then resume speed control with flying-start.
# The speed controller's torque demand is ramped in on each enable.
ctrl.set_speed_ref(100)
ctrl.set_enable(lambda t: t < 0.80 or t >= 1.05)

# %%
# Simulate and show the enable state, machine terminal voltage and current, and
# DC-bus voltage and current. Negative converter DC current feeds the DC source.

# Resolve the zero-current diode boundary layer to avoid spurious commutations
# caused by integration error after the winding currents have decayed.
solver_cfg = SolverCfg(rtol=1e-8, atol=1e-10)
res = model.Simulation(mdl, ctrl, cfg=solver_cfg).simulate(t_stop=1.60)
utils.plot(res)
utils.plot_stator_waveforms(res)

# %%
# Show effective gate enable and flying-start status, including PWM priming with
# the gates still blocked. Show the ramped, limited torque reference passed to
# the inner controller. After the second enable at 1.05 s, torque takeover
# finishes at approximately 1.10 s.

fig, ax = plt.subplots(1, 1, sharex=True, figsize=(8, 5), layout="constrained")
ax.step(res.ctrl.t, res.ctrl.ref.enabled, where="post", label="Gates enabled")
ax.step(
    res.ctrl.t,
    res.ctrl.ref.flying_start,
    where="post",
    linestyle="--",
    label="Flying-start",
)
ax.set_ylabel("State")
ax.set_yticks([0, 1])
ax.set_xlabel("Time (s)")
ax.legend()
plt.show()
