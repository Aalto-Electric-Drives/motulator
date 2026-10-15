"""
IM drive enable and sensored flying-start
=========================================

This example follows the same operating sequence as the IPMSM drive-enable
example: accelerate from rest, block the gates and coast, then resume speed
control with sensored flying-start. The mechanical system determines the shaft
speed from electromagnetic and friction torques.

After gate blocking, the stator currents freewheel through the antiparallel
diodes and decay to zero. The remaining rotor flux also decays while the shaft
coasts. Flying-start synchronizes the measured shaft speed, preserves the
observer's residual flux estimate, primes the PWM delay, and ramps in torque.
The induction motor rebuilds its flux as active control resumes.

"""

# %%
import matplotlib.pyplot as plt

import motulator.drive.control.im as control
from motulator.common.model import SolverCfg
from motulator.drive import model, utils

# %%
# Configure the 2.2-kW induction machine, shaft inertia and friction, and constant
# DC source. Use the same mechanical system as in the IPMSM drive-enable example.

par = model.InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224
)
machine = model.InductionMachine(par)
mechanics = model.MechanicalSystem(J=0.015, B_L=0.02)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure sensored current-vector control, the speed controller, and flying-start.
# Rotor speed comes from the mechanical sensor interface. The residual rotor flux
# is tracked by the observer rather than inferred from mechanical position.

cfg = control.CurrentVectorControllerCfg(psi_s_nom=1.04, i_s_max=10.6, sensorless=False)
ctrl = control.VectorControlSystem(
    control.CurrentVectorController(par, cfg),
    speed_ctrl=control.SpeedController(J=mechanics.J, alpha_s=20),
    flying_start=control.FlyingStartCfg(T_ramp=0.05),
)

# %%
# Use the same speed reference and enable schedule as in the IPMSM example.
# Power regulators stop integrating while blocked, but the observers keep running.
# Either ctrl.set_enable(...) or mdl.set_enable(...) can block the gates.

ctrl.set_speed_ref(100)
ctrl.set_enable(lambda t: t < 0.80 or t >= 1.05)

# %%
# Simulate and plot the mechanical and electrical responses. Tight solver
# tolerances resolve the passive diode zero-current boundary layer.

solver_cfg = SolverCfg(rtol=1e-8, atol=1e-10)
res = model.Simulation(mdl, ctrl, cfg=solver_cfg).simulate(t_stop=1.60)
utils.plot(res)
utils.plot_stator_waveforms(res)

# %%
# Show effective gate enable, flying-start status, and the torque reference passed
# to the inner controller. Re-enabling at 1.05 s includes PWM priming followed by
# the 0.05-s torque ramp, as in the IPMSM example.

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
