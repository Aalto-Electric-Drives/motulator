"""
2.2-kW PMSM
===========

This example simulates sensorless vector control of a 2.2-kW PMSM drive. Square-wave
signal injection is used with a simple phase-locked loop.

"""

# %%
import matplotlib.pyplot as plt
import numpy as np

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = utils.BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the system model.

par = model.SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

est_par = par  # Assume accurate model parameter estimates
cfg = control.CurrentVectorControllerCfg(i_s_max=2 * base.i)
vector_ctrl = control.SignalInjectionController(est_par, cfg)
speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * np.pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)


# %%
# Set the speed reference and the external load torque.

t_stop = 4
times = np.array([0, 0.25, 0.25, 0.375, 0.5, 0.625, 0.75, 0.75, 1]) * t_stop
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0]) * 0.1 * base.w_M
ctrl.set_speed_ref(utils.SequenceGenerator(times, values))

times = np.array([0, 0.125, 0.125, 0.875, 0.875, 1]) * t_stop
values = np.array([0, 0, 1, 1, 0, 0]) * nom.tau
mdl.mechanics.set_external_load_torque(utils.SequenceGenerator(times, values))

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop)
utils.plot(res, base)

# %%
# Plot also the angles.

plt.figure()
plt.plot(res.mdl.t, res.mdl.machine.theta_m, label=r"$\vartheta_\mathrm{m}$")
plt.plot(
    res.ctrl.t,
    res.ctrl.fbk.theta_m,
    ds="steps-post",
    label=r"$\hat \vartheta_\mathrm{m}$",
)
plt.legend()
plt.xlim(0, 4)
plt.xlabel("Time (s)")
plt.ylabel("Electrical angle (rad)")
plt.show()
