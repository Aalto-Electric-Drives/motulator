"""
2.2-kW PMSM
===========

This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive.

"""
# %%

import numpy as np

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values (just for figures).

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
cfg = control.ObserverBasedVHzControllerCfg(i_s_max=1.5 * base.i)
vhz_ctrl = control.ObserverBasedVHzController(est_par, cfg)
ctrl = control.VHzControlSystem(vhz_ctrl)

# %%
# Set the speed reference and the external load torque.

t_stop = 8
times = np.array([0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1]) * t_stop
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0]) * base.w_M
ctrl.set_speed_ref(utils.SequenceGenerator(times, values))

times = np.array([0, 0.125, 0.125, 0.875, 0.875, 1]) * t_stop
values = np.array([0, 0, 1, 1, 0, 0]) * nom.tau
mdl.mechanics.set_external_load_torque(utils.SequenceGenerator(times, values))

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop)
utils.plot(res, base)
