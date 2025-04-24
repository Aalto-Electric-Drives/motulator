"""
6.7-kW SyRM
===========

This example simulates sensorless current-vector control of a 6.7-kW SyRM drive.

"""
# %%

from math import pi

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values (just for figures).

nom = utils.NominalValues(U=370, I=15.5, f=105.8, P=6.7e3, tau=20.1)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

par = model.SynchronousMachinePars(
    n_p=2, R_s=0.54, L_d=41.5e-3, L_q=6.2e-3, psi_f=0, kind="rel"
)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

est_par = par  # Assume accurate model parameter estimates
cfg = control.CurrentVectorControllerCfg(
    i_s_max=1.5 * base.i, psi_s_min=0.5 * base.psi, psi_s_max=1.5 * base.psi
)
vector_ctrl = control.CurrentVectorController(est_par, cfg)
speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

# %%
# Set the speed reference and the external load torque.

ctrl.set_speed_ref(lambda t: (t > 0.2) * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 0.6) * nom.tau)

# %%
# Create the simulation object, simulate, and plot results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1)
utils.plot(res, base)
