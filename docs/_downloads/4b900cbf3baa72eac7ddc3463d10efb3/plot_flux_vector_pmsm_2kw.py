"""
2.2-kW PMSM
===========

This example simulates sensorless flux-vector control of a 2.2-kW PMSM drive.

"""
# %%

from motulator import model, control
from motulator import BaseValues, NominalValues, plot

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

machine = model.SynchronousMachine(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.Drive(converter, machine, mechanics)

# %%
# Configure the control system.

par = control.sm.ModelPars(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545, J=.015)
cfg = control.sm.FluxTorqueReferenceCfg(par, max_i_s=1.5*base.i, k_u=.9)
ctrl = control.sm.FluxVectorCtrl(par, cfg, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference (electrical rad/s)
ctrl.ref.w_m = lambda t: (t > .2)*2*base.w

# Load torque step
mdl.mechanics.tau_L_t = lambda t: (t > .8)*nom.tau*.7

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1.6)

# %%
# Plot results in per-unit values.

plot(sim, base)
