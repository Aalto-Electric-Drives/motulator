"""
2.2-kW induction motor
======================

This example simulates observer-based V/Hz control of a 2.2-kW induction motor
drive.

"""
# %%

import numpy as np

from motulator.drive import model
import motulator.drive.control.im as control
from motulator.drive.utils import (
    BaseValues, InductionMachinePars, InductionMachineInvGammaPars,
    NominalValues, plot, Sequence)

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

# Configure the induction machine using its inverse-Γ parameters
par = InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224)
mdl_par = InductionMachinePars.from_inv_gamma_model_pars(par)
machine = model.InductionMachine(mdl_par)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.Drive(converter, machine, mechanics)

# %%
# Configure the control system.

# Inverse-Γ model parameter estimates
par = control.ModelPars(R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
cfg = control.ObserverBasedVHzCtrlCfg(
    nom_psi_s=base.psi, max_i_s=1.5*base.i, slip_compensation=False)
ctrl = control.ObserverBasedVHzCtrl(par, cfg, T_s=250e-6)

# %%
# Set the speed reference.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.ref.w_m = Sequence(times, values)

# %%
# Set the load torque reference.

# External load torque
times = np.array([0, .125, .125, .875, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, 0])*nom.tau
mdl.mechanics.tau_L_t = Sequence(times, values)

# Quadratic load torque profile, e.g. pumps and fans (uncomment to enable)
# k = 1.1*base.tau_nom/(base.w/base.n_p)**2
# mdl.mechanics.tau_L_w = lambda w_M: np.sign(w_M)*k*w_M**2

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=4)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

plot(sim, base)
