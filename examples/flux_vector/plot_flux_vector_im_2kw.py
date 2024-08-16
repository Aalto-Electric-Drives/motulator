"""
2.2-kW PMSM
===========

This example simulates sensorless flux-vector control of a 2.2-kW PMSM drive.

"""
# %%
import numpy as np
from motulator.common.utils import Sequence
from motulator.drive import model
import motulator.drive.control.im as control
from motulator.drive.utils import (
    BaseValues, NominalValues, plot, InductionMachineInvGammaPars, InductionMachinePars)

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

# Unsaturated machine model, using its inverse-Γ parameters
par = InductionMachineInvGammaPars(
    n_p=2, R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224)
mdl_par = InductionMachinePars.from_inv_gamma_model_pars(par)
machine = model.InductionMachine(mdl_par)
mechanics = model.StiffMechanicalSystem(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.Drive(converter, machine, mechanics)

# %%
# Configure the control system.
# Set nominal values and limits for reference generation
cfg = control.FluxVectorControlCfg(base.u, base.w, base.tau)
ctrl = control.FluxVectorControl(par, cfg, J=.015, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference (electrical rad/s)
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.ref.w_m = Sequence(times, values)
# External load torque
times = np.array([0, .125, .125, .875, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, 0])*nom.tau
mdl.mechanics.tau_L = Sequence(times, values)
# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=4)

# %%
# Plot results in per-unit values.
plot(sim, base)
