"""
2.2-kW induction motor
======================

This example simulates sensorless flux-vector control of a 2.2-kW induction
machine drive.

"""

# %%
import numpy as np

from motulator.common.utils import Sequence
from motulator.drive import model
import motulator.drive.control.im as control
from motulator.drive.utils import (
    BaseValues,
    NominalValues,
    InductionMachineInvGammaPars,
    InductionMachinePars,
    plot,
)

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

# Unsaturated machine model, using its inverse-Γ parameters
par = InductionMachineInvGammaPars(n_p=2, R_s=3.7, R_R=2.1, L_sgm=0.021, L_M=0.224)
mdl_par = InductionMachinePars.from_inv_gamma_model_pars(par)
machine = model.InductionMachine(mdl_par)
mechanics = model.StiffMechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(converter, machine, mechanics)

# %%
# Configure the control system.

# Set nominal values and limits for reference generation
cfg = control.FluxVectorControlCfg(0.95 * base.psi, 1.5 * base.i, 1.5 * nom.tau)
ctrl = control.FluxVectorControl(par, cfg, J=0.015, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference (electrical rad/s)
times = np.array([0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1]) * 4
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0]) * base.w
ctrl.ref.w_m = Sequence(times, values)
# External load torque
times = np.array([0, 0.125, 0.125, 0.875, 0.875, 1]) * 4
values = np.array([0, 0, 1, 1, 0, 0]) * nom.tau
mdl.mechanics.tau_L = Sequence(times, values)

# No load, field-weakening (uncomment to try)
# ctrl.ref.w_m = lambda t: (t > .2)*2*base.w
# mdl.mechanics.tau_L = lambda t: 0

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=4)

# %%
# Plot results in per-unit values.

plot(sim, base)
