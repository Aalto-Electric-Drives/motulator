"""
2.2-kW PMSM
===========

This example simulates sensorless flux-vector control of a 2.2-kW PMSM drive.

"""

# %%

from motulator.drive import model
import motulator.drive.control.sm as control
from motulator.drive.utils import (
    BaseValues,
    NominalValues,
    plot,
    SynchronousMachinePars,
)

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model.

mdl_par = SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
machine = model.SynchronousMachine(mdl_par)
mechanics = model.StiffMechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(converter, machine, mechanics)

# %%
# Configure the control system.

par = mdl_par  # Assume accurate machine model parameter estimates
cfg = control.FluxTorqueReferenceCfg(par, max_i_s=1.5 * base.i, k_u=0.9)
ctrl = control.FluxVectorControl(par, cfg, J=0.015, T_s=250e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference (electrical rad/s)
ctrl.ref.w_m = lambda t: (t > 0.2) * 2 * base.w

# Load torque step
mdl.mechanics.tau_L = lambda t: (t > 0.8) * nom.tau * 0.7

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1.6)

# %%
# Plot results in per-unit values.

plot(sim, base)
