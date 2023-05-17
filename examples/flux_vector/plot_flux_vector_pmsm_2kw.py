"""
Flux-vector control: 2.2-kW PMSM
================================

This example simulates sensorless stator-flux-vector control of a 2.2-kW PMSM
drive.

"""

# %%
# Import the package.

import motulator.model.sm as model
import motulator.control.sm as control
from motulator.helpers import BaseValues
from motulator.plots import plot

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14, P_nom=2.2e3, n_p=3)

# %%
# Configure the system model.

machine = model.SynchronousMachine(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

par = control.ModelPars(n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545, J=.015)
ref = control.FluxTorqueReferencePars(par, i_s_max=1.5*base.i, k_u=.9)
ctrl = control.FluxVectorCtrl(par, ref, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Simple acceleration and load torque step
ctrl.w_m_ref = lambda t: (t > .2)*(2*base.w)
mdl.mechanics.tau_L_t = lambda t: (t > .8)*base.tau_nom*.7

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl, pwm=False)
sim.simulate(t_stop=1.6)

# %%
# Plot results in per-unit values.

plot(sim, base)
