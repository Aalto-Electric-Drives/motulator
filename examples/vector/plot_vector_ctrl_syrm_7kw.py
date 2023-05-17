"""
Vector control: 6.7-kW SyRM
===========================

This example simulates sensorless vector control of a 6.7-kW SyRM drive.

"""

# %%
# Import the packages.

import numpy as np
import motulator.model.sm as model
import motulator.control.sm as control
from motulator.helpers import BaseValues, Sequence
from motulator.plots import plot

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=370, I_nom=15.5, f_nom=105.8, tau_nom=20.1, P_nom=6.7e3, n_p=2)

# %%
# Configure the system model.

machine = model.SynchronousMachine(
    n_p=2, R_s=.54, L_d=41.5e-3, L_q=6.2e-3, psi_f=0)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system. You may also try to change the parameters.

par = control.ModelPars(
    n_p=2, R_s=.54, L_d=41.5e-3, L_q=6.2e-3, psi_f=0, J=.015)
ref = control.CurrentReferencePars(
    par, w_m_nom=base.w, i_s_max=1.5*base.i, psi_s_min=.5*base.psi, k_u=.9)
ctrl = control.VectorCtrl(par, ref, T_s=125e-6, sensorless=True)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.w_m_ref = Sequence(times, values)
# External load torque
times = np.array([0, .125, .125, .875, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, 0])*base.tau_nom
mdl.mechanics.tau_L_t = Sequence(times, values)

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl, pwm=False)
sim.simulate(t_stop=4)

# %%
# Plot results in per-unit values.

plot(sim, base)
