"""
2.2-kW PMSM
===========

This example simulates sensorless vector control of a 2.2-kW PMSM drive.

"""

# %%
# Imports.

import numpy as np
from motulator import model, control
from motulator import BaseValues, Sequence, plot, plot_extra

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14, P_nom=2.2e3, n_p=3)

# %%
# Configure the system model.

machine = model.sm.SynchronousMachine(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.sm.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

par = control.sm.ModelPars(
    n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545, J=.015)
ref = control.sm.CurrentReferencePars(par, w_m_nom=base.w, i_s_max=1.5*base.i)
ctrl = control.sm.VectorCtrl(par, ref, T_s=250e-6, sensorless=True)

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

# mdl.mechanics.tau_L_t = lambda t: (t > .8)*base.tau_nom*.7
# ctrl.w_m_ref = lambda t: (t > .2)*(2*base.w)

# %%
# Create the simulation object and simulate it.

# Simulate the system without modeling PWM
sim = model.Simulation(mdl, ctrl, pwm=False)
sim.simulate(t_stop=4)
plot(sim, base)  # Plot results in per-unit values.

# Repeat the same simulation with PWM model enabled (takes a bit longer)
mdl.clear()  # First clear the stored data from the previous simulation run
ctrl.clear()
sim = model.Simulation(mdl, ctrl, pwm=True)
sim.simulate(t_stop=4)
plot(sim, base)
# Plot a zoomed view
plot_extra(sim, t_span=(1.1, 1.125), base=base)
