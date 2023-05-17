"""
Vector control with signal injection: 2.2-kW PMSM
=================================================

This example simulates sensorless vector control of a 2.2-kW PMSM drive.
Square-wave signal injection is used with a simple phase-locked loop.

"""

# %%
# Import the packages.

import numpy as np
import matplotlib.pyplot as plt
import motulator.model.sm as model
import motulator.control.sm as control
from motulator.helpers import BaseValues, Sequence
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
ref = control.CurrentReferencePars(par, w_m_nom=base.w, i_s_max=2*base.i)
ctrl = control.SignalInjectionCtrl(par, ref, T_s=250e-6)
#ctrl.current_ctrl = control.CurrentCtrl(par, 2*np.pi*100)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .25, .25, .375, .5, .625, .75, .75, 1])*4
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w*.1
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

# Plot the "basic" figure
plot(sim, base)

# Plot also the angles
mdl = sim.mdl.data  # Continuous-time data
ctrl = sim.ctrl.data  # Discrete-time data
plt.figure()
plt.plot(mdl.t, mdl.theta_m, label=r"$\vartheta_\mathrm{m}$")
plt.step(
    ctrl.t, ctrl.theta_m, where='post', label=r"$\hat \vartheta_\mathrm{m}$")
plt.legend()
plt.xlim(0, 4)
plt.xlabel("Time (s)")
plt.ylabel("Electrical angle (rad)")
plt.show()
