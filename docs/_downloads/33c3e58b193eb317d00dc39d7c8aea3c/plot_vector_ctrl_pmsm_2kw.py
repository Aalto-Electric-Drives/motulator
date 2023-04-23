"""
Vector control: 2.2-kW PMSM
===========================

This example simulates sensorless vector control of a 2.2-kW PMSM drive.

"""

# %%
# Import the packages.

import numpy as np
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14, P_nom=2.2e3, n_p=3)

# %%
# Configure the system model.

motor = mt.SynchronousMotor(n_p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
mech = mt.Mechanics(J=.015)
conv = mt.Inverter(u_dc=540)
mdl = mt.SynchronousMotorDrive(motor, mech, conv)

# %%
# Configure the control system.

pars = mt.SynchronousMotorVectorCtrlPars(sensorless=True)
ctrl = mt.SynchronousMotorVectorCtrl(pars)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.w_m_ref = mt.Sequence(times, values)
# External load torque
times = np.array([0, .125, .125, .875, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, 0])*base.tau_nom
mdl.mech.tau_L_t = mt.Sequence(times, values)

# %%
# Create the simulation object and simulate it.

sim = mt.Simulation(mdl, ctrl, pwm=False)
sim.simulate(t_stop=4)

# %%
# Plot results in per-unit values.

mt.plot(sim, base=base)
