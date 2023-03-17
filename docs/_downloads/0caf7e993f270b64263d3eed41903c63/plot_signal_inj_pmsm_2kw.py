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
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14, P_nom=2.2e3, p=3)

# %%
# Configure the system model.

motor = mt.SynchronousMotor(p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545)
mech = mt.Mechanics(J=.015)
conv = mt.Inverter()
mdl = mt.SynchronousMotorDrive(motor, mech, conv)

# %%
# Configure the control system.

pars = mt.SynchronousMotorSignalInjectionCtrlPars(
    T_s=250e-6,
    alpha_c=2*np.pi*100,
    alpha_s=2*np.pi*4,
    w_o=2*np.pi*40,
    U_inj=250,
    L_d=.036,
    L_q=.051,
    psi_f=.545,
    i_s_max=2*base.i,
    tau_M_max=2*base.tau_nom)
ctrl = mt.SynchronousMotorSignalInjectionCtrl(pars)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .25, .25, .375, .5, .625, .75, .75, 1])*4
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w*.1
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

# Plot the "basic" figure
mt.plot(sim, base=base)

# Plot also the angles
mdl = sim.mdl.data  # Continuous-time data
ctrl = sim.ctrl.data  # Discrete-time data
plt.figure()
plt.plot(mdl.t, mdl.theta_m, label=r'$\vartheta_\mathrm{m}$')
plt.step(
    ctrl.t, ctrl.theta_m, where='post', label=r'$\hat \vartheta_\mathrm{m}$')
plt.legend()
plt.xlim(0, 4)
plt.xlabel('Time (s)')
plt.ylabel('Electrical angle (rad)')
plt.show()
