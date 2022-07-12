"""
Vector-Controlled 6.7-kW SyRM Drive
===================================

This example simulates sensorless vector control of a 6.7-kW SyRM drive.

"""

# %%
# Import the packages.

from time import time
import numpy as np
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=370, I_nom=15.5, f_nom=105.8, tau_nom=20.1, P_nom=6.7e3, p=2)

# %%
# Configure the system model.

motor = mt.SynchronousMotor(p=2, R_s=.54, L_d=41.5e-3, L_q=6.2e-3, psi_f=0)
mech = mt.Mechanics(J=.015)
conv = mt.Inverter()
mdl = mt.SynchronousMotorDrive(motor, mech, conv)

# %%
# Configure the control system. You may also try to change the parameters.

pars = mt.SynchronousMotorVectorCtrlPars(
    sensorless=True,
    T_s=250e-6,
    alpha_c=2*np.pi*200, alpha_fw=2*np.pi*20, alpha_s=2*np.pi*4,
    w_o=2*np.pi*80,  # Used only in the sensorless mode
    tau_M_max=2*base.tau_nom,
    i_s_max=2*base.i,
    i_sd_min=.25*base.i,  # Can be 0 in the sensored mode
    k_u=.95, w_nom=2*np.pi*105.8,
    p=2, R_s=.54, L_d=41.5e-3, L_q=6.2e-3, psi_f=0,
    J=.015)
# pars.plot(base)  # Plot control look-up tables
ctrl = mt.SynchronousMotorVectorCtrl(pars)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*4
values = np.array([0,  0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.w_m_ref = mt.Sequence(times, values)
# External load torque
times = np.array([0, .125, .125, .875, .875, 1])*4
values = np.array([0, 0, 1, 1, 0, 0])*base.tau_nom
mdl.mech.tau_L_ext = mt.Sequence(times, values)
# Print the system model and controller parameters
print(str(mdl)+'\n\n'+str(ctrl))

# %%
# Create the simulation object and simulate it.

sim = mt.Simulation(mdl, ctrl, base=base, enable_pwm=False, t_stop=4)
start_time = time()  # Start the timer
sim.simulate()
# Print the execution time
print('\nExecution time: {:.2f} s'.format((time() - start_time)))

# %%
# Plot results in per-unit values.

mt.plot_pu(sim)
# mt.plot(sim)
