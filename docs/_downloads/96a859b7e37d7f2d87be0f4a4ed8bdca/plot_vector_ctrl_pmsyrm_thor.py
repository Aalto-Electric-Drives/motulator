"""
Vector control: 5-kW PM-SyRM
============================

This example simulates sensorless vector control of a 5-kW permanent-magnet
synchronous reluctance motor. Control look-up tables are also plotted.

"""

# %%
# Import the packages.

import numpy as np
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=220, I_nom=15.6, f_nom=85, tau_nom=19, P_nom=5.07e3, p=2)

# %%
# Configure the system model.

# Configure magnetically linear motor model
motor = mt.SynchronousMotor(p=2, R_s=.2, L_d=4e-3, L_q=17e-3, psi_f=.134)
mech = mt.Mechanics(J=.0042)
conv = mt.Inverter(u_dc=310)
mdl = mt.SynchronousMotorDrive(motor, mech, conv)

# %%
# Configure the control system.

pars = mt.SynchronousMotorVectorCtrlPars(
    sensorless=True,
    p=2,
    R_s=.2,
    L_d=4e-3,
    L_q=17e-3,
    psi_f=.134,
    J=.0042,
    tau_M_max=2*base.tau_nom,  # Maximum torque
    i_s_max=2*base.i,  # Maximum current
    T_s=125e-6,  # Sampling period
    k_u=.9,  # Voltage margin
    w_nom=base.w,  # Nominal speed
    w_o=2*np.pi*200,  # Observer bandwidth
    alpha_c=2*np.pi*200,  # Current control bandwidth
    alpha_fw=2*np.pi*20,  # Field-weakening bandwidth
    alpha_s=2*np.pi*4,  # Speed control bandwidth
)
ctrl = mt.SynchronousMotorVectorCtrl(pars)

# %%
# Plot control characteristics, computed using constant L_d, L_q, and psi_f.

# sphinx_gallery_thumbnail_number = 1
tq = mt.TorqueCharacteristics(pars)
tq.plot_current_loci(pars.i_s_max, base)
tq.plot_torque_flux(pars.i_s_max, base)
tq.plot_torque_current(pars.i_s_max, base)
# tq.plot_flux_loci(pars.i_s_max, base)

# %%
# Set the speed reference and the external load torque.

# Acceleration and load torque step
ctrl.w_m_ref = lambda t: (t > .1)*base.w*3
# Quadratic load torque profile
k = .05*base.tau_nom/(base.w/base.p)**2
mdl.mech.tau_L_w = lambda w_M: k*w_M**2*np.sign(w_M)

# %%
# Create the simulation object and simulate it.

sim = mt.Simulation(mdl, ctrl, pwm=False)
sim.simulate(t_stop=.6)

# %%
# Plot results in per-unit values.

mt.plot(sim, base=base)
