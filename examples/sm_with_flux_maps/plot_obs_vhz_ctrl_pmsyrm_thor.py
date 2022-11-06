"""
Observer-based V/Hz control: 7.5-kW PM-SyRM
===========================================

This example simulates observer-based V/Hz control of a saturated 7.5-kW
permanent-magnet synchronous reluctance motor. This example machine is known as
THOR, and its flux maps are from the SyR-e project:

    https://github.com/SyR-e/syre_public

The THOR data has been licensed under the Apache License, Version 2.0. The flux
maps from other sources can be used in the similar manner. To study the flux
maps in more detail, see also the module `sm_flux_maps`. It is worth noticing
that the saturation is not taken into account in the control method, only in
the system model. Naturally, the control performance can be improved by taking
the saturation into account in the control algorithm.

"""

# %%
# Import the package.

import numpy as np
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=284, I_nom=18, f_nom=100, tau_nom=23.8, P_nom=7.5e3, p=2)

# %%
# # Load and plot the flux maps.

data = mt.import_syre_data(fname='THOR.mat')
mt.plot_flux_vs_current(data)
mt.plot_flux_map(data)

# %%
# Configure the system model.

# Create the motor model
motor = mt.SynchronousMotorSaturatedLUT(
    p=2, R_s=.23, psi_s_data=data.psi_s.ravel(), i_s_data=data.i_s.ravel())
# Magnetically linear PM-SyRM model
# motor = mt.SynchronousMotor(p=2, R_s=.23, L_d=4.2e-3, L_q=17.3e-3, psi_f=.17)
mech = mt.Mechanics(J=.015)
conv = mt.Inverter(u_dc=400)
mdl = mt.SynchronousMotorDrive(motor, mech, conv)

# %%
# Configure the control system.

pars = mt.SynchronousMotorVHzObsCtrlPars(
    p=2,
    R_s=.23,
    L_d=4.2e-3,
    L_q=17.3e-3,
    psi_f=.17,
    alpha_psi=2*np.pi*50,
    zeta_inf=.1,
    T_s=250e-6,
    rate_limit=2*np.pi*120*10,
    i_s_max=2*base.i,
    psi_s_min=.5*base.psi,
    psi_s_max=1.5*base.psi,
)
ctrl = mt.SynchronousMotorVHzObsCtrl(pars)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .125, .25, .375, .5, .625, .75, .875, 1])*8
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0])*base.w
ctrl.w_m_ref = mt.Sequence(times, values)

# Quadratic load torque profile (corresponding to pumps and fans)
k = base.tau_nom/(base.w/base.p)**2
mdl.mech.tau_L_w = lambda w_M: k*w_M**2*np.sign(w_M)

# Rated load torque step at t = 2.5 s (set k = 0 above)
# mdl.mech.tau_L_t = lambda t: (t > 2.5)*base.tau_nom

# %%
# Create the simulation object and simulate it. You can also enable the PWM
# model (which makes simulation slower). One-sampling-period computational
# delay is modeled.

sim = mt.Simulation(mdl, ctrl, pwm=False, delay=1)
sim.simulate(t_stop=8)

# %%
# Plot results in per-unit values. By omitting the argument `base` you can plot
# the results in SI units.

mt.plot(sim, base=base)
