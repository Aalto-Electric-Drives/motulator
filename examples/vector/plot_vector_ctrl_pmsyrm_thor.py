"""
5-kW PM-SyRM
============

This example simulates sensorless vector control of a 5-kW permanent-magnet
synchronous reluctance motor. Control look-up tables are also plotted.

"""

# %%
# Imports.

import numpy as np
from motulator import model, control
from motulator import BaseValues, plot

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=220, I_nom=15.6, f_nom=85, tau_nom=19, P_nom=5.07e3, n_p=2)

# %%
# Configure the system model.

# Configure magnetically linear motor model
machine = model.sm.SynchronousMachine(
    n_p=2, R_s=.2, L_d=4e-3, L_q=17e-3, psi_f=.134)
mechanics = model.Mechanics(J=.0042)
converter = model.Inverter(u_dc=310)
mdl = model.sm.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

par = control.sm.ModelPars(
    n_p=2, R_s=.2, L_d=4e-3, L_q=17e-3, psi_f=.134, J=.0042)
ref = control.sm.CurrentReferencePars(
    par, w_m_nom=base.w, i_s_max=2*base.i, k_u=.9)
ctrl = control.sm.VectorCtrl(par, ref, T_s=125e-6, sensorless=True)
ctrl.observer = control.sm.Observer(par, alpha_o=2*np.pi*200)
ctrl.speed_ctrl = control.SpeedCtrl(
    J=par.J, alpha_s=2*np.pi*4, tau_M_max=1.5*base.tau_nom)

# %%
# Plot control characteristics, computed using constant L_d, L_q, and psi_f.

# sphinx_gallery_thumbnail_number = 1
tq = control.sm.TorqueCharacteristics(par)
tq.plot_current_loci(ctrl.current_ref.i_s_max, base)
tq.plot_torque_flux(ctrl.current_ref.i_s_max, base)
tq.plot_torque_current(ctrl.current_ref.i_s_max, base)
# tq.plot_flux_loci(ctrl.current_ref.i_s_max, base)

# %%
# Set the speed reference and the external load torque.

# Acceleration and load torque step
ctrl.w_m_ref = lambda t: (t > .1)*base.w*3
# Quadratic load torque profile
k = .05*base.tau_nom/(base.w/base.n_p)**2
mdl.mechanics.tau_L_w = lambda w_M: k*w_M**2*np.sign(w_M)

# %%
# Create the simulation object, simulate, and plot results in per-unit values.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=.6)
plot(sim, base)
