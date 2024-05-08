"""
2.2-kW induction motor, LC filter
=================================

This example simulates open-loop V/Hz control of a 2.2-kW induction machine
drive equipped with an LC filter. 

"""
# %%
# Imports.

import numpy as np
import matplotlib.pyplot as plt
from motulator import model, control
from motulator import BaseValues, plot

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, n_p=2)

# %%
# Create the system model. The filter parameters correspond to [#Sal2006]_.

machine = model.im.InductionMachineInvGamma(  # Inverse-Γ parameters
    R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
lc_filter = model.LCFilter(L=8e-3, C=9.9e-6, R=.1)
mdl = model.im.DriveWithLCFilter(machine, mechanics, converter, lc_filter)
mdl.pwm = model.CarrierComparison()  # Enable the PWM model

# %%
# Control system (parametrized as open-loop V/Hz control).

# Inverse-Γ model parameter estimates
par = control.im.ModelPars(R_s=0*3.7, R_R=0*2.1, L_sgm=.021, L_M=.224)
ctrl = control.im.VHzCtrl(250e-6, par, psi_s_nom=base.psi, k_u=0, k_w=0)
ctrl.rate_limiter = control.RateLimiter(2*np.pi*120)

# %%
# Set the speed reference and the external load torque.

ctrl.w_m_ref = lambda t: (t > .2)*base.w

# Quadratic load torque profile (corresponding to pumps and fans)
k = 1.1*base.tau_nom/(base.w/base.n_p)**2
mdl.mechanics.tau_L_w = lambda w_M: k*w_M**2*np.sign(w_M)

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=1.5)

# %%
# Plot results in per-unit values.

# sphinx_gallery_thumbnail_number = 2
plot(sim, base)

# %%
# Plot additional waveforms.

t_span = (1.1, 1.125)  # Time span for the zoomed-in plot
mdl = sim.mdl.data  # Continuous-time data
# Plot the converter and stator voltages (phase a)
fig1, (ax1, ax2) = plt.subplots(2, 1)
ax1.plot(mdl.t, mdl.u_cs.real/base.u, label=r"$u_\mathrm{ca}$")
ax1.plot(mdl.t, mdl.u_ss.real/base.u, label=r"$u_\mathrm{sa}$")
ax1.set_xlim(t_span)
ax1.legend()
ax1.set_xticklabels([])
ax1.set_ylabel("Voltage (p.u.)")
# Plot the converter and stator currents (phase a)
ax2.plot(mdl.t, mdl.i_cs.real/base.i, label=r"$i_\mathrm{ca}$")
ax2.plot(mdl.t, mdl.i_ss.real/base.i, label=r"$i_\mathrm{sa}$")
ax2.set_xlim(t_span)
ax2.legend()
ax2.set_ylabel("Current (p.u.)")
ax2.set_xlabel("Time (s)")

plt.tight_layout()
plt.show()

# %%
# .. rubric:: References
#
# .. [#Sal2006] Salomäki, Hinkkanen, Luomi, "Sensorless control of induction
#    motor drives equipped with inverter output filter," IEEE Trans. Ind.
#    Electron., 2006, https://doi.org/10.1109/TIE.2006.878314
