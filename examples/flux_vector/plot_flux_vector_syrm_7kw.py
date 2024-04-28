"""
6.7-kW SyRM, saturated, disturbance estimation
==============================================

This example simulates sensorless stator-flux-vector control of a saturated
6.7-kW synchronous reluctance motor drive. The saturation is not taken into
account in the control method (only in the system model). Even if the machine 
has no magnets, the PM-flux disturbance estimation is enabled [#Tuo2018]_. In 
this case, this PM-flux estimate lumps the effects of inductance errors. 
Naturally, the PM-flux estimation can be used in PM machine drives as well. 

"""

# %%
# Imports.

import numpy as np
import matplotlib.pyplot as plt
from motulator import model, control
from motulator import BaseValues, Sequence, plot

# %%
# Compute base values based on the nominal values (just for figures).

base = BaseValues(
    U_nom=370, I_nom=15.5, f_nom=105.8, tau_nom=20.1, P_nom=6.7e3, n_p=2)

# %%
# Create a saturation model, see the example
# :doc:`/auto_examples/obs_vhz/plot_obs_vhz_ctrl_syrm_7kw` for further details.


def i_s(psi_s):
    """Magnetic model for a 6.7-kW synchronous reluctance motor."""
    # Parameters
    a_d0, a_dd, S = 17.4, 373., 5  # d-axis self-saturation
    a_q0, a_qq, T = 52.1, 658., 1  # q-axis self-saturation
    a_dq, U, V = 1120., 1, 0  # Cross-saturation
    # Inverse inductance functions
    G_d = a_d0 + a_dd*np.abs(psi_s.real)**S + (
        a_dq/(V + 2)*np.abs(psi_s.real)**U*np.abs(psi_s.imag)**(V + 2))
    G_q = a_q0 + a_qq*np.abs(psi_s.imag)**T + (
        a_dq/(U + 2)*np.abs(psi_s.real)**(U + 2)*np.abs(psi_s.imag)**V)
    # Stator current
    return G_d*psi_s.real + 1j*G_q*psi_s.imag


# %%
# Configure the system model.

machine = model.sm.SynchronousMachineSaturated(n_p=2, R_s=.54, current=i_s)
# Magnetically linear SyRM model for comparison
# machine = model.sm.SynchronousMachine(
#    n_p=2, R_s=.54, L_d=37e-3, L_q=6.2e-3, psi_f=0)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.sm.Drive(machine, mechanics, converter)

# %%
# Configure the control system. The saturation is not taken into account.
# Furthermore, the inductance estimates L_d and L_q are intentionally set to
# lower values in order to demonstrate the PM-flux disturbance estimation.

par = control.sm.ModelPars(
    n_p=2, R_s=.54, L_d=.7*37e-3, L_q=.8*6.2e-3, psi_f=0, J=.015)
# Disable MTPA since the control system does not consider the saturation
ref = control.sm.FluxTorqueReferencePars(
    par, i_s_max=2*base.i, k_u=.9, psi_s_min=base.psi, psi_s_max=base.psi)
ctrl = control.sm.FluxVectorCtrl(par, ref, sensorless=True)
# Since the saturation is not considered in the control system, the speed
# estimation bandwidth is set to a lower value. Furthermore, the PM-flux
# disturbance estimation is enabled at speeds above 2*pi*20 rad/s (electrical).
ctrl.observer = control.sm.Observer(
    par,
    alpha_o=2*np.pi*40,
    k_f=lambda w_m: max(.05*(np.abs(w_m) - 2*np.pi*20), 0),
    sensorless=True)

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

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=4)

# %%
# Plot results in per-unit values. The transient after t = 0.5 s is due to the
# errors in the inductances. The PM-flux estimate compensates for these errors.

plot(sim, base)

# %%
# Plot the flux linkages and the PM-flux disturbance estimate. Due to the
# inductance errors and the magnetic saturation, it is nonzero even if the
# machine has no magnets.

mdl = sim.mdl.data  # Continuous-time data
ctrl = sim.ctrl.data  # Discrete-time data
plt.figure()
plt.plot(mdl.t, np.abs(mdl.psi_s)/base.psi, label=r"$\psi_\mathrm{s}$")
plt.plot(ctrl.t, np.abs(ctrl.psi_s)/base.psi, label=r"$\hat{\psi}_\mathrm{s}$")
plt.plot(ctrl.t, ctrl.psi_f/base.psi, label=r"$\hat{\psi}_\mathrm{f}$")
plt.plot(ctrl.t, ctrl.psi_s_ref/base.psi, "--", label=r"$\psi_\mathrm{s,ref}$")
plt.xlim(0, 4)
plt.xlabel("Time (s)")
plt.ylabel("Flux linkage (p.u.)")
plt.legend()
plt.show()

# %%
# .. rubric:: References
#
# .. [#Tuo2018] Tuovinen, Awan, Kukkola, Saarakkala, Hinkkanen, "Permanent-
#    magnet flux adaptation for sensorless synchronous motor drives," Proc.
#    IEEE SLED, 2018, https://doi.org/10.1109/SLED.2018.8485899
