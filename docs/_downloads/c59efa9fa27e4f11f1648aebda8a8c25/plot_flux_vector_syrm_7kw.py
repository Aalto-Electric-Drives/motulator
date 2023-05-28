"""
6.7-kW SyRM
===========

This example simulates sensorless stator-flux-vector control of a saturated
6.7-kW synchronous reluctance motor drive. The saturation is not taken into
account in the control method (only in the system model).

"""

# %%
# Imports.

import numpy as np
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
# machine = model.SynchronousMachine(
#    n_p=2, R_s=.54, L_d=37e-3, L_q=6.2e-3, psi_f=0)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.sm.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

par = control.sm.ModelPars(
    n_p=2, R_s=.54, L_d=37e-3, L_q=6.2e-3, psi_f=0, J=.015)
# Disable MTPA since the control system does not consider the saturation
ref = control.sm.FluxTorqueReferencePars(
    par, i_s_max=2*base.i, k_u=.9, psi_s_min=base.psi, psi_s_max=base.psi)
ctrl = control.sm.FluxVectorCtrl(par, ref, sensorless=True)
# Since the saturation is not considered in the control system, the speed
# estimation bandwidth is set to a lower value
ctrl.observer = control.sm.Observer(par, alpha_o=2*np.pi*50)

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

sim = model.Simulation(mdl, ctrl, pwm=False)
sim.simulate(t_stop=4)

# %%
# Plot results in per-unit values.

plot(sim, base)
