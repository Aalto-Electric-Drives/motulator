"""
Flux-vector control: 6.7-kW SyRM
================================

This example simulates sensorless stator-flux-vector control of a saturated
6.7-kW synchronous reluctance motor drive. The saturation is not taken into
account in the control method (only in the system model).

"""

# %%
# Import the packages.

import numpy as np
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=370, I_nom=15.5, f_nom=105.8, tau_nom=20.1, P_nom=6.7e3, n_p=2)

# %%
# Configure the system model.

# Saturated SyRM model
motor = mt.SynchronousMotorSaturated()
# Magnetically linear SyRM model below (uncomment to try)
# motor = mt.SynchronousMotor(n_p=2, R_s=.54, L_d=37e-3, L_q=6.2e-3, psi_f=0)
mech = mt.Mechanics()
conv = mt.Inverter()
mdl = mt.SynchronousMotorDrive(motor, mech, conv)

# %%
# Configure the control system.

pars = mt.SynchronousMotorFluxVectorCtrlPars(
    sensorless=True,
    T_s=250e-6,
    # Disable MTPA since the control system does not consider the saturation
    psi_s_min=base.psi,
    psi_s_max=base.psi,
    # Motor parameter estimates
    R_s=.54,
    L_d=37e-3,
    L_q=6.2e-3,
    psi_f=0,
    n_p=2,
    J=.015,
    # Other controller parameters
    alpha_psi=2*np.pi*50,
    alpha_tau=2*np.pi*50,
    alpha_s=2*np.pi*4,
    w_o=2*np.pi*50,
    tau_M_max=2*base.tau_nom,
    i_s_max=2*base.i,
)
ctrl = mt.SynchronousMotorFluxVectorCtrl(pars)

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
