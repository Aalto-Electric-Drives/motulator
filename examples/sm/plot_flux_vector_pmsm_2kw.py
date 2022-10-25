"""
Flux-vector controlled 2.2-kW PMSM drive
========================================

This example simulates sensorless stator-flux-vector control of a 2.2-kW PMSM
drive.

"""

# %%
# Import the package.

import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14, P_nom=2.2e3, p=3)

# %%
# Configure the system model.

motor = mt.SynchronousMotor()
mech = mt.Mechanics()
conv = mt.Inverter()
mdl = mt.SynchronousMotorDrive(motor, mech, conv)

# %%
# Configure the control system.

pars = mt.SynchronousMotorFluxVectorCtrlPars(sensorless=True, T_s=250e-6)
ctrl = mt.SynchronousMotorFluxVectorCtrl(pars)

# %%
# Set the speed reference and the external load torque.

# Simple acceleration and load torque step
ctrl.w_m_ref = lambda t: (t > .2)*(2*base.w)
mdl.mech.tau_L_t = lambda t: (t > .8)*base.tau_nom*.7

# %%
# Create the simulation object and simulate it.

sim = mt.Simulation(mdl, ctrl, pwm=False)
sim.simulate(t_stop=1.6)

# %%
# Plot results in per-unit values.

mt.plot(sim, base=base)
