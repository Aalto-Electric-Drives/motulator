"""
Flux-Vector Controlled 2.2-kW PMSM Drive
========================================

This example simulates sensorless stator-flux-vector control of a 2.2-kW PMSM
drive.

"""

# %%
# Import the packages.

from time import time
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

pars = mt.control.sm_flux_vector.SynchronousMotorFluxVectorCtrlPars()
ctrl = mt.control.sm_flux_vector.SynchronousMotorFluxVectorCtrl(pars)

# %%
# Set the speed reference and the external load torque.

# Simple acceleration and load torque step
ctrl.w_m_ref = lambda t: (t > .2)*(2*base.w)
mdl.mech.tau_L_ext = lambda t: (t > .8)*base.tau_nom*.7

# %%
# Create the simulation object and simulate it.

sim = mt.Simulation(mdl, ctrl, base=base, enable_pwm=False, t_stop=1.6)
start_time = time()  # Start the timer
sim.simulate()
# Print the execution time
print('\nExecution time: {:.2f} s'.format((time() - start_time)))

# %%
# Plot results in per-unit values. By uncommenting the second line you can
# plot the results in SI units.

mt.plot_pu(sim)
# mt.plot(sim)
