"""
V/Hz control of 2.2-kW induction motor drive
===========================================================

This example simulates V/Hz of a 2.2-kW induction motor
drive. The six-step overmodulation is enabled, which increases the fundamental
voltage as well as the harmonics. Since the PWM is not synchronized with the
stator freuqency, the harmonic content also depends on the ratoi between the
stator frequency and the switching frequency.

"""
# %%
# Import the package.


import scipy.io as io
import numpy as np
import motulator as mt
from time import time


# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, p=2)

# %%
# Create the system model.

# Γ-equivalent motor model with main-flux saturation included
motor = mt.InductionMotor(
    R_s=3.7, R_r=2.1, L_ell=.021, L_s=.224, p=2)
# Mechanics model
mech = mt.Mechanics(J=.016)
# Frequency converter with a diode bridge
conv = mt.Inverter(u_dc=540)
# Collect the system model
mdl = mt.InductionMotorDrive(motor, mech, conv)

# %%
# Control system (parametrized as open-loop V/Hz control).

ctrl = mt.InductionMotorVHzCtrl(

    mt.InductionMotorVHzCtrlPars(R_s=0, R_R=0, k_u=0, k_w=0, six_step=True))

# %%
# Set the speed reference and the external load torque. More complicated
# signals could be defined as functions.

# Speed reference
times = np.array([0, .2, .75, 1])*2
values = np.array([1, 1, 1, 1])*base.w*1.5
ctrl.w_m_ref = mt.Sequence(times, values)

# External load torque
times = np.array([0, .5, .75, 1])*2
values = np.array([0, 0, 1, 1])*base.tau_nom*0
mdl.mech.tau_L_ext = mt.Sequence(times, values)

# %%
# Create the simulation object and simulate it. The option `pwm=True` enables
# the model for the carrier comparison.

sim = mt.Simulation(mdl, ctrl, pwm=True)
t_start = time()  # Start the timer
sim.simulate(t_stop=1.5)
print('\nExecution time: {:.2f} s'.format((time() - t_start)))

# %%
# Plot results in per-unit values.
#
# .. note::
#    The DC link of this particular example is actually unstable at 1-p.u.
#    speed at the rated load torque, since the inverter looks like a negative
#    resistance to the DC link. You could notice this instability if simulating
#    a longer period (e.g. set `t_stop=2`). For more information, see e.g.
#    https://doi.org/10.1109/EPE.2007.4417763

# sphinx_gallery_thumbnail_number = 2
mt.plot(sim, base=base)
mt.plot_extra(sim, t_span=(0.35, 0.55), base=base)
