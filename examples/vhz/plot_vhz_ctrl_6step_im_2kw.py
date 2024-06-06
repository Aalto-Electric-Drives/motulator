"""
2.2-kW induction motor, 6-step mode
===================================

This example simulates V/Hz control of a 2.2-kW induction motor drive. The 
six-step overmodulation is enabled, which increases the fundamental voltage as 
well as the harmonics. Since the PWM is not synchronized with the stator 
frequency, the harmonic content also depends on the ratio between the stator 
frequency and the sampling frequency.

"""
# %%

import numpy as np
from motulator.drive import model
import motulator.drive.control.im as control
from motulator.drive.utils import BaseValues, NominalValues, plot, plot_extra
from motulator.utils import Sequence

# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=5, f=50, P=2.2e3, tau=14.6)
base = BaseValues.from_nominal(nom, n_p=2)

# %%
# Create the system model.

# Configure the induction machine using its inverse-Γ parameters
machine = model.InductionMachineInvGamma(
    R_s=3.7, R_R=2.1, L_sgm=.021, L_M=.224, n_p=2)
mechanics = model.Mechanics(J=.015)
converter = model.Inverter(u_dc=540)
mdl = model.Drive(converter, machine, mechanics)
mdl.pwm = model.CarrierComparison()  # Enable the PWM model

# %%
# Control system (parametrized as open-loop V/Hz control).

par = control.ModelPars(R_s=0*3.7, R_R=0*2.1, L_sgm=.021, L_M=.224)
ctrl = control.VHzCtrl(
    control.VHzCtrlCfg(par, nom_psi_s=base.psi, k_u=0, k_w=0, six_step=True))
# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .1, .3, 1])*2
values = np.array([0, 0, 1, 1])*2*base.w
ctrl.ref.w_m = Sequence(times, values)

# Quadratic load torque profile (corresponding to pumps and fans)
k = .2*nom.tau/(base.w/base.n_p)**2
mdl.mechanics.tau_L_w = lambda w_M: k*w_M**2*np.sign(w_M)
# External load torque could be set here, now zero
mdl.mechanics.tau_L_t = lambda t: (t > 1.)*nom.tau*0

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=2)

# %%
# Plot results in per-unit values.

# sphinx_gallery_thumbnail_number = 2
plot(sim, base)
plot_extra(sim, base, t_span=(0.58, 0.7))
