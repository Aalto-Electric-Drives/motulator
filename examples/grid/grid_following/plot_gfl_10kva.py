"""
10-kVA converter
================
    
This example simulates a grid-following-controlled converter connected to an L
filter and a strong grid. The control system includes a phase-locked loop (PLL) 
to synchronize with the grid, a current reference generator, and a PI-based
current controller.

"""

# %%
from motulator.grid import model, control
from motulator.grid.utils import (
    BaseValues, ACFilterPars, NominalValues, plot)
# from motulator.grid.utils import plot_voltage_vector
# import numpy as np

# %%
# Compute base values based on the nominal values.

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

# Filter and grid
par = ACFilterPars(L_fc=.2*base.L)
ac_filter = model.ACFilter(par)
ac_source = model.ThreePhaseVoltageSource(w_g=base.w, abs_e_g=base.u)
# Inverter with constant DC voltage
converter = model.VoltageSourceConverter(u_dc=650)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# Uncomment line below to enable the PWM model
# mdl.pwm = model.CarrierComparison()

# %%
# Configure the control system.

cfg = control.GFLControlCfg(
    L=.2*base.L, nom_u=base.u, nom_w=base.w, max_i=1.5*base.i)
ctrl = control.GFLControl(cfg)

# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
ctrl.ref.p_g = lambda t: (t > .02)*5e3
ctrl.ref.q_g = lambda t: (t > .04)*4e3

# Uncomment lines below to simulate an unbalanced fault (add negative sequence)
# mdl.ac_source.par.abs_e_g = .75*base.u
# mdl.ac_source.par.abs_e_g_neg = .25*base.u
# mdl.ac_source.par.phi_neg = -np.pi/3

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=.1)

# %%
# Plot the results.

# By default results are plotted in per-unit values. By omitting the argument
# `base` you can plot the results in SI units.

# Uncomment line below to plot locus of the grid voltage space vector
# plot_voltage_vector(sim, base)
plot(sim, base, plot_pcc_voltage=False)
