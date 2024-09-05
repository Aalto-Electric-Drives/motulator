"""
10-kVA converter, DC-bus voltage
================================

This example simulates a grid-following-controlled converter connected to a
strong grid and regulating the DC-bus voltage. The control system includes a
DC-bus voltage controller, a phase-locked loop (PLL) to synchronize with the
grid, a current reference generator, and a PI-type current controller.

"""

# %%
import numpy as np

from motulator.grid import model, control
from motulator.grid.utils import (
    BaseValues, ACFilterPars, NominalValues, plot)

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
# Converter model with the DC-bus dynamics
converter = model.VoltageSourceConverter(u_dc=600, C_dc=1e-3)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system.

# Create the control system
cfg = control.GridFollowingControlCfg(
    L=.2*base.L, nom_u=base.u, nom_w=base.w, max_i=1.5*base.i)
ctrl = control.GridFollowingControl(cfg)

# Add the DC-bus voltage controller to the control system
ctrl.dc_bus_voltage_ctrl = control.DCBusVoltageController(
    C_dc=1e-3, alpha_dc=2*np.pi*30, max_p=base.p)

# %%
# Set the time-dependent reference and disturbance signals.

# Set the references for DC-bus voltage and reactive power
ctrl.ref.u_dc = lambda t: 600 + (t > .02)*50
ctrl.ref.q_g = lambda t: (t > .04)*4e3

# Set the external current fed to the DC bus
mdl.converter.i_dc = lambda t: (t > .06)*10

# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop=.1)

# %%
# Plot the results.

# By default results are plotted in per-unit values. By omitting the argument
# `base` you can plot the results in SI units.

plot(sim, base)
