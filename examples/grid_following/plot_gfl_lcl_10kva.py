"""
10-kVA converter, LCL filter
============================

This example simulates a grid-following-controlled converter connected to a
strong grid through an LCL filter. The control system includes a phase-locked
loop (PLL) to synchronize with the grid, a current reference generator, and a
PI-type current controller.

"""

# %%
from motulator.common.model import VoltageSourceConverter, Simulation
from motulator.common.utils import BaseValues, NominalValues
from motulator.grid import model
import motulator.grid.control.grid_following as control
from motulator.grid.utils import FilterPars, GridPars, plot

# %%
# Compute base values based on the nominal values.

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

# Grid and filter parameters
grid_par = GridPars(u_gN=base.u, w_gN=base.w)
filter_par = FilterPars(L_fc=0.073 * base.L, L_fg=0.073 * base.L, C_f=0.043 * base.C)

# DC-bus parameters
ac_filter = model.ACFilter(filter_par, grid_par)

# AC grid model with constant voltage magnitude and frequency
grid_model = model.ThreePhaseVoltageSource(w_g=base.w, abs_e_g=base.u)

# Inverter model with constant DC voltage
converter = VoltageSourceConverter(u_dc=650)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, grid_model)

# %%
# Configure the control system.

# Control parameters
cfg = control.GFLControlCfg(grid_par, filter_par, max_i=1.5 * base.i)

# Create the control system
ctrl = control.GFLControl(cfg)

# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
ctrl.ref.p_g = lambda t: (t > 0.02) * 5e3
ctrl.ref.q_g = lambda t: (t > 0.04) * 4e3

# %%
# Create the simulation object and simulate it.

sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=0.1)

# %%
# Plot the results.

plot(sim, base)
