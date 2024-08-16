"""
12.5-kVA grid-forming converter, reference-feedforward PSC (RFPSC)
==================================================================
    
This example simulates a grid-forming-controlled converter connected to a
weak grid. The control system includes a power synchronization loop (PSL) to
synchronize with the grid and an inner P-type current controller used to damp
the current oscillations, enhanced with a reference-feedforward term.

"""

# %%
from motulator.common.model import (
    CarrierComparison,
    Inverter,
    Simulation,
)
from motulator.common.utils import BaseValues, NominalValues
from motulator.grid import model
import motulator.grid.control.grid_forming as control
from motulator.grid.utils import FilterPars, GridPars, plot_grid

# %%
# Compute base values based on the nominal values.

nom = NominalValues(U=400, I=18, f=50, P=12.5e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

# Grid parameters
grid_par = GridPars(u_gN=base.u, w_gN=base.w, L_g=0.74*base.L)
# Uncomment line below to simulate a strong grid
#grid_par.L_g = 0

# Filter parameters
filter_par = FilterPars(L_fc=0.15*base.L)

# Create AC filter with given parameters
grid_filter = model.GridFilter(filter_par, grid_par)

# Grid voltage source with constant frequency and voltage magnitude
grid_model = model.StiffSource(w_g=grid_par.w_gN, e_g_abs=grid_par.u_gN)

# Inverter with constant DC voltage
converter = Inverter(u_dc=650)

# Create system model
mdl = model.GridConverterSystem(converter, grid_filter, grid_model)

# Uncomment line below to enable the PWM model
#mdl.pwm = CarrierComparison()

# %%
# Configure the control system.

# Control configuration parameters
cfg = control.PSCControlCfg(
    grid_par=grid_par,
    filter_par=filter_par,
    T_s=1/10e3,
    on_rf=True,  # Enable reference-feedforward
    i_max=1.3*base.i,
    R_a=.2*base.Z,
)

# Create the control system
ctrl = control.PSCControl(cfg)

# %%
# Set the references for converter output voltage magnitude and active power.

# Converter output voltage magnitude reference
ctrl.ref.U = lambda t: grid_par.u_gN

# Active power reference
ctrl.ref.p_g = lambda t: ((t > .2)*(1/3) + (t > .5)*(1/3) + (t > .8)*(1/3) -
                          (t > 1.2))*nom.P

# %%
# Create the simulation object and simulate it.

sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=1.5)

# %%
# Plot the results.

# By default results are plotted in per-unit values. By omitting the argument
# `base` you can plot the results in SI units.

plot_grid(sim, base=base, plot_pcc_voltage=True)
