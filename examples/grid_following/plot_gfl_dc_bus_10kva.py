"""
10-kVA converter, DC-bus voltage
================================
    
This example simulates a grid-following-controlled converter connected to a
strong grid and regulating the DC-bus voltage. The control system includes a 
DC-bus voltage controller, a phase-locked loop (PLL) to synchronize with the 
grid, a current reference generator, and a PI-type current controller.

"""

# %%
from motulator.common.model import VoltageSourceConverter, Simulation
from motulator.common.utils import BaseValues, NominalValues
from motulator.grid import model
import motulator.grid.control.grid_following as control
from motulator.grid.control import DCBusVoltageController
from motulator.grid.utils import FilterPars, GridPars, plot

# %%
# Compute base values based on the nominal values.

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

# Grid parameters
grid_par = GridPars(u_gN=base.u, w_gN=base.w)

# Filter parameters
filter_par = FilterPars(L_fc=.2*base.L)

# Create AC filter with given parameters
ac_filter = model.ACFilter(filter_par, grid_par)

# AC grid model with constant voltage magnitude and frequency
grid_model = model.ThreePhaseVoltageSource(w_g=base.w, abs_e_g=base.u)

# Inverter model with DC-bus dynamics included
converter = VoltageSourceConverter(u_dc=600, C_dc=1e-3)

# Create system model
mdl = model.GridConverterSystem(converter, ac_filter, grid_model)

# %%
# Configure the control system.

# Create the control system
cfg = control.GFLControlCfg(grid_par, filter_par, max_i=1.5*base.i, C_dc=1e-3)
ctrl = control.GFLControl(cfg)

# Add the DC-bus voltage controller to the control system
ctrl.dc_bus_volt_ctrl = DCBusVoltageController(p_max=base.p)

# %%
# Set the time-dependent reference and disturbance signals.

# Set the references for DC-bus voltage and reactive power
ctrl.ref.u_dc = lambda t: 600 + (t > .02)*50
ctrl.ref.q_g = lambda t: (t > .04)*4e3

# Set the external DC-bus current
mdl.converter.i_ext = lambda t: (t > .06)*10

# %%
# Create the simulation object and simulate it.

sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=.1)

# %%
# Plot the results.

# By default results are plotted in per-unit values. By omitting the argument
# `base` you can plot the results in SI units.

plot(sim, base)
