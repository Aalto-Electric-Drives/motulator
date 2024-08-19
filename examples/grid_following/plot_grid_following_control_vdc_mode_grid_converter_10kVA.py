"""
10-kVA grid-following converter, DC-bus voltage control
=======================================================
    
This example simulates a grid-following-controlled converter connected to a
strong grid and regulating the DC-bus voltage at the same time. The control
system includes a DC-bus voltage controller, a phase-locked loop (PLL) to
synchronize with the grid, a current reference generator, and a PI-based
current controller.

"""

# %%
from motulator.common.model import (
    CarrierComparison,
    Inverter,
    Simulation,
)
from motulator.common.utils import (
    BaseValues,
    NominalValues,
)
from motulator.grid import model
import motulator.grid.control.grid_following as control
from motulator.grid.control import DCBusVoltageController
from motulator.grid.utils import FilterPars, GridPars, plot_grid

# %%
# Compute base values based on the nominal values.

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Configure the system model.

# Grid parameters
grid_par = GridPars(u_gN=base.u, w_gN=base.w)

# Filter parameters
filter_par = FilterPars(L_fc=0.2*base.L)

# Create AC filter with given parameters
grid_filter = model.GridFilter(filter_par, grid_par)

# AC-voltage magnitude (to simulate voltage dips or short-circuits)
e_g_abs_var = lambda t: base.u

# AC grid model with constant voltage magnitude and frequency
grid_model = model.StiffSource(w_g=grid_par.w_gN, e_g_abs=e_g_abs_var)

# DC-bus parameters
C_dc = 1e-3

# Inverter model with DC-bus dynamics included
converter = Inverter(u_dc=600, C_dc=C_dc)

# Create system model
mdl = model.GridConverterSystem(converter, grid_filter, grid_model)

# %%
# Configure the control system.

# Control parameters
cfg = control.GFLControlCfg(
    grid_par=grid_par,
    C_dc=C_dc,
    filter_par=filter_par,
    i_max=1.5*base.i,
)
# Create the control system
ctrl = control.GFLControl(cfg)

# Add the DC-bus voltage controller to the control system
ctrl.dc_bus_volt_ctrl = DCBusVoltageController(p_max=base.p)

# %%
# Set the time-dependent reference and disturbance signals.

# Set the references for DC-bus voltage and reactive power
ctrl.ref.u_dc = lambda t: 600 + (t > .02)*(50)
ctrl.ref.q_g = lambda t: (t > .04)*(4e3)

# Set the external DC-bus current
mdl.converter.i_ext = lambda t: (t > .06)*(10)

# %%
# Create the simulation object and simulate it.

sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=.1)

# %%
# Plot the results.

# By default results are plotted in per-unit values. By omitting the argument
# `base` you can plot the results in SI units.

plot_grid(sim=sim, base=base, plot_pcc_voltage=True)
