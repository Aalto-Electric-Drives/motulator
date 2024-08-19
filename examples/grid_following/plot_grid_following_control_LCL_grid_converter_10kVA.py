"""
10-kVA grid-following converter with LCL filter, power control
==============================================================
    
This example simulates a grid-following-controlled converter connected to a
strong grid through an LCL filter. The control system includes a phase-locked
loop (PLL) to synchronize with the grid, a current reference generator, and a
PI-based current controller.

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
filter_par = FilterPars(L_fc=0.073*base.L, L_fg=0.073*base.L, C_f=0.043*base.C)

# DC-bus parameters
grid_filter = model.GridFilter(filter_par, grid_par)

# AC-voltage magnitude (to simulate voltage dips or short-circuits)
e_g_abs_var = lambda t: grid_par.u_gN

# AC grid model with constant voltage magnitude and frequency
grid_model = model.StiffSource(w_g=grid_par.w_gN, e_g_abs=e_g_abs_var)

# Inverter model with constant DC voltage
converter = Inverter(u_dc=650)

# Create system model
mdl = model.GridConverterSystem(converter, grid_filter, grid_model)

# Uncomment line below to enable the PWM model
#mdl.pwm = CarrierComparison()

# %%
# Configure the control system.

# # Control parameters
cfg = control.GFLControlCfg(
    grid_par=grid_par,
    filter_par=filter_par,
    on_u_cap=True,
    i_max=1.5*base.i,
)

# Create the control system
ctrl = control.GFLControl(cfg)

# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
ctrl.ref.p_g = lambda t: (t > .02)*(5e3)
ctrl.ref.q_g = lambda t: (t > .04)*(4e3)

# %%
# Create the simulation object and simulate it.

sim = Simulation(mdl, ctrl)
sim.simulate(t_stop=.1)

# %%
# Plot the results.

# By default results are plotted in per-unit values. By omitting the argument
# `base` you can plot the results in SI units.

plot_grid(sim, base=base, plot_pcc_voltage=True)
