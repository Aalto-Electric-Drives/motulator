"""
10-kVA grid following converter, power control
==============================================
    
This example simulates a grid following controlled converter connected to a
strong grid. The control system includes a phase-locked loop (PLL) to
synchronize with the grid, a current reference generatior and a PI-based
current controller.

"""

# %%
# Imports.

import numpy as np
from motulator.grid import model
import motulator.grid.control.grid_following as control
from motulator.grid.utils import BaseValues, NominalValues, plot_grid

# To check the computation time of the program
import time
start_time = time.time()


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Create the system model.

# grid impedance and filter model
grid_filter = model.LFilter(L_f=10e-3, L_g=0, R_g=0)
# AC grid model (either constant frequency or dynamic electromechanical model)
grid_model = model.StiffSource(w_N=2*np.pi*50)
converter = model.Inverter(u_dc=650)
mdl = model.StiffSourceAndLFilterModel(converter, grid_filter, grid_model)


# %%
# Configure the control system.

# Control parameters
pars = control.GridFollowingCtrlPars(
            L_f=10e-3,
            f_sw = 5e3,
            T_s = 1/(10e3),
            i_max = 1.5*base.i,
            )
ctrl = control.GridFollowingCtrl(pars)


# %%
# Set the time-dependent reference and disturbance signals.

# Set the active and reactive power references
ctrl.p_g_ref = lambda t: (t > .02)*(5e3)
ctrl.q_g_ref = lambda t: (t > .04)*(4e3)

# AC-voltage magnitude (to simulate voltage dips or short-circuits)
e_g_abs_var =  lambda t: np.sqrt(2/3)*400
mdl.grid_model.e_g_abs = e_g_abs_var # grid voltage magnitude


# %%
# Create the simulation object and simulate it.

sim = model.Simulation(mdl, ctrl)
sim.simulate(t_stop = .1)

# Print the execution time
print('\nExecution time: {:.2f} s'.format((time.time() - start_time)))


# %%
# Plot results in SI or per unit values.

plot_grid(sim, base, plot_pcc_voltage=True)
