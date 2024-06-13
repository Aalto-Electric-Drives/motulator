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

import time
import numpy as np
import matplotlib.pyplot as plt

from motulator.grid import model
import motulator.grid.control.grid_following as control
#import motulator.grid.control.grid_following as control
from motulator.grid.utils import BaseValues, NominalValues, plot_grid
from motulator.common.utils import complex2abc

# To check the computation time of the program
start_time = time.time()


# %%
# Compute base values based on the nominal values (just for figures).

nom = NominalValues(U=400, I=14.5, f=50, P=10e3)
base = BaseValues.from_nominal(nom)

# %%
# Create the system model.

# grid impedance and filter model
grid_filter = model.LFilter(U_gN=400*np.sqrt(2/3) ,R_f=0 ,L_f=10e-3, L_g=0, R_g=0)
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

#plot_grid(sim, base, plot_pcc_voltage=True)

fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 7))
FS = 16 # Font size of the plots axis
FL = 16 # Font size of the legends only
LW = 3 # Line width in plots
t_range = (0, 0.1)
ctrl=sim.ctrl.data

# Subplot 1: PCC voltage
ax1.plot(ctrl.t, ctrl.u_g_abc/base.u, linewidth=LW)
#ax1.plot(mdl.grid_filter.data.t, (mdl.grid_filter.data.e_gs)/base.u, linewidth=LW)
ax1.legend([r'$u_g^a$',r'$u_g^b$',r'$u_g^c$'],
            prop={'size': FL}, loc= 'upper right')
ax1.set_xlim(t_range)
ax1.set_xticklabels([])

ax2.plot(ctrl.t, np.real(ctrl.i_c/base.i), linewidth=LW)
ax2.plot(ctrl.t, np.imag(ctrl.i_c/base.i), linewidth=LW)
ax2.plot(ctrl.t, np.real(ctrl.i_c_ref/base.i), '--', linewidth=LW)
ax2.plot(ctrl.t, np.imag(ctrl.i_c_ref/base.i), '--', linewidth=LW)
#ax2.plot(mdl.t, mdl.iL, linewidth=LW) converter-side dc current for debug
ax2.legend([r'$i_{c}^d$',r'$i_{c}^q$',r'$i_{c,ref}^d$',r'$i_{c,ref}^q$'],
            prop={'size': FL}, loc= 'upper right')
ax2.set_xlim(t_range)
ax2.set_xticklabels([])

plt.show()
print(mdl.grid_filter.data.e_gs)
