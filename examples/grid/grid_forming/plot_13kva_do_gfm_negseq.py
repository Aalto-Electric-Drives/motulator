"""
12.5-kVA, DO-GFM with sequence separation
=========================================

This example simulates a 12.5-kVA disturbance-observer-based grid-forming (DO-GFM)
converter, connected to a weak grid. The control method is designed to take into account
negative sequence grid voltage. The positive-sequence converter output voltage and the
positive-sequence active power are directly controlled. Grid synchronization is provided
by the disturbance observer. A transparent current controller is included for current
limitation.

"""

# %%
from math import pi

from motulator.common.control._pwm import PWM
from motulator.grid import control, model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=400, I=18, f=50, P=12.5e3)
base = utils.BaseValues.from_nominal(nom)

# %%
# %%
# Configure the system model. An unbalanced fault is simulated, where the phase-a
# voltage drops to 0.5 p.u. at 0.4 s, rises to 0.75 p.u. at 0.5 s, and finally rises to
# 1 p.u. at 0.6 s.

ac_filter = model.LFilter(L_f=0.15 * base.L, L_g=0.62 * base.L)
ac_source = model.ThreePhaseSource(
    w_g=base.w,
    e_g=lambda t: (1 - (t > 0.4) * 1 / 6 + (t > 0.5) * 1 / 12 + (t > 0.6) * 1 / 12)
    * base.u,
    e_g_neg=lambda t: ((t > 0.4) * 1 / 6 - (t > 0.5) * 1 / 12 - (t > 0.6) * 1 / 12)
    * base.u,
    phi_neg=pi,
)
converter = model.VoltageSourceConverter(u_dc=660)
mdl = model.GridConverterSystem(converter, ac_filter, ac_source)

# %%
# Configure the control system.
cfg = control.SequenceSeparatedObserverBasedGridFormingControllerCfg(
    i_max=1.3 * base.i, i_d_max=1.1 * base.i, L=0.2 * base.L, T_s=1 / 16e3
)
inner_ctrl = control.SequenceSeparatedObserverBasedGridFormingController(cfg)
ctrl = control.GridConverterControlSystem(inner_ctrl)
ctrl.pwm = PWM(k_comp=0)

# %%
# Set the references for converter output voltage magnitude and active power.
ctrl.set_ac_voltage_ref(base.u)
ctrl.set_power_ref(lambda t: (t > 0.1) * nom.P)

# Uncomment the lines below to simulate a three-phase grid voltage sag to 0.5 p.u.
# mdl.ac_source.e_g = lambda t: (1 - (t > 0.4) * 0.5 + (t > 0.6) * 0.5) * base.u
# mdl.ac_source.e_g_neg = 0

# Uncomment the lines below to simulate active-power reference tracking
# mdl.ac_source.e_g = base.u
# mdl.ac_source.e_g_neg = 0
# ctrl.set_power_ref(lambda t: ((t > 0.1) - (t > 0.25) - (t > 0.4) + (t > 0.55)) * nom.P)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=0.8)
utils.plot_control_signals(res, base)
utils.plot_grid_waveforms(res, base)
