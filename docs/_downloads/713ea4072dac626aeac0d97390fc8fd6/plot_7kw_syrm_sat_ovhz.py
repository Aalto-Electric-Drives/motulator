"""
6.7-kW saturated SyRM, O-V/Hz control
=====================================

This example simulates observer-based V/Hz (O-V/Hz) control of a 6.7-kW synchronous
reluctance machine (SyRM) drive. The magnetic saturation is included in the machine
model, while the control system uses constant parameters.

"""

# %%
import numpy as np

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values (just for figures).

nom = utils.NominalValues(U=370, I=15.5, f=105.8, P=6.7e3, tau=20.1)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the system model. The saturation model is based on [#Hin2017]_.

i_s_dq_fcn = utils.SaturationModelSyRM(
    a_d0=17.4, a_dd=373, S=5, a_q0=52.1, a_qq=658, T=1, a_dq=1120, U=1, V=0
)
par = model.SaturatedSynchronousMachinePars(n_p=2, R_s=0.54, i_s_dq_fcn=i_s_dq_fcn)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

est_par = control.SynchronousMachinePars(
    n_p=2, R_s=0.54, L_d=37e-3, L_q=6.2e-3, psi_f=0
)
cfg = control.ObserverBasedVHzControllerCfg(
    i_s_max=2 * base.i, psi_s_min=base.psi, psi_s_max=base.psi
)
vhz_ctrl = control.ObserverBasedVHzController(est_par, cfg)
ctrl = control.VHzControlSystem(vhz_ctrl)

# %%
# Set the speed reference and the external load torque.

t_stop = 8
times = np.array([0, 0.125, 0.25, 0.375, 0.5, 0.625, 0.75, 0.875, 1]) * t_stop
values = np.array([0, 0, 1, 1, 0, -1, -1, 0, 0]) * base.w_M
ctrl.set_speed_ref(utils.SequenceGenerator(times, values))

times = np.array([0, 0.125, 0.125, 0.875, 0.875, 1]) * t_stop
values = np.array([0, 0, 1, 1, 0, 0]) * nom.tau
mdl.mechanics.set_external_load_torque(utils.SequenceGenerator(times, values))

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop)
utils.plot(res, base)

# %%
# .. rubric:: References
#
# .. [#Hin2017] Hinkkanen, Pescetto, Mölsä, Saarakkala, Pellegrino, Bojoi, “Sensorless
#    self-commissioning of synchronous reluctance motors at standstill without rotor
#    locking, ”IEEE Trans. Ind. Appl., 2017, https://doi.org/10.1109/TIA.2016.2644624
