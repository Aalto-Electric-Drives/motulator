"""
2.2-kW IPMSM, blocked gates, CVC
================================

This example simulates sensorless current-vector control (CVC) of a 2.2-kW interior
permanent-magnet synchronous machine (IPMSM) drive, whose converter gates are blocked
before the start and during coasting. While the gates are blocked, the control system
runs in standby: its observer tracks the machine using the measured converter voltage,
and its current controller tracks the measured operating point, from which the control
continues when the gates are released at 0.05 s. The drive then accelerates under speed
control. At 0.6 s, the gates are blocked again: the stator current decays through the
diodes within a millisecond, after which the loaded machine coasts at open circuit,
since its voltage is below the DC-bus voltage. At 0.8 s, the gates are released, and
the drive returns to the speed reference.

"""

# %%
from math import pi

import matplotlib.pyplot as plt
import numpy as np

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
base = utils.BaseValues.from_nominal(nom, n_p=3)

# %%
# Configure the system model.

par = model.SynchronousMachinePars(n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545)
machine = model.SynchronousMachine(par)
mechanics = model.MechanicalSystem(J=0.015)
converter = model.VoltageSourceConverter(u_dc=540)
mdl = model.Drive(machine, mechanics, converter)

# %%
# Configure the control system.

est_par = par  # Assume accurate model parameter estimates
cfg = control.CurrentVectorControllerCfg(i_s_max=1.5 * base.i)
vector_ctrl = control.CurrentVectorController(est_par, cfg)
speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

# %%
# Set the gate enable, the speed reference, and the external load torque.

ctrl.set_enable(lambda t: 0.05 <= t < 0.6 or t >= 0.8)
ctrl.set_speed_ref(lambda t: (t > 0.05) * 0.8 * base.w_M)
mdl.mechanics.set_external_load_torque(lambda t: (t > 0.3) * 0.5 * nom.tau)

# %%
# Create the simulation object, simulate, and plot the results in per-unit values.

sim = model.Simulation(mdl, ctrl)
res = sim.simulate(t_stop=1.2)
utils.plot(res, base)

# %%
# Plot the speed, the stator current magnitude, and the DC current of the converter,
# the intervals with blocked gates shaded. The DC current is negative while the stator
# current decays through the diodes.

t_ctrl, enabled = res.ctrl.t, res.ctrl.ref.enabled
w_M = res.mdl.mechanics.w_M / base.w_M
i_s = np.abs(res.mdl.machine.i_s_ab) / base.i
i_dc = res.mdl.converter.i_dc_int / base.i
fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)
edges = np.flatnonzero(np.diff(np.concatenate(([1], enabled, [1])).astype(int)))
for ax in (ax1, ax2, ax3):
    for k_off, k_on in zip(edges[::2], edges[1::2], strict=True):
        ax.axvspan(t_ctrl[k_off], t_ctrl[min(k_on, len(t_ctrl) - 1)], color="0.9")
ax1.plot(res.mdl.t, w_M, label=r"$\omega_\mathrm{M}$")
ax1.plot(t_ctrl, res.ctrl.ref.w_M / base.w_M, "--", label=r"$\omega_\mathrm{M,ref}$")
ax1.set_ylabel("Speed (p.u.)")
ax1.legend()
ax2.plot(res.mdl.t, i_s)
ax2.set_ylabel(r"$|i_\mathrm{s}|$ (p.u.)")
ax3.plot(res.mdl.t, i_dc)
ax3.set_ylabel(r"$i_\mathrm{dc}$ (p.u.)")
ax3.set_xlabel("Time (s)")
ax3.set_xlim(0, 1.2)
plt.show()
