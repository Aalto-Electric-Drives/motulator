"""
5.6-kW saturated PM-SyRM, discrete-time current control
=======================================================

This example compares the continuous-time and direct discrete-time designs of the
current controller [#Awa2019]_ in sensorless current-vector control of a saturated
5.6-kW permanent-magnet synchronous reluctance machine (PM-SyRM). The sampling frequency
is 5 kHz and the current-control bandwidth is 2π·500 rad/s, which is high compared with
the sampling frequency.

"""
# %%

from math import pi
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import motulator.drive.control.sm as control
from motulator.drive import model, utils

# %%
# Compute base values based on the nominal values.

nom = utils.NominalValues(U=460, I=8.8, f=60, P=5.6e3, tau=29.7)
base = utils.BaseValues.from_nominal(nom, n_p=2)

# %%
# Configure the machine parameters based on the measured flux map, see
# :doc:`/drive_examples/flux_vector/plot_6kw_pmsyrm_sat_fvc`.

p = Path(utils.__file__).resolve().parents[3] / "examples/drive/data"
data = np.load(p / "baldor_400rpm_map.npz")
flux_map = utils.MagneticModel(
    i_s_dq=data["i_s_dq"], psi_s_dq=data["psi_s_dq"], type="flux_map"
)
par = model.SaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, i_s_dq_fcn=flux_map.invert()
)

# %%
# The control system uses an analytical saturation model fitted to the measured data,
# so the model has small errors.

est_current_map = utils.SaturationModelPMSyRM(
    a_d0=3.96,
    a_dd=28.5,
    S=4,
    a_q0=5.89,
    a_qq=2.67,
    T=6,
    a_dq=41.5,
    U=1,
    V=1,
    a_b=81.75,
    a_bp=1,
    k_q=0.1,
    psi_n=0.804,
    W=2,
).as_magnetic_model(
    d_range=np.linspace(-0.1 * base.psi, base.psi, 256),
    q_range=np.linspace(-1.4 * base.psi, 1.4 * base.psi, 256),
)
est_par = control.SaturatedSynchronousMachinePars(
    n_p=2, R_s=0.63, psi_s_dq_fcn=est_current_map.invert()
)

# %%
# Simulate the drive in torque-control mode. A load machine ramps the speed up to 1.5
# p.u. and keeps it constant, and the PWM is modeled. In the discrete-time design, the
# controller compensates for the delays itself, so the PWM does not compensate for the
# angle of the voltage reference, and it gives the realized voltage of the ongoing
# sampling period instead of the average of two sampling periods.


def simulate(discrete: bool):
    mdl = model.Drive(
        model.SynchronousMachine(par),
        model.ExternalRotorSpeed(),
        model.VoltageSourceConverter(u_dc=540),
        pwm=True,
    )
    mdl.mechanics.set_external_rotor_speed(lambda t: 1.5 * base.w_M * min(t / 0.1, 1))
    cfg = control.CurrentVectorControllerCfg(
        i_s_max=2 * base.i,
        alpha_c=2 * pi * 500,
        alpha_ref=2 * pi * 500,  # Fast reference generation
        online_ref=True,
        T_s=200e-6,
        discrete=discrete,
    )
    pwm = control.PWM(k_comp=0, average=False) if discrete else None
    ctrl = control.VectorControlSystem(
        control.CurrentVectorController(est_par, cfg), pwm=pwm
    )
    ctrl.set_torque_ref(
        lambda t: nom.tau * (0.2 * (t > 0.12) + 0.1 * (t > 0.17) - 0.5 * (t > 0.22))
    )
    return model.Simulation(mdl, ctrl).simulate(t_stop=0.27)


res_cont, res_disc = simulate(discrete=False), simulate(discrete=True)

# %%
# Plot the current responses in per-unit values.

_, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
for res, ls, label in [(res_cont, "--", "continuous"), (res_disc, "-", "discrete")]:
    t, i_s = res.ctrl.t, res.ctrl.fbk.i_s / base.i
    ax1.plot(t, i_s.real, ls, ds="steps-post", label=label)
    ax2.plot(t, i_s.imag, ls, ds="steps-post", label=label)
i_s_ref = res_disc.ctrl.ref.i_s / base.i
ax1.plot(t, i_s_ref.real, "k:", ds="steps-post", label="reference")
ax2.plot(t, i_s_ref.imag, "k:", ds="steps-post", label="reference")
ax1.set_ylabel(r"$i_\mathrm{d}$ (p.u.)")
ax2.set_ylabel(r"$i_\mathrm{q}$ (p.u.)")
ax2.set_xlabel("Time (s)")
ax2.set_xlim(0.11, 0.27)
ax2.legend(loc="lower left")
ax1.grid(True)
ax2.grid(True)
plt.show()

# %%
# At this bandwidth, the continuous-time design is sensitive to the model errors and
# results in poorly damped oscillations, while the discrete-time design is robust. In
# the largest steps, the voltage is limited.
#
# .. rubric:: References
#
# .. [#Awa2019] Awan, Saarakkala, Hinkkanen, "Flux-linkage-based current control of
#    saturated synchronous motors," IEEE Trans. Ind. Appl. 2019,
#    https://doi.org/10.1109/TIA.2019.2919258
