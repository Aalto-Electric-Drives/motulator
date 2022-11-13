"""
Observer-based V/Hz control: 2.2-kW PMSM with two-mass mechanics
================================================================

This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive. The
mechanical subsystem is modeled as a two-mass system. The mechanical parameters
correspond approximately to [1]_, except that the torsional damping is set to
a smaller value in this example. The resonance freuqency of the mechanics is
around 85 Hz.

References
----------
.. [1] Saarakkala, Hinkkanen, "Identification of two-mass mechanical
   systems using torque excitation: Design and experimental evaluation,"
   IEEE Trans. Ind. Appl., 2015, https://doi.org/10.1109/tia.2015.2416128.

"""

# %%
# Import the packages.

import numpy as np
import motulator as mt
import matplotlib.pyplot as plt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14, P_nom=2.2e3, p=3)

# %%
# Configure the system model.

mech = mt.MechanicsTwoMass(J_M=.005, J_L=.005, K_S=700, C_S=.01)
motor = mt.SynchronousMotor()
conv = mt.Inverter()
mdl = mt.SynchronousMotorDriveTwoMass(motor, mech, conv)

# %%
# Configure the control system.

pars = mt.SynchronousMotorVHzObsCtrlPars()
ctrl = mt.SynchronousMotorVHzObsCtrl(pars)

# Default sensorless vector control is unstable (uncomment to try)
# pars = mt.SynchronousMotorVectorCtrlPars(sensorless=True)
# ctrl = mt.SynchronousMotorVectorCtrl(pars)

# %%
# Set the speed reference and the external load torque.

# Speed reference
times = np.array([0, .1, .2, 1])
values = np.array([0, 0, 1, 1])*base.w*.5
ctrl.w_m_ref = mt.Sequence(times, values)
# External load torque
times = np.array([0, .4, .4, 1])
values = np.array([0, 0, 1, 1])*base.tau_nom
mdl.mech.tau_L_t = mt.Sequence(times, values)

# %%
# Create the simulation object and simulate it.

sim = mt.Simulation(mdl, ctrl, pwm=False)
sim.simulate(t_stop=1.2)
# sphinx_gallery_thumbnail_number = 3
mt.plot(sim, base=base)  # Plot results in per-unit values

# %%
# Plot the load speed and the twist angle.


def plot(sim, t_span=None):
    """Plot the load speed and the twist angle."""
    # Continuous-time data
    mdl = sim.mdl.data
    # Plot
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5))
    ax1.plot(mdl.t, mdl.w_M, label=r'$\omega_\mathrm{M}$')
    ax1.plot(mdl.t, mdl.w_L, label=r'$\omega_\mathrm{L}$')
    ax2.plot(mdl.t, mdl.theta_ML*180/np.pi)
    ax1.set_xlim(t_span)
    ax2.set_xlim(t_span)
    ax1.set_xticklabels([])
    ax1.set_ylabel(r'$\omega_\mathrm{M}$, $\omega_\mathrm{L}$ (rad/s)')
    ax2.set_ylabel(r'$\vartheta_\mathrm{ML}$ (deg)')
    ax2.set_xlabel('Time (s)')


plot(sim, t_span=(0, 1.2))


# %%
# Plot also the frequency response from the electromagnetic torque tau_M to the
# rotor speed w_M.

def plot_freq_resp(mech, f_span=None, num=200):
    """Plot the frequency response."""
    # Parameters
    J_M, J_L, K_S, C_S = mech.J_M, mech.J_L, mech.K_S, mech.C_S
    # Frequencies
    w = 2*np.pi*np.logspace(np.log10(f_span[0]), np.log10(f_span[-1]), num=num)
    s = 1j*w
    # Frequency response
    B = J_L*s**2 + C_S*s + K_S
    A = s*(J_M*J_L*s**2 + (J_M + J_L)*C_S*s + (J_M + J_L)*K_S)
    G = B/A
    # Plot figure
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8, 5))
    ax1.loglog(w/(2*np.pi), np.abs(G))
    ax1.set_xticklabels([])
    ax2.semilogx(w/(2*np.pi), np.angle(G)*180/np.pi)
    ax1.set_xlim(f_span)
    ax2.set_xlim(f_span)
    ax2.set_ylim([-100, 100])
    ax2.set_yticks([-90, -45, 0, 45, 90])
    ax1.set_ylabel(r'Amplitude (rad/(s$\cdot$Nm))')
    ax2.set_ylabel('Phase (deg)')
    ax2.set_xlabel('Frequency (Hz)')
    fig.align_ylabels()


plot_freq_resp(mech, f_span=(5, 500))
