"""
V/Hz control: 2.2-kW induction motor
====================================

A diode bridge, stiff three-phase grid, and a DC link is modeled. The default
parameters correspond to open-loop V/Hz control. Magnetic saturation of the 
motor is also modeled.

"""
# %%
# Import the package.

from time import time
import numpy as np
import motulator as mt

# %%
# Compute base values based on the nominal values (just for figures).

base = mt.BaseValues(
    U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, n_p=2)

# %%
# The main-flux saturation model is created based on [1]_. For simplicity, the
# parameters are hard-coded in the function below, but this model structure can
# be used also for other induction motors.


def L_s(psi):
    """
    Stator inductance saturation model for a 2.2-kW motor.

    Parameters
    ----------
    psi : float
        Magnitude of the stator flux linkage (Vs).
    
    Returns
    -------
    float
        Stator inductance (H).

    """
    # Saturation model parameters for a 2.2-kW induction motor
    L_su, beta, S = .34, .84, 7
    # Stator inductance
    return L_su/(1 + (beta*psi)**S)


# %%
# Create the system model.

mdl = mt.InductionMotorDriveDiode()
# Γ-equivalent motor model with main-flux saturation included
mdl.motor = mt.InductionMotorSaturated(
    n_p=2, R_s=3.7, R_r=2.5, L_ell=.023, L_s=L_s)
# Mechanics model
mdl.mech = mt.Mechanics(J=.015)
# Frequency converter with a diode bridge
mdl.conv = mt.FrequencyConverter(L=2e-3, C=235e-6, U_g=400, f_g=50)

# %%
# Control system (parametrized as open-loop V/Hz control).

ctrl = mt.InductionMotorVHzCtrl(
    mt.InductionMotorVHzCtrlPars(R_s=0, R_R=0, k_u=0, k_w=0))

# %%
# Set the speed reference and the external load torque. More complicated
# signals could be defined as functions.

ctrl.w_m_ref = lambda t: (t > .2)*(1.*base.w)

# Quadratic load torque profile (corresponding to pumps and fans)
k = 1.1*base.tau_nom/(base.w/base.n_p)**2
mdl.mech.tau_L_w = lambda w_M: k*w_M**2*np.sign(w_M)

# Stepwise load torque at t = 1 s, 20% of the rated torque
mdl.mech.tau_L_t = lambda t: (t > 1.)*base.tau_nom*.2

# %%
# Create the simulation object and simulate it. The option `pwm=True` enables
# the model for the carrier comparison.

sim = mt.Simulation(mdl, ctrl, pwm=True)
t_start = time()  # Start the timer
sim.simulate(t_stop=1.5)
print(f'\nExecution time: {(time() - t_start):.2f} s')

# %%
# Plot results in per-unit values.
#
# .. note::
#    The DC link of this particular example is actually unstable at 1-p.u.
#    speed at the rated load torque, since the inverter looks like a negative
#    resistance to the DC link. You could notice this instability if simulating
#    a longer period (e.g. set `t_stop=2`). For more information, see e.g. [2]_.

# sphinx_gallery_thumbnail_number = 2
mt.plot(sim, base=base)
mt.plot_extra(sim, t_span=(1.1, 1.125), base=base)

# %%
# References
# ----------
# .. [1] Qu, Ranta, Hinkkanen, Luomi, "Loss-minimizing flux level control of
#    induction motor drives," IEEE Trans. Ind. Appl., 2012,
#    https://doi.org/10.1109/TIA.2012.2190818
# .. [2] Hinkkanen, Harnefors, Luomi, "Control of induction motor drives
#    equipped with small DC-Link capacitance," Proc. EPE, 2007,
#    https://doi.org/10.1109/EPE.2007.4417763
