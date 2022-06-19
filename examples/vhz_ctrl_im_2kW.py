"""
Example simulation script: V/Hz-controlled 2.2-kW induction motor drive.

A diode bridge, stiff three-phase grid, and a DC link is modeled. The default
parameters correspond to an open-loop V/Hz control.

"""
from time import time
import motulator as mt

if __name__ == '__main__':

    # Start computing the execution time
    start_time = time()

    # Compute base values based on the nominal values (just for figures)
    base = mt.BaseValues(
        U_nom=400, I_nom=5, f_nom=50, tau_nom=14.6, P_nom=2.2e3, p=2)

    # Motor model with main-flux saturation included
    L_s = mt.SaturableStatorInductance()
    motor = mt.InductionMotorSaturated(
        R_s=3.7, R_r=2.5, L_ell=.023, L_s=L_s, p=2)
    mech = mt.Mechanics(J=.015)
    # System model with a diode bridge
    conv = mt.FrequencyConverter(L=2e-3, C=235e-6, U_g=400, f_g=50)
    mdl = mt.InductionMotorDriveDiode(motor, mech, conv)

    # Control system (parametrized as open-loop V/Hz control)
    ctrl = mt.InductionMotorVHzCtrl(mt.InductionMotorVHzCtrlPars(
        R_s=0, R_R=0, k_u=0, k_w=0))

    # Set the speed reference and the external load torque
    ctrl.w_m_ref = lambda t: (t > .2)*(1.*base.w)
    mdl.mech.tau_L_ext = lambda t: (t > 1.)*base.tau_nom

    # Create the simulation object and simulate it
    sim = mt.Simulation(mdl, ctrl, base=base, t_stop=1.5)
    sim.simulate()

    # Print the execution time
    print('\nExecution time: {:.2f} s'.format((time() - start_time)))

    # Plot results in per unit values
    mt.plot_pu(sim)
    mt.plot_extra_pu(sim, t_zoom=(1.1, 1.125))
