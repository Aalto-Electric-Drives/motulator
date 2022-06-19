"""
Example simulation script: vector-controlled 2.2-kW PMSM drive.

Sensorless vector control is used in the default parameters.

"""
from time import time
import numpy as np
import motulator as mt

if __name__ == '__main__':

    # Start computing the execution time
    start_time = time()

    # Base values
    base = mt.BaseValues(U_nom=370, I_nom=4.3, f_nom=75, tau_nom=14,
                         P_nom=2.2e3, p=3)

    # System model
    motor = mt.SynchronousMotor()
    mech = mt.Mechanics()
    conv = mt.Inverter()
    # conv = mt.PWMInverter()
    mdl = mt.SynchronousMotorDrive(motor, mech, conv)

    # Controller
    pars = mt.SynchronousMotorVectorCtrlPars(sensorless=True)
    # pars.plot(base)  # Plot control look-up tables
    ctrl = mt.SynchronousMotorVectorCtrl()

    # Set the speed reference and the external load torque
    # Speed reference
    times = np.array([0, .125, .25, .375, .5, .625,  .75, .875, 1])*4
    values = np.array([0,  0, 1,   1, 0,  -1, -1,   0, 0])*base.w
    ctrl.w_m_ref = mt.Sequence(times, values)
    # External load torque
    times = np.array([0, .125, .125, .875, .875, 1])*4
    values = np.array([0, 0, 1, 1, 0, 0])*base.tau_nom
    mdl.mech.tau_L_ext = mt.Sequence(times, values)

    # Print the system model and controller parameters
    print(str(mdl)+'\n\n'+str(ctrl))

    # Create the simulation object and simulate it
    sim = mt.Simulation(mdl, ctrl, base=base, t_stop=4)
    sim.simulate()

    # Print the execution time
    print('\nExecution time: {:.2f} s'.format((time() - start_time)))

    # Plot results
    mt.plot_pu(sim)
