# pylint: disable=C0103
'''
(c) Aalto Electric Drives (contact: marko.hinkkanen@aalto.fi).

motulator: Open-Source Simulator for Motor Drives and Power Converters

This software includes simulation models for an induction motor, a synchronous
reluctance motor, and a permanent-magnet synchronous motor. Furthermore,
some simple control algorithms are included as examples. The motor models
are simulated in the continuous-time domain while the control algorithms run
in discrete time. The default solver is the explicit Runge-Kutta method of
order 5(4) from scipy.integrate.solve_ivp.

More detailed configuration can be done by editing the config files. There
are separate config files for a drive system and for its controller. For
example, pulse-width modulation (PWM) can be enabled in the drive system
config file. The example control algorithms aim to be simple yet feasible.
They have not been optimized at all.

Notes
-----
This is the very first version. No detailed testing has been carried out.
There can be bugs and misleading comments. Many interfaces will change in the
later versions.

'''
# %% Imports
from time import time
from model.interfaces import solve

# %% Import configuration data for the controller and the model
# You can use the existing config files as templates and modify them
#from config.ctrl_sensored_vector_pmsm_2kW import ctrl, mdl
#from config.ctrl_sensored_vector_syrm_7kW import ctrl, mdl
from config.ctrl_sensored_vector_im_2kW import ctrl, mdl


# %%
def main():
    """
    Run the digital controller and solve the continuous-time system model.

    """
    while mdl.t0 <= mdl.t_stop:
        # Sample the phase currents and the DC-bus voltage
        i_abc_meas = mdl.motor.meas_currents()
        u_dc_meas = mdl.converter.meas_dc_voltage()
        # Measure the rotor position
        theta_M_meas = mdl.mech.meas_position()
        w_M_meas = mdl.mech.meas_speed()
        # Get the speed reference
        w_m_ref = mdl.speed_ref(mdl.t0)
        # Run the digital controller
        d_abc_ref, T_s = ctrl(w_m_ref, w_M_meas, theta_M_meas,
                              i_abc_meas, u_dc_meas)
        # Model the computational delay
        d_abc = mdl.delay(d_abc_ref)
        # Simulate the continuous-time system model over the sampling period
        solve(mdl, d_abc, [mdl.t0, mdl.t0+T_s])


# %% Main program
if __name__ == '__main__':
    # Start computing the execution time
    start_time = time()
    # Run the model
    main()
    # Print the execution time
    print('\nExecution time: {:.2f} s'.format((time() - start_time)))
    # Post-process and plot
    mdl.datalog.post_process(mdl)
    ctrl.datalog.post_process()
    ctrl.datalog.plot(mdl)
