# pylint: disable=C0103
'''
(c) Aalto Electric Drives (contact: marko.hinkkanen@aalto.fi)

motulator: Open-Source Simulator for Motor Drives and Power Converters

'''
# %% Imports
from time import time
from model.interfaces import solve

# %% Import configuration data for the controller and the model
# You can use the existing config files as templates and modify them
# from config.ctrl_sensorless_vector_pmsm_2kW import ctrl, mdl, base
# from config.ctrl_sensorless_vector_syrm_7kW import ctrl, mdl, base
from config.ctrl_sensorless_vector_im_2kW import ctrl, mdl, base
# from config.ctrl_sensorless_vector_im_45kW import ctrl, mdl, base


# %%
def main():
    """
    Run the digital controller and solve the continuous-time system model.

    """
    while mdl.t0 <= mdl.t_stop:
        # Sample the phase currents and the DC-bus voltage
        i_abc_meas = mdl.motor.meas_currents()
        u_dc_meas = mdl.converter.meas_dc_voltage()
        # Get the speed reference
        w_m_ref = mdl.speed_ref(mdl.t0)
        # Run the digital controller
        d_abc_ref, T_s = ctrl(w_m_ref, i_abc_meas, u_dc_meas)
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
    # ctrl.datalog.plot(mdl, base)
    ctrl.datalog.plot_simple(mdl, base)
    # ctrl.datalog.plot_extra(mdl, base)
