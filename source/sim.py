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
from config.ctrl_vector_im_2kW import ctrl, pars, mdl, base, plot
# from config.ctrl_vector_pmsm_2kW import ctrl, pars, mdl, base, plot
# from config.ctrl_vector_syrm_7kW import ctrl, pars, mdl, base, plot
# from config.ctrl_vhz_im_2kW import ctrl, pars, mdl, base, plot


# %%
def sensorless_ctrl():
    """
    Run the digital controller and solve the continuous-time system model.

    """
    while mdl.t0 <= mdl.t_stop:
        # Sample the phase currents and the DC-bus voltage
        i_s_abc_meas = mdl.motor.meas_currents()
        u_dc_meas = mdl.converter.meas_dc_voltage()
        # Get the speed reference
        w_m_ref = mdl.speed_ref(mdl.t0)
        # Run the digital controller
        d_abc_ref, T_s = ctrl(w_m_ref, i_s_abc_meas, u_dc_meas)
        # Model the computational delay
        d_abc = mdl.delay(d_abc_ref)
        # Simulate the continuous-time system model over the sampling period
        solve(mdl, d_abc, [mdl.t0, mdl.t0+T_s])


# %%
def sensored_ctrl():
    """
    Run the digital controller and solve the continuous-time system model.

    """
    while mdl.t0 <= mdl.t_stop:
        # Sample the phase currents and the DC-bus voltage
        i_s_abc_meas = mdl.motor.meas_currents()
        u_dc_meas = mdl.converter.meas_dc_voltage()
        # Measure the rotor position (not used in the case of an IM)
        theta_m_meas = ctrl.p*mdl.mech.meas_position()
        # Measure the rotor speed
        w_m_meas = ctrl.p*mdl.mech.meas_speed()
        # Get the speed reference
        w_m_ref = mdl.speed_ref(mdl.t0)
        # Run the digital controller
        d_abc_ref, T_s = ctrl(w_m_ref, i_s_abc_meas, u_dc_meas, w_m_meas,
                              theta_m_meas)
        # Model the computational delay
        d_abc = mdl.delay(d_abc_ref)
        # Simulate the continuous-time system model over the sampling period
        solve(mdl, d_abc, [mdl.t0, mdl.t0+T_s])


# %% Main program
if __name__ == '__main__':

    # Start computing the execution time
    start_time = time()

    # Run the model
    if pars.sensorless:
        sensorless_ctrl()
    else:
        sensored_ctrl()

    # Print the execution time
    print('\nExecution time: {:.2f} s'.format((time() - start_time)))

    # Post-process and plot
    mdl.datalog.post_process(mdl)
    ctrl.datalog.post_process()
    plot(mdl.datalog.data, ctrl.datalog.data, base)
