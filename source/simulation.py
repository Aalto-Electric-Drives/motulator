# pylint: disable=C0103
'''
(c) Aalto Electric Drives (contact: marko.hinkkanen@aalto.fi)

motulator: Open-Source Simulator for Motor Drives and Power Converters

'''
# %% Imports

from model.interfaces import solve
import copy
import numpy as np
from helpers import plot, save_plot
import matplotlib.pyplot as plt

import scipy.io

class Simulation:
    """
    This class implements a simulation object.

    Each simulation has a control module and a plant module.
    """
    def __init__(self, mdl, ctrl, base, name='sim', print_opts=True):
        """
        Initializes the simulation.

        Parameters
        ----------
        mdl : mdl
            drive model
        ctrl : ctrl
            control model
        base : base
            base values
        name : str, Optional
            name for the simulation instance
        """
        self.mdl = copy.deepcopy(mdl)
        self.ctrl = copy.deepcopy(ctrl)
        self.base = copy.deepcopy(base)

        self.name = name

        if print_opts:
            self.print_model_config()
            self.print_control_config()
            self.print_simulation_profile()

    def simulate(self):
        """ Run the main simulation loop.
        """

        # Run the model
        if self.ctrl.sensorless:
            self.sensorless_ctrl()
        else:
            self.sensored_ctrl()

        # Call the post-processing function

        self.post_process()

    def sensorless_ctrl(self):
        """
        Run the digital controller and solve the continuous-time system model.

        """
        while self.mdl.t0 <= self.mdl.t_stop:
            # Sample the phase currents and the DC-bus voltage
            i_s_abc_meas = self.mdl.motor.meas_currents()
            u_dc_meas = self.mdl.converter.meas_dc_voltage()
            # Get the speed reference
            w_m_ref = self.mdl.speed_ref(self.mdl.t0)
            # Run the digital controller
            d_abc_ref, T_s = self.ctrl(w_m_ref, i_s_abc_meas, u_dc_meas)
            # Model the computational delay
            d_abc = self.mdl.delay(d_abc_ref)
            # Simulate the continuous-time system model over the sampling period
            solve(self.mdl, d_abc, [self.mdl.t0, self.mdl.t0+T_s])

    def sensored_ctrl(self):
        """
        Run the digital controller with speed sensor and solve the continuous-time system model.

        """
        while self.mdl.t0 <= self.mdl.t_stop:
            # Sample the phase currents and the DC-bus voltage
            i_s_abc_meas = self.mdl.motor.meas_currents()
            u_dc_meas = self.mdl.converter.meas_dc_voltage()
            # Measure the rotor position (not used in the case of an IM)
            theta_m_meas = self.ctrl.p * self.mdl.mech.meas_position()
            # Measure the rotor speed
            w_m_meas = self.ctrl.p * self.mdl.mech.meas_speed()
            # Get the speed reference
            w_m_ref = self.mdl.speed_ref(self.mdl.t0)
            # Run the digital controller
            d_abc_ref, T_s = self.ctrl(w_m_ref, i_s_abc_meas, u_dc_meas, w_m_meas,
                                  theta_m_meas)
            # Model the computational delay
            d_abc = self.mdl.delay(d_abc_ref)
            # Simulate the continuous-time system model over the sampling period
            solve(self.mdl, d_abc, [self.mdl.t0, self.mdl.t0 + T_s])

    def post_process(self):
        """ Executes post simulation processes for the saved simulation data.

        """
        self.mdl.datalog.post_process(self.mdl)
        self.ctrl.datalog.post_process()

    def print_control_config(self):
        """ Prints the control system data, speed reference and load reference.

        """
        with np.printoptions(precision=1, suppress=True):
            print('\nControl: ',
                  self.ctrl,
                  '\nProfiles\n',
                  '    {}\n'.format(self.mdl.speed_ref),
                  'External load torque:\n',
                  '    {}'.format(self.mdl.mech.tau_L_ext))

    def print_model_config(self):
        """ Prints the model configuration data.

        """
        print('\nSystem model:')
        print(self.mdl.motor)
        print(self.mdl.mech)
        print('PWM model:\n    ', self.mdl.pwm)
        print(self.mdl.converter)
        print(self.mdl.delay)

    def print_simulation_profile(self):

        """ Prints the simulation profiles. """
        np.set_printoptions(precision=1, suppress=True)
        print('Profiles')
        print('--------')
        print('Speed reference:')
        print('    {}'.format(self.mdl.speed_ref))
        print('External load torque:')
        print('    {}\n'.format(self.mdl.mech.tau_L_ext))

    def plot_figure(self, save=False):
        """
        Generates a plot of the simulation.

        A wrapper for the plot function in helpers.py.

        Returns
        -------

        """

        plot(self.mdl.datalog.data, self.ctrl.datalog.data, self.base)
        if save:
            save_plot(self.name)

    def save_mat(self):
        """ Saves the simulation data to MATLAB .mat file. """

        plant_data = self.mdl.datalog.data
        controller_data = self.ctrl.datalog.data

        # Copies the continuous time vector to a new variable to avoid being rewritten over by the discrete time vector
        t_cont = {'t_cont': plant_data.t}

        # Unpack data
        data = {**plant_data, **t_cont, **controller_data}

        scipy.io.savemat(self.name+'.mat', data)


