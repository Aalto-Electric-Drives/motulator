# pylint: disable=C0103
"""
This module contains the simulation class.

"""

# %% Imports
import copy
import numpy as np
import scipy.io
from helpers import plot, save_plot
from control.common import state_machine


class Simulation:
    """
    Create a simulation object.

    Each simulation has a control module and a plant module.

    """

    def __init__(self, mdl, ctrl, base, name='sim', print_opts=True):
        """
        Parameters
        ----------
        mdl : mdl
            drive model
        ctrl : ctrl
            control model
        base : base
            base values
        name : str, optional
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
        """
        Run the main simulation loop.

        """
        # Run the model
        state_machine(self.mdl, self.ctrl)

        # Call the post-processing function
        self.post_process()

    def post_process(self):
        """
        Execute post simulation processes for the saved simulation data.

        """
        self.mdl.datalog.post_process(self.mdl)
        self.ctrl.datalog.post_process()

    def print_control_config(self):
        """
        Prints the control system data, speed reference and load reference.

        """
        with np.printoptions(precision=1, suppress=True):
            print('\nControl: ',
                  self.ctrl,
                  '\nProfiles\n',
                  '    {}\n'.format(self.mdl.speed_ref),
                  'External load torque:\n',
                  '    {}'.format(self.mdl.mech.tau_L_ext))

    def print_model_config(self):
        """
        Print the model configuration data.

        """
        print('\nSystem model:')
        print(self.mdl.motor)
        print(self.mdl.mech)
        print('PWM model:\n    ', self.mdl.pwm)
        print(self.mdl.converter)
        print(self.mdl.delay)

    def print_simulation_profile(self):
        """
        Print the simulation profiles.

        """
        np.set_printoptions(precision=1, suppress=True)
        print('Profiles')
        print('--------')
        print('Speed reference:')
        print('    {}'.format(self.mdl.speed_ref))
        print('External load torque:')
        print('    {}\n'.format(self.mdl.mech.tau_L_ext))

    def plot_figure(self, save=False):
        """
        Plot the simulation results.

        A wrapper for the plot function in helpers.py.

        """
        plot(self.mdl.datalog.data, self.ctrl.datalog.data, self.base)
        if save:
            save_plot(self.name)

    def save_mat(self):
        """
        Save the simulation data to MATLAB .mat file.

        """
        plant_data = self.mdl.datalog.data
        controller_data = self.ctrl.datalog.data

        # Copies the continuous time vector to a new variable to avoid being
        # rewritten over by the discrete time vector
        t_cont = {'t_cont': plant_data.t}

        # Unpack data
        data = {**plant_data, **t_cont, **controller_data}

        scipy.io.savemat(self.name+'.mat', data)
