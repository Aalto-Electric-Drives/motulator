# pylint: disable=C0103
'''
(c) Aalto Electric Drives (contact: marko.hinkkanen@aalto.fi)

motulator: Open-Source Simulator for Motor Drives and Power Converters

'''
# %% Imports
from time import time
from model.interfaces import solve
import matplotlib.pyplot as plt
from multiprocessing import Pool, RLock# , freeze_support
import copy
import numpy as np
from helpers import plot

try:
    from tqdm import tqdm, trange  # Can play around with tqdm.gui aswell, prints progress bars using matplotlib
except ImportError:
    print('tqdm could not be imported, because the tqdm library is missing')

# %%
# Import control and model settings from the config files
import config.ctrl_vector_im_2kW as vec_im_2kW
import config.ctrl_vector_pmsm_2kW as vec_pmsm_2kW
import config.ctrl_vector_syrm_7kW as vec_syrm_7kW
import config.ctrl_vhz_im_2kW as vhz_im_2kW


class Simulation:
    """
    This class implements a simulation object. Each simulation has a control module and a plant model
    in addition to a main simulation loop and plotting functions
    """

    def __init__(self, ctrl, mdl, base, pars, name='sim') -> None:
        # Deepcopy() creates new instances of the models (and whatever is under them)
        self.ctrl = copy.deepcopy(ctrl)
        self.mdl = copy.deepcopy(mdl)
        self.base = copy.deepcopy(base)
        self.pars = copy.deepcopy(pars)
        self.name = name

    def plot_figure(self):
        plot(self.mdl.datalog.data, self.ctrl.datalog.data, self.base)

    def sensorless_ctrl(self, i=0):
        """
        Run the digital controller and solve the continuous-time system model.

        """
        # Start computing the execution time
        if hasattr(self.ctrl, 'speed_ctrl'):
            runs = int(self.mdl.t_stop / self.ctrl.speed_ctrl.T_s)
        else:
            runs = int(self.mdl.t_stop / self.pars.T_s)

        with tqdm(total=runs, position=i, desc="Simulating: " + self.name, ncols=80, leave=False,
                  disable=False) as pbar:
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
                solve(self.mdl, d_abc, [self.mdl.t0, self.mdl.t0 + T_s])
                pbar.update()

    def sensored_ctrl(self, i=0):
        """
        Run the digital controller and solve the continuous-time system model.

        """
        # Start computing the execution time
        if hasattr(self.ctrl, 'speed_ctrl'):
            runs = int(self.mdl.t_stop / self.ctrl.speed_ctrl.T_s)
        else:
            runs = int(self.mdl.t_stop / self.pars.T_s)

        with tqdm(total=runs, position=i, desc="Simulating: " + self.name, ncols=80, leave=False,
                  disable=False) as pbar:
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
                d_abc_ref, T_s = self.ctrl(w_m_ref, i_s_abc_meas, u_dc_meas, w_m_meas, theta_m_meas)
                # Model the computational delay
                d_abc = self.mdl.delay(d_abc_ref)
                # Simulate the continuous-time system model over the sampling period
                solve(self.mdl, d_abc, [self.mdl.t0, self.mdl.t0 + T_s])
                pbar.update()

    def simulate(self, i=0):
        # Run the model
        if self.pars.sensorless:
            self.sensorless_ctrl(i)
        else:
            self.sensored_ctrl(i)

        # Post-process
        self.mdl.datalog.post_process(self.mdl)
        self.ctrl.datalog.post_process()

    def print_control_config(self):
        # %% Print the control system data, speed reference and load reference
        with np.printoptions(precision=1, suppress=True):
            print('\nControl: ',
                  self.ctrl,
                  '\nProfiles\n',
                  '    {}\n'.format(self.mdl.speed_ref),
                  'External load torque:\n',
                  '    {}'.format(self.mdl.mech.tau_L_ext))

    def print_model_config(self):
        # %% Print the system data
        print('\nSystem model:')
        print(self.mdl.motor)
        print(self.mdl.mech)
        print('PWM model:\n    ', self.mdl.pwm)
        print(self.mdl.converter)
        print(self.mdl.delay)


class SimEnv:
    """
    Implements a class to store individual simulation objects. Has a method to simulate using multiple processors
    i.e. allows the user to run several simulations in parallel.
    """

    def __init__(self) -> None:
        # Create a list of simulations
        self.simulations = []

    def add_sim(self, ctrl, mdl, base, pars, name=""):
        sim = Simulation(ctrl, mdl, base, pars, name)
        self.simulations.append(sim)
        return sim

    def simulate(self):
        for sim in self.simulations:
            sim.simulate()

    def mp_simulation_function(self, i):
        self.simulations[i].simulate(i)
        return self.simulations[i]

    def simulate_all(self):
        """
        Simulates all the stored simulation instances simultaneously using multiple processors
        """
        with Pool(initializer=tqdm.set_lock(RLock()), initargs=(tqdm.get_lock(),)) as pool:
            num_tasks = range(len(self.simulations))
            self.simulations = pool.map(self.mp_simulation_function, num_tasks)

    def plot_figures(self):
        with tqdm(total=self.simulations.__len__(), desc='Plotting figures', ncols=80, leave=False,
                  disable=False) as pbar:
            for sim in self.simulations:
                sim.plot_figure()
                pbar.update()

    def print_simulation_configs(self):
        for idx, simulation in enumerate(self.simulations):
            print('\nSimulation {}\n-----------'.format(str(idx)))
            simulation.print_model_config()
            simulation.print_control_config()


# %% Main program
if __name__ == '__main__':
    start_time = time()

    # Syntax for simulating just one model
    # sim = Simulation(vec_im_2kW.ctrl, vec_im_2kW.mdl, vec_im_2kW.base, vec_im_2kW.pars, 'ctrl_vector_im_2kW')
    # sim.simulate()
    # sim.plot_figure()
    # plt.show()

    # Syntax for simulating several models simultaneously
    # Initialize a simulation environment
    Simulator = SimEnv()

    # Add some simulation instances
    Simulator.add_sim(vec_im_2kW.ctrl, vec_im_2kW.mdl, vec_im_2kW.base, vec_im_2kW.pars, 'ctrl_vector_im_2kW')
    Simulator.add_sim(vec_pmsm_2kW.ctrl, vec_pmsm_2kW.mdl, vec_pmsm_2kW.base, vec_pmsm_2kW.pars, 'ctrl_vector_pmsm_2kW')
    Simulator.add_sim(vec_syrm_7kW.ctrl, vec_syrm_7kW.mdl, vec_syrm_7kW.base, vec_syrm_7kW.pars, 'ctrl_vector_syrm_7kW')
    Simulator.add_sim(vhz_im_2kW.ctrl, vhz_im_2kW.mdl, vhz_im_2kW.base, vhz_im_2kW.pars, 'ctrl_vhz_im_2kW')

    # Prints out the configs for each simulation
    # Simulator.print_simulation_configs()

    # To disable the progress bars (for example if they don't work properly on your IDE, set "disable=True" in the
    # keyword arguments on line 55
    Simulator.simulate_all()

    print('Execution time: {:.2f} s'.format(time() - start_time))

    Simulator.plot_figures()

    # Show all plotted figures
    plt.show()

# %%
