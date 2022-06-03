# pylint: disable=C0103
"""
This module includes tools for simulating the induction motor model

"""
# %% Imports
from model.interfaces import solve
from multiprocessing import Pool, RLock#, freeze_support
import copy
import numpy as np
from helpers import plot

try:
    from tqdm import tqdm, trange  # Can play around with tqdm.gui aswell, prints progress bars using matplotlib
except ImportError:
    print('tqdm could not be imported, because the tqdm library is missing')


# %%
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

    def print_profiles_config(self):
        with np.printoptions(precision=1, suppress=True):
            print('Profiles'
                  '\n--------',
                  '\nSpeed reference:'
                  '\n    {}'.format(self.mdl.speed_ref),
                  '\nExternal load torque:',
                  '\n    {}'.format(self.mdl.mech.tau_L_ext))

    def print_control_config(self):
        # %% Print the control system data, speed reference and load reference
        print(self.ctrl)

    def print_model_config(self):
        # %% Print the induction motor model data
        print(self.mdl)


# %%
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
            print('\nSimulation {}\n-----------'.format(str(idx+1)))
            simulation.print_model_config()
            simulation.print_control_config()
            simulation.print_profiles_config()
