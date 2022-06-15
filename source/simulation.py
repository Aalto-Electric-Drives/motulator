# pylint: disable=C0103
"""
This module contains the simulation class.

"""

# %% Imports
import numpy as np
from scipy.io import savemat
from scipy.integrate import solve_ivp
from helpers import plot, plot_pu, plot_pu_extra, save_plot


# %%
def state_machine(mdl, ctrl):
    """
    State machine for running the simulation.

    This runs the digital controller and calls the solver for integrating the
    continuous-time system model.

    Parameters
    ----------
    mdl : object
        System model.
    ctrl : object
        Controller.

    """
    def sensorless_ctrl():
        while mdl.t0 <= mdl.t_stop:
            # Sample the phase currents and the DC-bus voltage
            i_s_abc_meas = mdl.motor.meas_currents()
            u_dc_meas = mdl.conv.meas_dc_voltage()
            # Get the speed reference
            w_m_ref = mdl.speed_ref(mdl.t0)
            # Run the digital controller
            d_abc_ref, T_s = ctrl(w_m_ref, i_s_abc_meas, u_dc_meas)
            # Model the computational delay
            d_abc = ctrl.delay(d_abc_ref)
            # Simulate the continuous-time plant model over the sampling period
            solve(mdl, d_abc, [mdl.t0, mdl.t0+T_s])

    def sensored_ctrl():
        while mdl.t0 <= mdl.t_stop:
            # Sample the phase currents and the DC-bus voltage
            i_s_abc_meas = mdl.motor.meas_currents()
            u_dc_meas = mdl.conv.meas_dc_voltage()
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
            d_abc = ctrl.delay(d_abc_ref)
            # Simulate the continuous-time plant model over the sampling period
            solve(mdl, d_abc, [mdl.t0, mdl.t0 + T_s])

    # Run the state machine
    if ctrl.sensorless:
        sensorless_ctrl()
    else:
        sensored_ctrl()


# %%
def solve(mdl, d_abc, t_span, max_step=np.inf):
    """
    Solve the continuous-time model over t_span.

    Parameters
    ----------
    mdl : object
        Model to be simulated.
    d_abc : array_like of floats, shape (3,)
        Duty ratio references in the interval [0, 1].
    t_span : 2-tuple of floats
        Interval of integration (t0, tf). The solver starts with t=t0 and
        integrates until it reaches t=tf.
    max_step : float, optional
        Max step size of the solver. The default is inf.

    """
    def run_solver(t_span):
        # Skip possible zero time spans
        if t_span[-1] > t_span[0]:
            # Get initial values
            x0 = mdl.get_initial_values()
            # Integrate
            sol = solve_ivp(mdl.f, t_span, x0, max_step=max_step)
            # Set the new initial values (last points of the solution)
            t0_new, x0_new = t_span[-1], sol.y[:, -1]
            mdl.set_initial_values(t0_new, x0_new)
            # Data logging
            # Switching state vector is constant
            sol.q = len(sol.t)*[mdl.conv.q]
            mdl.save(sol)

    # Sampling period
    T_s = t_span[-1] - t_span[0]
    # Compute the normalized switching spans and the corresponding states
    tn_sw, q_sw = mdl.conv.pwm(d_abc)
    # Convert the normalized switching spans to seconds
    t_sw = t_span[0] + T_s*tn_sw
    # Loop over the switching time spans
    for i, t_sw_span in enumerate(t_sw):
        # Update the switching state vector (constant over the time span)
        mdl.conv.q = q_sw[i]
        # Run the solver
        run_solver(t_sw_span)


# %%
class Simulation:
    """
    Create a simulation object.

    Each simulation has a control module and a plant module.

    """

    def __init__(self, mdl, ctrl, base=None, name='sim', print_opts=True):
        """
        Parameters
        ----------
        mdl : mdl
            System model.
        ctrl : ctrl
            Controller.
        base : dataclass
            Base values for plotting figures.
        name : str, optional
            Name for the simulation instance.

        """
        self.mdl = mdl
        self.ctrl = ctrl
        self.base = base
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
        self.mdl.post_process()
        self.ctrl.post_process()

    def print_model_config(self):
        """
        Print the model configuration data.

        """
        print('\n--- Model ---')
        print(self.mdl)

    def print_control_config(self):
        """
        Print the control system data, speed reference and load reference.

        """
        print('\n--- Control ---')
        print(self.ctrl)

    def print_simulation_profile(self):
        """
        Print the simulation profiles.

        """
        print('\n--- Speed reference ---')
        print(self.mdl.speed_ref)
        print('--- External load torque ---')
        print(self.mdl.mech.tau_L_ext)

    def plot_figure(self, save=False):
        """
        Plot the simulation results.

        A wrapper for the plot function in helpers.py.

        """
        if self.base is not None:
            plot_pu(self.mdl.data, self.ctrl.data, self.base)
            try:
                plot_pu_extra(self.mdl.data, self.ctrl.data, self.base)
            except AttributeError:
                pass
        else:
            plot(self.mdl.data, self.ctrl.data)
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

        savemat(self.name+'.mat', data)
