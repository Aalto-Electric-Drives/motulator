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
class Simulation:
    """
    Simulation object.

    Each simulation has a control module and a plant module.

    """

    def __init__(self, mdl, ctrl, base=None, name='sim', print_opts=True):
        """
        Parameters
        ----------
        mdl : (InductionMotorDrive | SynchronousMotorDrive)
            Continuous-time system model.
        ctrl : (SynchronousMotorVectorCtrl | InductionMotorVectorCtrl |
                InductionMotorVHzCtrl)
            Discrete-time controller.
        base : BaseValues, optional
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

    def simulate(self, max_step=np.inf):
        """
        Solve the continuous-time model and call the discrete-time controller.

        Parameters
        ----------
        max_step : float, optional
            Max step size of the solver. The default is inf.

        Notes
        -----
        Other options of solve_ivp() could be easily changed if needed, but,
        for simplicity, only max_step is included as an option of this method.

        """

        def solve(d_abc, t_span):
            """
            Solve the continuous-time model over t_span.

            Parameters
            ----------
            d_abc : array_like of floats, shape (3,)
                Duty ratio references in the interval [0, 1].
            t_span : 2-tuple of floats
                Interval of integration (t0, tf). The solver starts with t=t0
                and integrates until it reaches t=tf.

            """
            # Sampling period
            T_s = t_span[-1] - t_span[0]
            # Compute the normalized switching spans and corresponding states
            tn_sw, q_sw = self.mdl.conv.pwm(d_abc)
            # Convert the normalized switching spans to seconds
            t_sw = t_span[0] + T_s*tn_sw
            # Loop over the switching time spans
            for i, t_sw_span in enumerate(t_sw):
                # Update the switching states (constant over the time span)
                self.mdl.conv.q = q_sw[i]
                # Skip possible zero time spans
                if t_sw_span[-1] > t_sw_span[0]:
                    # Get initial values
                    x0 = self.mdl.get_initial_values()
                    # Integrate
                    sol = solve_ivp(self.mdl.f, t_sw_span, x0,
                                    max_step=max_step)
                    # Set the new initial values (last points of the solution)
                    t0_new, x0_new = t_sw_span[-1], sol.y[:, -1]
                    self.mdl.set_initial_values(t0_new, x0_new)
                    # Data logging
                    sol.q = len(sol.t)*[self.mdl.conv.q]
                    self.mdl.save(sol)

        def sensorless_ctrl():
            while self.mdl.t0 <= self.mdl.t_stop:
                # Sample the phase currents and the DC-bus voltage
                i_s_abc_meas = self.mdl.motor.meas_currents()
                u_dc_meas = self.mdl.conv.meas_dc_voltage()
                # Get the speed reference
                w_m_ref = self.mdl.speed_ref(self.mdl.t0)
                # Run the digital controller
                d_abc_ref, T_s = self.ctrl(w_m_ref, i_s_abc_meas, u_dc_meas)
                # Model the computational delay
                d_abc = self.ctrl.delay(d_abc_ref)
                # Simulate the continuous-time model over the sampling period
                solve(d_abc, [self.mdl.t0, self.mdl.t0+T_s])

        def sensored_ctrl():
            while self.mdl.t0 <= self.mdl.t_stop:
                # Sample the phase currents and the DC-bus voltage
                i_s_abc_meas = self.mdl.motor.meas_currents()
                u_dc_meas = self.mdl.conv.meas_dc_voltage()
                # Measure the rotor position (not used in the case of an IM)
                theta_M_meas = self.mdl.mech.meas_position()
                # Measure the rotor speed
                w_M_meas = self.mdl.mech.meas_speed()
                # Get the speed reference
                w_m_ref = self.mdl.speed_ref(self.mdl.t0)
                # Run the digital controller
                d_abc_ref, T_s = self.ctrl(w_m_ref, i_s_abc_meas, u_dc_meas,
                                           w_M_meas, theta_M_meas)
                # Model the computational delay
                d_abc = self.ctrl.delay(d_abc_ref)
                # Simulate the continuous-time model over the sampling period
                solve(d_abc, [self.mdl.t0, self.mdl.t0+T_s])

        # Choose the state machine
        if self.ctrl.sensorless:
            sensorless_ctrl()
        else:
            sensored_ctrl()

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
        print('\n--- External load torque ---')
        print(self.mdl.mech.tau_L_ext)

    def plot_figure(self, save=False, extra=False):
        """
        Plot the simulation results.

        A wrapper for the plot function in helpers.py.

        """
        if self.base is not None:
            plot_pu(self.mdl.data, self.ctrl.data, self.base)
            if extra:
                # Try plotting extra figures
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
