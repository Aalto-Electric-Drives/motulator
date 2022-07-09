# pylint: disable=C0103
"""
This module contains the simulation class.

"""

# %% Imports
import numpy as np
from scipy.integrate import solve_ivp
from scipy.io import savemat


# %%
class Simulation:
    """
    Simulation class.

    Each simulation object has a system model object and a controller object.

    """

    def __init__(self, mdl=None, ctrl=None, base=None, t_stop=1):
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
        t_stop : float, optional
            Simulation stop time. The default is 1.

        """
        self.mdl = mdl
        self.ctrl = ctrl
        self.base = base
        self.t_stop = t_stop

    def simulate(self, max_step=np.inf):
        """
        Solve the continuous-time model and call the discrete-time controller.

        Parameters
        ----------
        max_step : float, optional
            Max step size of the solver. The default is inf.

        Notes
        -----
        Other options of solve_ivp could be easily changed if needed, but, for
        simplicity, only max_step is included as an option of this method.

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

        # Simulation loop
        while self.mdl.t0 <= self.t_stop:
            # Run the digital controller
            d_abc_ref, T_s = self.ctrl(self.mdl)
            # Model the computational delay
            d_abc = self.ctrl.delay(d_abc_ref)
            # Simulate the continuous-time model over the sampling period
            solve(d_abc, [self.mdl.t0, self.mdl.t0+T_s])

        # Call the post-processing functions
        self.mdl.post_process()
        self.ctrl.post_process()

    def save_mat(self, name='sim'):
        """
        Save the simulation data into MATLAB .mat files.

        Parameters
        ----------
        name : str, optional
            Name for the simulation instance. The default is 'sim'.

        """
        savemat(name+'_mdl_data'+'.mat', self.mdl.data)
        savemat(name+'_ctrl_data'+'.mat', self.ctrl.data)
