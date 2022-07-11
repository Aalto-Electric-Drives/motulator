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
        # Simulation loop
        while self.mdl.t0 <= self.t_stop:

            # Run the digital controller
            T_s, d_abc_ref = self.ctrl(self.mdl)

            # Model the computational delay
            d_abc = self.ctrl.delay(d_abc_ref)

            # Arrays of switching states and their durations
            t_steps, q = self.mdl.conv.pwm(T_s, d_abc)

            # Loop over the sampling period T_s
            for i, t_step in enumerate(t_steps):

                if t_step > 0:
                    # Update the switching state
                    self.mdl.conv.q = q[i]

                    # Get initial values
                    x0 = self.mdl.get_initial_values()

                    # Integrate over t_span
                    t_span = (self.mdl.t0, self.mdl.t0+t_step)
                    sol = solve_ivp(self.mdl.f, t_span, x0, max_step=max_step)

                    # Set the new initial values (last points of the solution)
                    t0_new, x0_new = t_span[-1], sol.y[:, -1]
                    self.mdl.set_initial_values(t0_new, x0_new)

                    # Save the solution
                    sol.q = len(sol.t)*[self.mdl.conv.q]
                    self.mdl.save(sol)

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
