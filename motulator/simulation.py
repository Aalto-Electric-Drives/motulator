# pylint: disable=invalid-name
"""Simulation environment."""

import numpy as np
from scipy.integrate import solve_ivp
from scipy.io import savemat

from motulator.helpers import abc2complex


# %%
class Delay:
    """
    Computational delay.

    This models the computational delay as a ring buffer.

    Parameters
    ----------
    length : int, optional
        Length of the buffer in samples. The default is 1.

    """

    # pylint: disable=too-few-public-methods
    def __init__(self, length=1, elem=3):
        self.data = length*[elem*[0]]  # Creates a zero list

    def __call__(self, u):
        """
        Delay the input.

        Parameters
        ----------
        u : array_like, shape (elem,)
            Input array.

        Returns
        -------
        array_like, shape (elem,)
            Output array.

        """
        # Add the latest value to the end of the list
        self.data.append(u)
        # Pop the first element and return it
        return self.data.pop(0)


# %%
class CarrierCmp:
    """
    Carrier comparison.

    This computes the the switching states and their durations based on the
    duty ratios. Instead of searching for zero crossings, the switching
    instants are explicilty computed in the begininning of each sampling
    period, allowing faster simulations.

    Parameters
    ----------
    N : int, optional
        Amount of the counter quantization levels. The default is 2**12.

    """

    # pylint: disable=too-few-public-methods
    def __init__(self, N=2**12):
        self.N = N
        self.rising_edge = True  # Stores the carrier direction

    def __call__(self, T_s, d_abc):
        """
        Carrier comparison.

        Parameters
        ----------
        T_s : float
            Sampling period (either half or full carrier period).
        d_abc : array_like of floats, shape (3,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        t_steps : ndarray, shape (4,)
            Switching state durations, `[t0, t1, t2, t3]`.
        q : complex ndarray, shape (4,)
            Switching state vectors, `[0, q1, q2, 0]`, where `q1` and `q2` are
            active vectors.

        """
        # Quantize the duty ratios to N levels
        d_abc = np.round(self.N*np.asarray(d_abc))/self.N

        # Normalized switching instants and switching states
        if self.rising_edge:
            # Normalized switching instants: t_n = [0, t_n1, t_n2, t_n3]
            t_n = np.append(0, np.sort(1 - d_abc))
            # States: q_abc = [[0, 0, 0], [q_abc1], [q_abc2], [1, 1, 1]]
            q_abc = (t_n[:, np.newaxis] >= 1 - d_abc).astype(int)
        else:
            t_n = np.append(0, np.sort(d_abc))
            q_abc = (t_n[:, np.newaxis] < d_abc).astype(int)

        # Durations of switching states: t_steps = [t0, t1, t2, t3]
        t_steps = T_s*np.diff(t_n, append=1)
        # Note: No switching (e.g. d_a == 0 or d_a == 1) or simultaneous
        # switchings (e.g d_a == d_b) lead to some zero steps.

        # Array of the switching state space vectors: q = [0, q1, q2, 0]
        q = abc2complex(q_abc.T)

        # Change the carrier direction for the next call
        self.rising_edge = not self.rising_edge

        # If needed, alternatively q_abc could be returned
        return t_steps, q


# %%
def zoh(T_s, d_abc):
    """
    Zero-order hold of the duty ratios over the sampling period.

    Parameters
    ----------
    T_s : float
        Sampling period.
    d_abc : array_like of floats, shape (3,)
        Duty ratios in the range [0, 1].

    Returns
    -------
    t_steps : ndarray, shape (1,)
        Sampling period as an array compatible with the solver.
    q : complex ndarray, shape (1,)
        Duty ratio vector as an array compatible with the solver.

    """
    # Shape the output arrays to be compatible with the solver
    t_steps = np.array([T_s])
    q = np.array([abc2complex(d_abc)])
    return t_steps, q


# %%
class Simulation:
    """
    Simulation environment.

    Each simulation object has a system model object and a controller object.

    Parameters
    ----------
    mdl : InductionMotorDrive | SynchronousMotorDrive
        Continuous-time system model.
    ctrl : SynchronousMotorVectorCtrl | InductionMotorVectorCtrl |
        InductionMotorVHzCtrl
        Discrete-time controller.
    delay : int, optional
        Amount of computational delays. The default is 1.
    enable_pwm : bool, optional
        Enable carrier comparison. The default is False.
    t_stop : float, optional
        Simulation stop time. The default is 1.

    """

    def __init__(
            self, mdl=None, ctrl=None, delay=1, enable_pwm=False, t_stop=1):
        # pylint: disable=too-many-arguments
        self.mdl = mdl
        self.ctrl = ctrl
        self.delay = Delay(delay)
        self.t_stop = t_stop
        if enable_pwm:
            self.pwm = CarrierCmp()
        else:
            self.pwm = zoh

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

            # Computational delay model
            d_abc = self.delay(d_abc_ref)

            # Carrier comparison
            t_steps, q = self.pwm(T_s, d_abc)

            # Loop over the sampling period T_s
            for i, t_step in enumerate(t_steps):

                if t_step > 0:
                    # Update the switching state
                    self.mdl.conv.q = q[i]

                    # Get initial values
                    x0 = self.mdl.get_initial_values()

                    # Integrate over t_span
                    t_span = (self.mdl.t0, self.mdl.t0 + t_step)
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
        savemat(name + '_mdl_data' + '.mat', self.mdl.data)
        savemat(name + '_ctrl_data' + '.mat', self.ctrl.data)
