"""Simulation environment."""

from abc import ABC, abstractmethod
from types import SimpleNamespace
import numpy as np
from scipy.integrate import solve_ivp
from scipy.io import savemat
from motulator._helpers import abc2complex


# %%
class Delay:
    """
    Computational delay modeled as a ring buffer.

    Parameters
    ----------
    length : int, optional
        Length of the buffer in samples. The default is 1.

    """

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
class CarrierComparison:
    """
    Carrier comparison.

    This computes the the switching states and their durations based on the
    duty ratios. Instead of searching for zero crossings, the switching
    instants are explicitly computed in the beginning of each sampling period,
    allowing faster simulations.

    Parameters
    ----------
    N : int, optional
        Amount of the counter quantization levels. The default is 2**12.
    return_complex : bool, optional
        Complex switching state space vectors are returned if True. Otherwise
        phase switching states are returned. The default is True.

    Examples
    --------
    >>> from motulator.model import CarrierComparison
    >>> carrier_cmp = CarrierComparison(return_complex=False)
    >>> # First call gives rising edges
    >>> t_steps, q_c_abc = carrier_cmp(1e-3, [.4, .2, .8])
    >>> # Durations of the switching states
    >>> t_steps
    array([0.00019995, 0.00040015, 0.00019995, 0.00019995])
    >>> # Switching states
    >>> q_c_abc
    array([[0, 0, 0],
           [0, 0, 1],
           [1, 0, 1],
           [1, 1, 1]])
    >>> # Second call gives falling edges
    >>> t_steps, q_c_abc = carrier_cmp(.001, [.4, .2, .8])
    >>> t_steps
    array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
    >>> q_c_abc
    array([[1, 1, 1],
           [1, 0, 1],
           [0, 0, 1],
           [0, 0, 0]])
    >>> # Sum of the step times equals T_s
    >>> np.sum(t_steps)
    0.001
    >>> # 50% duty ratios in all phases
    >>> t_steps, q_c_abc = carrier_cmp(1e-3, [.5, .5, .5])
    >>> t_steps
    array([0.0005, 0.    , 0.    , 0.0005])
    >>> q_c_abc
    array([[0, 0, 0],
           [0, 0, 0],
           [0, 0, 0],
           [1, 1, 1]])

    """

    def __init__(self, N=2**12, return_complex=True):
        self.N = N
        self.return_complex = return_complex
        self._rising_edge = True  # Stores the carrier direction

    def __call__(self, T_s, d_c_abc):
        """
        Compute the switching state durations and vectors.

        Parameters
        ----------
        T_s : float
            Half carrier period (s).
        d_c_abc : array_like of floats, shape (3,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        t_steps : ndarray, shape (4,)
            Switching state durations (s), `[t0, t1, t2, t3]`.
        q_cs : complex ndarray, shape (4,)
            Switching state vectors, `[q0, q1, q2, q3]`, where `q1` and `q2`
            are active vectors.

        Notes
        -----
        No switching (e.g. `d_a == 0` or `d_a == 1`) or simultaneous switching
        (e.g. `d_a == d_b`) lead to zeroes in `t_steps`.

        """
        # Quantize the duty ratios to N levels
        d_c_abc = np.round(self.N*np.asarray(d_c_abc))/self.N

        # Assume falling edge and compute the normalized switching instants:
        t_n = np.append(0, np.sort(d_c_abc))
        # Compute the corresponding switching states:
        q_c_abc = (t_n[:, np.newaxis] < d_c_abc).astype(int)

        # Durations of switching states
        t_steps = T_s*np.diff(t_n, append=1)

        # Flip the sequence if rising edge
        if self._rising_edge:
            t_steps = np.flip(t_steps)
            q_c_abc = np.flipud(q_c_abc)

        # Change the carrier direction for the next call
        self._rising_edge = not self._rising_edge

        return ((t_steps, abc2complex(q_c_abc.T)) if self.return_complex else
                (t_steps, q_c_abc))


# %%
def zoh(T_s, d_c_abc):
    """
    Zero-order hold of the duty ratios over the sampling period.

    Parameters
    ----------
    T_s : float
        Sampling period.
    d_c_abc : array_like of floats, shape (3,)
        Duty ratios in the range [0, 1].

    Returns
    -------
    t_steps : ndarray, shape (1,)
        Sampling period as an array compatible with the solver.
    q_cs : complex ndarray, shape (1,)
        Duty ratio vector as an array compatible with the solver.

    """
    # Shape the output arrays to be compatible with the solver
    t_steps = np.array([T_s])
    q_cs = np.array([abc2complex(d_c_abc)])
    return t_steps, q_cs


# %%
class Simulation:
    """
    Simulation environment.

    Each simulation object has a system model object and a controller object.

    Parameters
    ----------
    mdl : Model 
        Continuous-time system model.
    ctrl : Ctrl
        Discrete-time controller.

    """

    def __init__(self, mdl=None, ctrl=None):
        self.mdl = mdl
        self.ctrl = ctrl

    def simulate(self, t_stop=1, max_step=np.inf):
        """
        Solve the continuous-time model and call the discrete-time controller.

        Parameters
        ----------
        t_stop : float, optional
            Simulation stop time. The default is 1.
        max_step : float, optional
            Max step size of the solver. The default is inf.

        Notes
        -----
        Other options of `solve_ivp` could be easily used if needed, but, for
        simplicity, only `max_step` is included as an option of this method.

        """
        try:
            self._simulation_loop(t_stop, max_step)
        except FloatingPointError:
            print(f"Invalid value encountered at {self.mdl.t0:.2f} seconds.")
        # Post-process the solution data
        self.mdl.post_process()
        self.ctrl.post_process()

    @np.errstate(invalid="raise")
    def _simulation_loop(self, t_stop, max_step):
        """Run the main simulation loop."""
        while self.mdl.t <= t_stop:

            # Run the digital controller
            T_s, ref_d_c_abc = self.ctrl(self.mdl)

            # Computational delay model
            d_c_abc = self.mdl.delay(ref_d_c_abc)

            # Carrier comparison
            t_steps, q_cs = self.mdl.pwm(T_s, d_c_abc)

            # Loop over the sampling period T_s
            for i, t_step in enumerate(t_steps):

                if t_step > 0:
                    # Update the converter switching state
                    self.mdl.converter.inp.q_cs = q_cs[i]

                    # Get initial values
                    state0 = self.mdl.get_initial_values()

                    # Integrate over t_span
                    t_span = (self.mdl.t, self.mdl.t + t_step)
                    sol = solve_ivp(
                        self.mdl.rhs, t_span, state0, max_step=max_step)

                    # Set the new initial values (last points of the solution)
                    t0_new, state0_new = t_span[-1], sol.y[:, -1]
                    self.mdl.set_initial_values(t0_new, state0_new)

                    # Save the solution
                    sol.q_cs = len(sol.t)*[q_cs[i]]
                    self.mdl.save(sol)

    def save_mat(self, name="sim"):
        """
        Save the simulation data into MATLAB .mat files.

        Parameters
        ----------
        name : str, optional
            Name for the simulation instance. The default is `sim`.

        """
        savemat(name + "_mdl_data" + ".mat", self.mdl.data)
        savemat(name + "_ctrl_data" + ".mat", self.ctrl.data)


# %%
class Model(ABC):
    """
    Base class for continuous-time system models.

    This base class is a template for a system model that interconnects the 
    subsystems and provides an interface to the solver. 

    Parameters
    ----------
    pwm : zoh | CarrierComparison, optional
        Zero-order hold of duty ratios or carrier comparison. If None, the 
        default is `zoh`.
    delay : int, optional
        Amount of computational delays. The default is 1.

    """

    def __init__(self, pwm=None, delay=1):
        self.delay = Delay(delay)
        self.pwm = zoh if pwm is None else pwm
        self.t = 0
        self.converter = None
        self.subsystems = []  # Contains the list of subsystems
        self.sol_t = []
        self.data = SimpleNamespace()

    def get_initial_values(self):
        """Get initial values of all subsystems."""
        state0 = []
        for subsystem in self.subsystems:
            if hasattr(subsystem, "state"):
                state0 += list(vars(subsystem.state).values())

        return state0

    def set_initial_values(self, t0, state0):
        """Set initial values to all subsystems."""
        self.t = t0
        index = 0
        for subsystem in self.subsystems:
            if hasattr(subsystem, "state"):
                for attr in vars(subsystem.state):
                    value = state0[index]
                    setattr(subsystem.state, attr, value)
                    index += 1

    def set_states(self, state_list):
        """Set the states in all subsystems."""
        index = 0
        for subsystem in self.subsystems:
            if hasattr(subsystem, "state"):
                for attr in vars(subsystem.state):
                    setattr(subsystem.state, attr, state_list[index])
                    index += 1

    def set_outputs(self, t):
        """Compute the output variables."""
        for subsystem in self.subsystems:
            if hasattr(subsystem, "set_outputs"):
                subsystem.set_outputs(t)

    @abstractmethod
    def interconnect(self, t):
        """Interconnect the subsystems."""

    def rhs(self, t, state_list):
        """Compute the complete state derivative list for the solver."""
        # Get the states from the list and set them to the subsystems
        self.set_states(state_list)

        # Set the outputs for the interconnections and for the rhs
        self.set_outputs(t)

        # Interconnections
        self.interconnect(t)

        # State derivatives
        rhs_list = []
        for subsystem in self.subsystems:
            if hasattr(subsystem, "rhs"):
                subsystem_rhs = subsystem.rhs()
                rhs_list += subsystem_rhs

        # List of state derivatives
        return rhs_list

    def save(self, sol):
        """Save the solution."""
        self.sol_t.extend(sol.t)
        self.converter.sol_q_cs.extend(sol.q_cs)

        index = 0
        for subsystem in self.subsystems:
            if hasattr(subsystem, "sol_states"):
                for attr in vars(subsystem.sol_states):
                    subsystem.sol_states.__dict__[attr].extend(sol.y[index])
                    index += 1

    def post_process_states(self):
        """Transform the lists to the ndarray format and post-process them."""
        self.data.t = np.asarray(self.sol_t)
        self.converter.data.q_cs = np.asarray(self.converter.sol_q_cs)

        for subsystem in self.subsystems:
            if hasattr(subsystem, "sol_states"):
                for key, value in vars(subsystem.sol_states).items():
                    setattr(subsystem.data, key, np.asarray(value))
            if hasattr(subsystem, "post_process_states"):
                subsystem.post_process_states()

    def post_process_with_inputs(self):
        """Post-process after the inputs have been added."""
        for subsystem in self.subsystems:
            if hasattr(subsystem, "post_process_with_inputs"):
                subsystem.data.t = self.data.t
                subsystem.post_process_with_inputs()


class Subsystem(ABC):
    """Base class for subsystems."""

    def __init__(self):
        # States, inputs, and outputs
        self.state = SimpleNamespace()
        self.inp = SimpleNamespace()
        self.out = SimpleNamespace()
        # Store the solutions in these lists
        self.sol_states = SimpleNamespace()
        # For post-processed data
        self.data = SimpleNamespace()
