# pylint: disable=C0103
"""
This module contains the interfaces for the solver.

"""
import numpy as np
from scipy.integrate import solve_ivp
from helpers import abc2complex


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
    # Common code
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
            sol.q = len(sol.t)*[mdl.q]  # Switching state vector is constant
            mdl.datalog.save(sol)

    if not mdl.pwm.enabled:
        # Update the duty ratio space vector (constant over the time span)
        mdl.q = abc2complex(d_abc)
        # Run the solver
        run_solver(t_span)
    else:
        # Sampling period
        T_s = t_span[-1] - t_span[0]
        # Compute the normalized switching spans and the corresponding states
        tn_sw, q_sw = mdl.pwm(d_abc)
        # Convert the normalized switching spans to seconds
        t_sw = t_span[0] + T_s*tn_sw
        # Loop over the switching time spans
        for i, t_sw_span in enumerate(t_sw):
            # Update the switching state vector (constant over the time span)
            mdl.q = q_sw[i]
            # Run the solver
            run_solver(t_sw_span)


# %%
class PWM:
    """
    Carrier comparison for pulse-width modulation.

    This implements carrier comparison for three-phase PWM. The switching
    instants and the switching states are explicitly and exactly computed from
    the duty ratios. The switching instants can be used in the solver.

    """

    # pylint: disable=R0903
    def __init__(self, enabled=True, N=2**12):
        """
        Parameters
        ----------
        N : int, optional
            Amount of PWM quantization levels. The default is 2**12.
        enabled : Boolean, optional
            PMW enabled. The default is True.

        """
        self.N = N
        self.falling_edge = False
        self.enabled = enabled

    def __call__(self, d_abc):
        """
        Compute the normalized switching instants and the switching states.

        Parameters
        ----------
        d_abc : array_like of floats, shape (3,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        tn_sw : ndarray, shape (4,2)
            Normalized switching instants, tn_sw = [0, t1, t2, t3, 1].
        q : complex ndarray, shape (4,)
            Switching state space vectors corresponding to the switching
            instants. For example, the switching state q[1] is applied
            at the interval tn_sw[1].

        Notes
        -----
        Switching instants t_sw split the sampling period T_s into
        four spans. No switching (e.g. da = 0 or da = 1) or simultaneous
        switching instants (e.g da == db) lead to zero spans, i.e.,
        t_sw[i] == t_sw[i].

        """
        # Quantize the duty ratios to N levels
        d_abc = np.round(self.N*np.asarray(d_abc))/self.N
        # Initialize the normalized switching instant array
        tn_sw = np.zeros((4, 2))
        tn_sw[3, 1] = 1
        # Could be understood as a carrier comparison
        if self.falling_edge:
            # Normalized switching instants (zero crossing instants)
            tn_sw[1:4, 0] = np.sort(d_abc)
            tn_sw[0:3, 1] = tn_sw[1:4, 0]
            # Compute the switching state array
            q_abc = (tn_sw[:, 0] < d_abc[:, np.newaxis]).astype(int)
        else:
            # Rising edge
            tn_sw[1:4, 0] = np.sort(1 - d_abc)
            tn_sw[0:3, 1] = tn_sw[1:4, 0]
            q_abc = (tn_sw[:, 0] >= 1 - d_abc[:, np.newaxis]).astype(int)
        # Change the carrier direction for the next call
        self.falling_edge = not self.falling_edge
        # Switching state space vector
        q = abc2complex(q_abc)
        return tn_sw, q

    def __str__(self):
        if not self.enabled:
            desc = 'PWM model:\n    disabled\n'
        else:
            desc = ('PWM model:\n'
                    '    {} quantization levels\n').format(self.N)
        return desc


# %%
class Delay:
    """
    Computational delay.

    This models the compuational delay as a ring buffer.

    """

    # pylint: disable=R0903
    def __init__(self, length=1, elem=3):
        """
        Parameters
        ----------
        length : int, optional
            Length of the buffer in samples. The default is 1.

        """
        self.data = length*[elem*[0]]  # Creates a zero list

    def __call__(self, u):
        """
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

    def __str__(self):
        length = len(self.data)
        desc = (('Computational delay:\n    {} sampling periods\n')
                .format(length))
        return desc
