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
    >>> from motulator.simulation import CarrierCmp
    >>> carrier_cmp = CarrierCmp(return_complex=False)
    >>> # First call gives rising edges
    >>> t_steps, q_abc = carrier_cmp(1e-3, [.4, .2, .8])
    >>> # Durations of the switching states
    >>> t_steps
    array([0.00019995, 0.00040015, 0.00019995, 0.00019995])
    >>> # Switching states
    >>> q_abc
    array([[0, 0, 0],
           [0, 0, 1],
           [1, 0, 1],
           [1, 1, 1]])
    >>> # Second call gives falling edges
    >>> t_steps, q_abc = carrier_cmp(.001, [.4, .2, .8])
    >>> t_steps
    array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
    >>> q_abc
    array([[1, 1, 1],
           [1, 0, 1],
           [0, 0, 1],
           [0, 0, 0]])
    >>> # Sum of the step times equals T_s
    >>> np.sum(t_steps)
    0.001
    >>> # 50% duty ratios in all phases
    >>> t_steps, q_abc = carrier_cmp(1e-3, [.5, .5, .5])
    >>> t_steps
    array([0.0005, 0.    , 0.    , 0.0005])
    >>> q_abc
    array([[0, 0, 0],
           [0, 0, 0],
           [0, 0, 0],
           [1, 1, 1]])

    """

    # pylint: disable=too-few-public-methods
    def __init__(self, N=2**12, return_complex=True):
        self.N = N
        self.return_complex = return_complex
        self.rising_edge = True  # Stores the carrier direction

    def __call__(self, T_s, d_abc):
        """
        Compute the switching state durations and vectors.

        Parameters
        ----------
        T_s : float
            Half carrier period.
        d_abc : array_like of floats, shape (3,)
            Duty ratios in the range [0, 1].

        Returns
        -------
        t_steps : ndarray, shape (4,)
            Switching state durations, `[t0, t1, t2, t3]`.
        q : complex ndarray, shape (4,)
            Switching state vectors, `[q0, q1, q2, q3]`, where `q1` and `q2`
            are active vectors.

        Notes
        -----
        No switching (e.g. `d_a == 0` or `d_a == 1`) or simultaneous switchings
        (e.g. `d_a == d_b`) lead to zeroes in `t_steps`.

        """
        # Quantize the duty ratios to N levels
        d_abc = np.round(self.N*np.asarray(d_abc))/self.N

        # Assume falling edge and compute the normalized switching instants:
        t_n = np.append(0, np.sort(d_abc))
        # Compute the correponding switching states:
        q_abc = (t_n[:, np.newaxis] < d_abc).astype(int)

        # Durations of switching states
        t_steps = T_s*np.diff(t_n, append=1)

        # Flip the sequence if rising edge
        if self.rising_edge:
            t_steps = np.flip(t_steps)
            q_abc = np.flipud(q_abc)

        # Change the carrier direction for the next call
        self.rising_edge = not self.rising_edge

        return ((t_steps, abc2complex(q_abc.T)) if self.return_complex else
                (t_steps, q_abc))

 # %%
class SVPWM_2LV:
    '''

    This module is to provide Space Vector PWM modulation method
    for two-level inverter

    '''

    def __init__(self, enabled=True):

        '''

        All switching states in its sector are counterclockwise
        For each action, only one phase leg changes

        '''
        self.enabled = enabled
        self.alpha_gs = np.linspace(0, np.pi / 6, 1000, endpoint=True)
        self.M1 = np.sqrt((6 / np.pi) * (np.tan(np.pi / 6 - self.alpha_gs) + self.alpha_gs / ((np.cos(np.pi / 6 - self.alpha_gs)) ** 2)))
        self.M2 = np.sqrt((6 / np.pi) * (np.tan(np.pi / 6 - self.alpha_gs) + 4 * self.alpha_gs / 3))
        self.M1 = np.flipud(self.M1)
        self.sw_states = [
            [[0, 0, 0],
             [1, 0, 0],
             [1, 1, 0],
             [1, 1, 1]],

            [[1, 1, 1],
             [1, 1, 0],
             [0, 1, 0],
             [0, 0, 0]],

            [[0, 0, 0],
             [0, 1, 0],
             [0, 1, 1],
             [1, 1, 1]],

            [[1, 1, 1],
             [0, 1, 1],
             [0, 0, 1],
             [0, 0, 0]],

            [[0, 0, 0],
             [0, 0, 1],
             [1, 0, 1],
             [1, 1, 1]],

            [[1, 1, 1],
             [1, 0, 1],
             [1, 0, 0],
             [0, 0, 0]]]

    def __call__(self, u_ref, u_dc):
        """
        Parameters
        ----------
        u_dc : float
        Full dc link voltage

        u_ref : complex
        Reference voltage vector

        """
        u_alpha = u_ref.real
        u_beta = u_ref.imag
        u_ref_mag = np.sqrt(u_alpha ** 2 + u_beta ** 2)
        u_ref_ang = (np.angle(u_ref) + 2 * np.pi) % (2 * np.pi)  # Calculating the angels of U_ref (0-2pi)


        # Sector Selection
        sector = int(u_ref_ang / (np.pi / 3)) % 6  # 0-5

        # Sector Angle: Alpha
        alpha = u_ref_ang - sector * np.pi / 3

        # Effective time on two vector (Counterclockwise  a-b)

        # Overmodulation
        M = u_ref_mag / (u_dc / np.sqrt(3))
        if M > 1:
            if M > 1.05 :
                alpha_g = np.interp(M, self.M2, self.alpha_gs)
                if alpha <= alpha_g and alpha >= 0:
                    T_a = 1
                    T_b = 0
                    T_0 = 0
                elif alpha >= np.pi / 3 - alpha_g and alpha <= np.pi/3:
                    T_a = 0
                    T_b = 1
                    T_0 = 0
                else:
                    beta = np.pi/6*(alpha-alpha_g)/(np.pi/6-alpha_g)
                    T_a = np.sin(np.pi / 3 - beta) / np.sin(np.pi / 3 + beta)
                    T_b = np.sin(beta) / np.sin(np.pi / 3 + beta)
                    T_0 = 0
            else:
                alpha_g = np.interp(M, self.M1, self.alpha_gs)
                alpha_g = (1 - alpha_g / (np.pi / 6)) * (np.pi / 6)
                if alpha >= alpha_g and alpha <= np.pi / 3 - alpha_g:
                    T_a = np.sin(np.pi / 3 - alpha) / np.sin(np.pi / 3 + alpha)
                    T_b = np.sin(alpha) / np.sin(np.pi / 3 + alpha)
                    T_0 = 1 - T_a - T_b
                else:
                    u_ref_mag = 2/3*u_dc*np.sin(np.pi/3)/np.sin(2*np.pi/3-alpha_g)
                    T_a = np.sqrt(3) * np.sin(np.pi / 3 - alpha) * u_ref_mag / u_dc
                    T_b = np.sqrt(3) * np.sin(alpha) * u_ref_mag / u_dc
                    T_0 = 0
        else:
            T_a = np.sqrt(3) * np.sin(np.pi / 3 - alpha) * u_ref_mag / u_dc
            T_b = np.sqrt(3) * np.sin(alpha) * u_ref_mag / u_dc
            T_0 = 1 - T_a - T_b

        '''
        Calculating the switching sequence
        000-100-110-111-110-100-000
        000-010-110-111-110-010-000
        000-010-011-111-011-010-000
        000-001-011-111-011-001-000
        000-001-101-111-101-001-000
        000-100-101-111-101-100-000

        '''
        sw_sequ = np.zeros(7)
        sw_time = np.zeros(7)
        if (sector % 2 == 0):  # Sector 0 2 4
            sw_sequ[0] = 0
            sw_time[0] = 0 + 0.25 * T_0
            sw_sequ[1] = 1
            sw_time[1] = 0 + 0.25 * T_0 + 0.5 * T_a
            sw_sequ[2] = 2
            sw_time[2] = 0 + 0.25 * T_0 + 0.5 * T_a + 0.5 * T_b
            sw_sequ[3] = 3
            sw_time[3] = 0 + 1 - (0.25 * T_0 + 0.5 * T_a + 0.5 * T_b)
            sw_sequ[4] = 2
            sw_time[4] = 0 + 1 - (0.25 * T_0 + 0.5 * T_a)
            sw_sequ[5] = 1
            sw_time[5] = 0 + 1 - 0.25 * T_0
            sw_sequ[6] = 0
            sw_time[6] = 0 + 1
        else:  # Sector 1 3 5
            sw_sequ[0] = 3
            sw_time[0] = 0 + 0.25 * T_0
            sw_sequ[1] = 2
            sw_time[1] = 0 + 0.25 * T_0 + 0.5 * T_b
            sw_sequ[2] = 1
            sw_time[2] = 0 + 0.25 * T_0 + 0.5 * T_b + 0.5 * T_a
            sw_sequ[3] = 0
            sw_time[3] = 0 + 1 - (0.25 * T_0 + 0.5 * T_b + 0.5 * T_a)
            sw_sequ[4] = 1
            sw_time[4] = 0 + 1 - (0.25 * T_0 + 0.5 * T_b)
            sw_sequ[5] = 2
            sw_time[5] = 0 + 1 - 0.25 * T_0
            sw_sequ[6] = 3
            sw_time[6] = 0 + 1

        # Calculating switching states q
        qs = np.zeros((7,3))
        tn_sw = np.zeros((7, 2))
        tn_sw[1:7, 0] = sw_time[0:6]
        tn_sw[:, 1] = sw_time[:]
        for i in range(7):
            j=int(sw_sequ[i])
            qs[i] = self.sw_states[sector][j]
        return tn_sw, qs

# %%
class SVPWM_3LV:
    '''

    This module is to provide Space Vector PWM modulation method
    for three-level inverter.

    This method is based on the application report from Texas Instruments
    "Center-Aligned SVPWM Realization for 3- Phase 3- Level Inverter":
    https://www.ti.com/lit/pdf/sprabs6

    '''

    def __init__(self, enabled=True):
        """
        Parameters

        ----------
        enabled : Boolean, optional
            SVPMW enabled. The default is True.

        sw_states: ndarray, shape (6,4,3)
            Switching states of 6 sectors in 2-level SVPWM.
            The arrangement is counterclockwise and only one phase leg
            changes in each action, which minimizes the loss of inverters.

        """

        self.enabled = enabled
        self.sw_states = np.array([
            [[0, 0, 0],
             [1, 0, 0],
             [1, 1, 0],
             [1, 1, 1]],

            [[1, 1, 1],
             [1, 1, 0],
             [0, 1, 0],
             [0, 0, 0]],

            [[0, 0, 0],
             [0, 1, 0],
             [0, 1, 1],
             [1, 1, 1]],

            [[1, 1, 1],
             [0, 1, 1],
             [0, 0, 1],
             [0, 0, 0]],

            [[0, 0, 0],
             [0, 0, 1],
             [1, 0, 1],
             [1, 1, 1]],

            [[1, 1, 1],
             [1, 0, 1],
             [1, 0, 0],
             [0, 0, 0]]])

    #def __call__(self, u_ref, u_dc, delta_uc):
    def __call__(self, u_ref, u_dc):
        """
        Parameters
        ----------
        u_dc : float
        Full dc link voltage

        u_ref : complex
        Reference voltage vector

        """

        u_alpha = u_ref.real
        u_beta = u_ref.imag
        u_ref_ang = (np.arctan2(u_beta,u_alpha) + 2 * np.pi) % (2 * np.pi)
        # Calculating the angels of U_ref (0-2pi)
        # Determine the main sector and apply mapped reference voltage
        sub_hex_deter = int(u_ref_ang // (np.pi / 6))
        if sub_hex_deter == 0 or sub_hex_deter == 11:
            sub_hex = 1
            u_alpha_map = u_alpha - u_dc / 3
            u_beta_map = u_beta
        elif sub_hex_deter == 1 or sub_hex_deter == 2:
            sub_hex = 2
            u_alpha_map = u_alpha - u_dc / 6
            u_beta_map = u_beta - u_dc / 6 * np.sqrt(3)
        elif sub_hex_deter == 3 or sub_hex_deter == 4:
            sub_hex = 3
            u_alpha_map = u_alpha + u_dc / 6
            u_beta_map = u_beta - u_dc / 6 * np.sqrt(3)
        elif sub_hex_deter == 5 or sub_hex_deter == 6:
            sub_hex = 4
            u_alpha_map = u_alpha + u_dc / 3
            u_beta_map = u_beta
        elif sub_hex_deter == 7 or sub_hex_deter == 8:
            sub_hex = 5
            u_alpha_map = u_alpha + u_dc / 6
            u_beta_map = u_beta + u_dc / 6 * np.sqrt(3)
        elif sub_hex_deter == 9 or sub_hex_deter == 10:
            sub_hex = 6
            u_alpha_map = u_alpha - u_dc / 6
            u_beta_map = u_beta + u_dc / 6 * np.sqrt(3)
        else:
            sub_hex = 1
            u_alpha_map = 0
            u_beta_map = 0

        # Calculating the mapped voltage's magnitude and angle(0-2pi)
        u_map_mag = np.sqrt(u_alpha_map ** 2 + u_beta_map ** 2)
        u_map_ang = (np.arctan2(u_beta_map,u_alpha_map) + 2 * np.pi) % (2 * np.pi)

        # Sector Selection in two-level
        sector = int(u_map_ang / (np.pi / 3) % 6)  # 0-5
        # Sector Angle: theta
        theta = u_map_ang - sector * np.pi / 3

        #Overmodulation
        M = u_map_mag / (u_dc / (2 * np.sqrt(3)))
        if M > 1:
            if M > 2 / np.sqrt(3):
                if theta <= np.pi / 6:
                    T_a = 1
                    T_b = 0
                    T_0 = 0
                else:
                    T_a = 0
                    T_b = 1
                    T_0 = 0
            else:
                alpha_g = np.pi / 6 - np.mod(np.arccos(u_dc / (u_map_mag * 2 * np.sqrt(3))), 2 * np.pi)
                if 0 <= theta and theta <= alpha_g:
                    theta = theta
                elif alpha_g <= theta and theta <= np.pi / 6:
                    theta = alpha_g
                elif np.pi / 6 <= theta and theta <= np.pi / 3 - alpha_g:
                    theta = np.pi / 3 - alpha_g
                elif np.pi / 3 - alpha_g <= theta and theta <= np.pi / 3:
                    theta = theta
                T_a = np.sqrt(3) * u_map_mag / u_dc * 2 * (np.sqrt(3.0) / 2 * np.cos(theta) - 1.0 / 2.0 * np.sin(theta))
                T_b = np.sqrt(3) * u_map_mag / u_dc * 2 * np.sin(theta)
                T_0 = 1 - T_a - T_b
        else:
            T_a = np.sqrt(3) * u_map_mag / u_dc * 2 * (np.sqrt(3.0) / 2 * np.cos(theta) - 1.0 / 2.0 * np.sin(theta))
            T_b = np.sqrt(3) * u_map_mag / u_dc * 2 * np.sin(theta)
            T_0 = 1 - T_a - T_b

        sw_sequ = np.zeros(7)
        sw_time = np.zeros(7)
        # k=0 means no solution for unbalanced neutral voltage problem 
        k=0
        if sector % 2 == 0:  # Sector 0 2 4
            sw_sequ[0] = 0
            sw_time[0] = 0 + 0.25 * T_0
            sw_sequ[1] = 1
            sw_time[1] = 0 + 0.25 * T_0 + 0.5 * T_a
            sw_sequ[2] = 2
            sw_time[2] = 0 + 0.25 * T_0 + 0.5 * T_a + 0.5 * T_b
            sw_sequ[3] = 3
            sw_time[3] = 0 + 1 - (0.25 * T_0 + 0.5 * T_a + 0.5 * T_b)
            sw_sequ[4] = 2
            sw_time[4] = 0 + 1 - (0.25 * T_0 + 0.5 * T_a)
            sw_sequ[5] = 1
            sw_time[5] = 0 + 1 - 0.25 * T_0
            sw_sequ[6] = 0
            sw_time[6] = 0 + 1
        else:  # Sector 1 3 5
            sw_sequ[0] = 3
            sw_time[0] = 0 + 0.25 * T_0
            sw_sequ[1] = 2
            sw_time[1] = 0 + 0.25 * T_0 + 0.5 * T_b
            sw_sequ[2] = 1
            sw_time[2] = 0 + 0.25 * T_0 + 0.5 * T_b + 0.5 * T_a
            sw_sequ[3] = 0
            sw_time[3] = 0 + 1 - (0.25 * T_0 + 0.5 * T_b + 0.5 * T_a)
            sw_sequ[4] = 1
            sw_time[4] = 0 + 1 - (0.25 * T_0 + 0.5 * T_b)
            sw_sequ[5] = 2
            sw_time[5] = 0 + 1 - 0.25 * T_0
            sw_sequ[6] = 3
            sw_time[6] = 0 + 1

        # Replacing states and calculating switching states q
        if sub_hex == 1:
            a_state = 'PO'
            b_state = 'ON'
            c_state = 'ON'
        elif sub_hex == 2:
            a_state = 'PO'
            b_state = 'PO'
            c_state = 'ON'
        elif sub_hex == 3:
            a_state = 'ON'
            b_state = 'PO'
            c_state = 'ON'
        elif sub_hex == 4:
            a_state = 'ON'
            b_state = 'PO'
            c_state = 'PO'
        elif sub_hex == 5:
            a_state = 'ON'
            b_state = 'ON'
            c_state = 'PO'
        elif sub_hex == 6:
            a_state = 'PO'
            b_state = 'ON'
            c_state = 'PO'

        tn_sw = np.zeros((7, 2))
        tn_sw[1:7, 0] = sw_time[0:6]
        tn_sw[:, 1] = sw_time[:]
        qs = np.empty((7,3))
        q_abc = np.empty(3, dtype=int)
        for i in range(7):
            j = int(sw_sequ[i])
            q_abc[0] = self.sw_states[sector][j][0] if a_state == 'PO' else self.sw_states[sector][j][0] - 1
            q_abc[1] = self.sw_states[sector][j][1] if b_state == 'PO' else self.sw_states[sector][j][1] - 1
            q_abc[2] = self.sw_states[sector][j][2] if c_state == 'PO' else self.sw_states[sector][j][2] - 1
            qs[i] = q_abc/2
        return tn_sw, qs
    
    
    
    
    

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
    ctrl : Ctrl
        Discrete-time controller.
    delay : int, optional
        Amount of computational delays. The default is 1.
    pwm : bool, optional
        Enable carrier comparison. The default is False.

    """

    def __init__(self, mdl=None, ctrl=None, delay=1, pwm=False, 2lv_svpwm=False, 3lv_svpwm=False):
        self.mdl = mdl
        self.ctrl = ctrl
        self.delay = Delay(delay)
        if pwm:
            self.pwm = CarrierCmp()
        elif 2lv_svpwm:
            self.pwm = 2lv_svpwm()
        elif 3lv_svpwm:
            self.pwm = 3lv_svpwm()
        else:
            self.pwm = zoh

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
        Other options of solve_ivp could be easily changed if needed, but, for
        simplicity, only max_step is included as an option of this method.

        """
        try:
            self.simulation_loop(t_stop, max_step)
        except FloatingPointError:
            print('Invalid value encountered at %.2f seconds.' % self.mdl.t0)
        # Call the post-processing functions
        self.mdl.post_process()
        self.ctrl.post_process()

    @np.errstate(invalid='raise')
    def simulation_loop(self, t_stop, max_step):
        """Run the main simulation loop."""
        while self.mdl.t0 <= t_stop:

            # Run the digital controller
            T_s, d_abc_ref, u_ref, u_dc = self.ctrl(self.mdl)

            # Computational delay model
            d_abc = self.delay(d_abc_ref)

            # Carrier comparison
            t_steps, q = self.pwm(T_s, d_abc)
            
            # Segments and switching states from 2-lv SVPWM 
            t_steps, q = self.pwm(u_ref, u_dc)
            t_steps = T_s*t_steps
            
            # Segments and switching states from 3-lv SVPWM 
            t_steps, q = self.pwm(u_ref, u_dc)
            t_steps = T_s*t_steps

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
