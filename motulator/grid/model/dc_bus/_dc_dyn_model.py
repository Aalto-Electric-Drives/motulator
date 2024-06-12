"""
DC-bus and filter model interconnectors.

    This module interconnects the subsystems of a converter with a grid and
    provides an interface to the solver. More complicated systems could be
    modeled using a similar template. Peak-valued complex space vectors are
    used. The space vector models are implemented in stationary coordinates.

"""

import numpy as np

from gritulator._helpers import complex2abc
from gritulator._utils import Bunch

# %%
class DCBusAndLFilterModel:
    """
    Continuous-time model for a stiff grid model with an RL impedance model.

    Parameters
    ----------
    grid_filter : LFilter
        RL line dynamic model.
    grid_model : StiffSource
        Constant voltage source model.
    dc_model : DCBus
        DC-bus voltage dynamics.
    converter : Inverter | PWMInverter
        Inverter model.

    """
    
    def __init__(
            self, grid_filter=None, grid_model=None,
            dc_model=None, converter=None):
        self.grid_filter = grid_filter
        self.grid_model = grid_model
        self.dc_model = dc_model
        self.converter = converter

        # Initial time
        self.t0 = 0

        # Store the solution in these lists
        self.data = Bunch()
        self.data.t, self.data.q = [], []
        self.data.i_gs = []
        self.data.u_dc = [] 
        self.data.i_dc = []
        
    def get_initial_values(self):
        """
        Get the initial values.

        Returns
        -------
        x0 : complex list, length 2
            Initial values of the state variables.

        """
        x0 = [self.grid_filter.i_gs0, self.dc_model.u_dc0]
        return x0

    def set_initial_values(self, t0, x0):
        """
        Set the initial values.

        Parameters
        ----------
        x0 : complex ndarray
            Initial values of the state variables.

        """
        self.t0 = t0
        self.grid_filter.i_gs0 = x0[0]
        self.dc_model.u_dc0 = x0[1].real
        self.converter.u_dc0 = x0[1].real
        # calculation of converter-side voltage
        u_cs0 = self.converter.ac_voltage(self.converter.q, x0[1].real)
        # calculation of grid-side voltage
        e_gs0 = self.grid_model.voltages(t0)
        # update pcc voltage
        self.grid_filter.u_gs0 = self.grid_filter.pcc_voltages(
                                                x0[0], u_cs0, e_gs0)

    def f(self, t, x):
        """
        Compute the complete state derivative list for the solver.

        Parameters
        ----------
        t : float
            Time.
        x : complex ndarray
            State vector.

        Returns
        -------
        complex list
            State derivatives.

        """
        # Unpack the states
        i_gs, u_dc = x
        # Interconnections: outputs for computing the state derivatives
        u_cs = self.converter.ac_voltage(self.converter.q, u_dc)
        e_gs = self.grid_model.voltages(t)
        q = self.converter.q
        i_g_abc = complex2abc(i_gs)
        i_dc = self.dc_model.dc_current(i_g_abc, q) # DC-current
        # State derivatives
        rl_f = self.grid_filter.f(i_gs, u_cs, e_gs)
        dc_f = self.dc_model.f(t, u_dc, i_dc)
        # List of state derivatives
        return [rl_f, dc_f]

    def save(self, sol):
        """
        Save the solution.

        Parameters
        ----------
        sol : Bunch object
            Solution from the solver.

        """
        self.data.t.extend(sol.t)
        self.data.i_gs.extend(sol.y[0])
        self.data.u_dc.extend(sol.y[1].real)
        self.data.q.extend(sol.q)
        q_abc=complex2abc(np.asarray(sol.q))
        i_c_abc=complex2abc(sol.y[0])
        self.data.i_dc.extend(
            q_abc[0]*i_c_abc[0] + q_abc[1]*i_c_abc[1] + q_abc[2]*i_c_abc[2])
                                    
    def post_process(self):
        """
        Transform the lists to the ndarray format and post-process them.

        """
        # From lists to the ndarray
        self.data.t = np.asarray(self.data.t)
        self.data.i_gs = np.asarray(self.data.i_gs)
        self.data.u_dc = np.asarray(self.data.u_dc)
        self.data.q = np.asarray(self.data.q)
        #self.data.theta = np.asarray(self.data.theta)
        # Some useful variables
        self.data.i_dc = np.asarray(self.data.i_dc)
        self.data.e_gs = self.grid_model.voltages(self.data.t)
        self.data.theta = np.mod(self.data.t*self.grid_model.w_N, 2*np.pi)
        self.data.u_cs = self.converter.ac_voltage(
            self.data.q, self.data.u_dc)
        self.data.u_gs = self.grid_filter.pcc_voltages(
            self.data.i_gs,
            self.data.u_cs,
            self.data.e_gs)


# %%
class DCBusAndLCLFilterModel:
    """
    Continuous-time model for a stiff grid model with an LCL impedance model.

    Parameters
    ----------
    grid_filter : LCLFilter
        LCL filter dynamic model.
    grid_model : StiffSource
        Constant voltage source model.
    dc_model : DCBus
        DC-bus voltage dynamics.
    converter : Inverter | PWMInverter
        Inverter model.

    """
    
    def __init__(
            self, grid_filter=None, grid_model=None, 
            dc_model=None, converter=None):
        self.grid_filter = grid_filter
        self.grid_model = grid_model
        self.dc_model = dc_model
        self.converter = converter

        # Initial time
        self.t0 = 0

        # Store the solution in these lists
        self.data = Bunch()
        self.data.t, self.data.q = [], []
        self.data.i_gs = []
        self.data.i_cs = []
        self.data.u_fs = []
        self.data.u_dc = [] 
        self.data.i_dc = []
        
    def get_initial_values(self):
        """
        Get the initial values.

        Returns
        -------
        x0 : complex list, length 4
            Initial values of the state variables.

        """
        x0 = [
            self.grid_filter.i_cs0,
            self.grid_filter.u_fs0,
            self.grid_filter.i_gs0,
            self.dc_model.u_dc0]

        return x0

    def set_initial_values(self, t0, x0):
        """
        Set the initial values.

        Parameters
        ----------
        x0 : complex ndarray
            Initial values of the state variables.

        """
        self.t0 = t0
        self.grid_filter.i_cs0 = x0[0]
        self.grid_filter.u_fs0 = x0[1]
        self.grid_filter.i_gs0 = x0[2]
        self.dc_model.u_dc0 = x0[3].real
        self.converter.u_dc0 = x0[3].real
        # calculation of grid-side voltage
        e_gs0 = self.grid_model.voltages(t0)
        # update pcc voltage
        self.grid_filter.u_gs0 = self.grid_filter.pcc_voltages(
                                                    x0[2], x0[1], e_gs0)

    def f(self, t, x):
        """
        Compute the complete state derivative list for the solver.

        Parameters
        ----------
        t : float
            Time.
        x : complex ndarray
            State vector.

        Returns
        -------
        complex list
            State derivatives.

        """
        # Unpack the states
        i_cs, u_fs, i_gs, u_dc = x
        # Interconnections: outputs for computing the state derivatives
        u_cs = self.converter.ac_voltage(self.converter.q, self.converter.u_dc0)
        e_gs = self.grid_model.voltages(t)
        q = self.converter.q
        i_c_abc = complex2abc(i_cs)
        i_dc = self.dc_model.dc_current(i_c_abc, q) # DC-current
        # State derivatives
        lcl_f = self.grid_filter.f(i_cs, u_fs, i_gs, u_cs, e_gs)
        dc_f = self.dc_model.f(t, u_dc, i_dc)
        # List of state derivatives
        all_f = [lcl_f[0], lcl_f[1], lcl_f[2], dc_f]
        return all_f

    def save(self, sol):
        """
        Save the solution.

        Parameters
        ----------
        sol : Bunch object
            Solution from the solver.

        """
        self.data.t.extend(sol.t)
        self.data.i_cs.extend(sol.y[0])
        self.data.u_fs.extend(sol.y[1])
        self.data.i_gs.extend(sol.y[2])
        self.data.u_dc.extend(sol.y[3].real)
        self.data.q.extend(sol.q)
        q_abc=complex2abc(np.asarray(sol.q))
        i_c_abc=complex2abc(sol.y[0])
        self.data.i_dc.extend(
            q_abc[0]*i_c_abc[0] + q_abc[1]*i_c_abc[1] + q_abc[2]*i_c_abc[2])
                                
    def post_process(self):
        """
        Transform the lists to the ndarray format and post-process them.

        """
        # From lists to the ndarray
        self.data.t = np.asarray(self.data.t)
        self.data.i_cs = np.asarray(self.data.i_cs)
        self.data.u_fs = np.asarray(self.data.u_fs)
        self.data.i_gs = np.asarray(self.data.i_gs)
        self.data.u_dc = np.asarray(self.data.u_dc)
        self.data.q = np.asarray(self.data.q)
        # Some useful variables
        self.data.i_dc = np.asarray(self.data.i_dc)
        self.data.e_gs = self.grid_model.voltages(self.data.t)
        self.data.theta = np.mod(self.data.t*self.grid_model.w_N, 2*np.pi)
        self.data.u_cs = self.converter.ac_voltage(self.data.q, self.data.u_dc)
        self.data.u_gs = self.grid_filter.pcc_voltages(
            self.data.i_gs,
            self.data.u_fs,
            self.data.e_gs)
