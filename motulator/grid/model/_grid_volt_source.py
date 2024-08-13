"""
3-phase AC voltage source models.

Peak-valued complex space vectors are used.

"""
from types import SimpleNamespace

import numpy as np

from motulator.common.model import Subsystem
from motulator.common.utils import complex2abc, abc2complex, wrap


# %%
# TODO: implement modeling of harmonics
class StiffSource(Subsystem):
    """
    3-phase voltage source model.

    This model is a 3-phase voltage source for the AC grid. A stiff grid is
    modeled, where the frequency is given by the user either as a constant
    or a time-dependent function. The grid voltage magnitude e_g_abs can also
    be a function, to simulate voltage dips and symmetrical short circuits. 
    
    In addition, nonsymmetric faults can be modeled either by giving the
    negative sequence voltage magnitude e_g_neg_abs, or by specifying any of
    the phase voltage magnitudes separately (e.g. e_ga_abs).
    
    A phase shift angle for the grid voltages can also be given as a function,
    to simulate phase angle jumps. The complex form of the phase A voltage
    angle (exp_j_theta_p) is used as a state variable.

    Parameters
    ----------
    w_gN : float
        Grid nominal frequency (rad/s).
    e_g_abs : float | callable
        3-phase grid voltage magnitude, phase-to-ground peak value (V).
    phi : callable, optional
        Phase shift of the grid voltage (rad). Default is 0.
    e_g_neg_abs : float | callable, optional
        Negative sequence voltage magnitude, phase-to-ground peak value (V).
        Default is None.
    phi_neg : float, optional
        Phase shift of the negative sequence component (rad). Default is 0.
    w_g : callable, optional
        Grid frequency (rad/s) as a function of time, `w_g(t)`. If given, w_g
        will be used to compute grid voltage angle instead of w_gN. Default
        is None.
    e_ga_abs : callable, optional
        Phase A voltage magnitude, phase-to-ground peak value (V). If given,
        this will be used instead of e_g_abs. Default is None.
    e_gb_abs : callable, optional
        Phase B voltage magnitude, phase-to-ground peak value (V). Default is
        None.
    e_gc_abs : callable, optional
        Phase C voltage magnitude, phase-to-ground peak value (V). Default is
        None.

    """

    def __init__(
            self,
            w_gN,
            e_g_abs,
            phi=lambda t: 0,
            e_g_neg_abs=0,
            phi_neg=0,
            w_g=None,
            e_ga_abs=None,
            e_gb_abs=None,
            e_gc_abs=None):
        super().__init__()
        self.par = SimpleNamespace(
            w_gN=w_gN,
            e_g_abs=e_g_abs,
            phi=phi,
            e_g_neg_abs=e_g_neg_abs,
            phi_neg=phi_neg,
            w_g=w_g,
            e_ga_abs=e_ga_abs,
            e_gb_abs=e_gb_abs,
            e_gc_abs=e_gc_abs,
        )
        # states
        self.state = SimpleNamespace(exp_j_theta_p=complex(1))
        # Store the solutions in these lists
        self.sol_states = SimpleNamespace(exp_j_theta_p=[])

    def voltages(self, t, theta_p):
        """
        Compute the grid voltage in stationary frame.
        
        Parameters
        ----------
        t : float
            Time (s).
        theta_p : float
            Phase A voltage angle (rad).

        Returns
        -------
        e_gs : complex
            Grid complex voltage (V).

        """
        # Phase voltage magnitudes
        par = self.par
        e_g_abs = par.e_g_abs(t) if callable(par.e_g_abs) else par.e_g_abs
        e_ga_abs = par.e_ga_abs(t) if par.e_ga_abs is not None else e_g_abs
        e_gb_abs = par.e_gb_abs(t) if par.e_gb_abs is not None else e_g_abs
        e_gc_abs = par.e_gc_abs(t) if par.e_gc_abs is not None else e_g_abs

        # Calculation of the three-phase voltages
        e_ga = e_ga_abs*np.cos(theta_p - par.phi(t))
        e_gb = e_gb_abs*np.cos(theta_p - 2*np.pi/3 - par.phi(t))
        e_gc = e_gc_abs*np.cos(theta_p - 4*np.pi/3 - par.phi(t))

        e_gs = abc2complex([e_ga, e_gb, e_gc])

        # Adding negative sequence component if given
        if par.e_g_neg_abs is not None:
            e_g_neg_abs = par.e_g_neg_abs(t) if callable(
                par.e_g_neg_abs) else par.e_g_neg_abs
            e_gs += e_g_neg_abs*np.exp(
                -1j*(theta_p + par.phi_neg + par.phi(t)))

        return e_gs

    def set_outputs(self, t):
        """Set output variables."""
        self.out.e_gs = self.voltages(t, np.angle(self.state.exp_j_theta_p))

    def set_inputs(self, t):
        """Set input variables."""
        self.inp.w_g = self.par.w_g(t) if callable(
            self.par.w_g) else self.par.w_gN

    def rhs(self):
        """
        Compute the state derivatives.
        
        Returns
        -------
        Complex list, length 1
            Time derivative of the complex state vector, [d_exp_j_theta_p].
            
        """
        d_exp_j_theta_p = 1j*self.inp.w_g*self.state.exp_j_theta_p
        return [d_exp_j_theta_p]

    def meas_voltages(self, t):
        """
        Measure the grid phase voltages.
        
        Parameters
        ----------
        t : float
            Time (s).

        Returns
        -------
        e_g_abc : 3-tuple of floats
            Phase voltages (V).

        """
        e_gs = self.voltages(t, np.angle(self.state.exp_j_theta_p))
        return complex2abc(e_gs)

    def post_process_states(self):
        """Post-process the solution."""
        if callable(self.par.w_g):
            self.data.w_g = self.par.w_g(self.data.t)
        else:
            self.data.w_g = np.full(np.size(self.data.t), self.par.w_gN)
        self.data.theta_p = np.angle(self.data.exp_j_theta_p)
        self.data.e_gs = self.voltages(self.data.t, self.data.theta_p)
        self.data.theta_g = np.angle(self.data.e_gs)


class FlexSource(Subsystem):
    """
    3-phase AC grid including synchronous generator electromechanical dynamics.

    This models the 3-phase voltage source of the AC grid while taking into
    account the electromechanical dynamics of a typical grid generated by the 
    synchronous generators. More information about the model can be found in
    [#ENT2013]_.
    
    Parameters
    ----------
    w_gN : float
        Grid nominal frequency (rad/s).
    e_g_abs : float | callable
        3-phase grid voltage magnitude, phase-to-ground peak value (V).
    S_grid : float
        Grid rated power (VA).
    T_D : float, optional
        Turbine delay time constant (s). Default is 10.
    T_N : float, optional
        Turbine derivative time constant (s). Default is 3.
    H_g : float, optional
        Grid inertia constant (s). Default is 3.
    r_d : float, optional
        Primary frequency droop control gain (p.u.). Default is 0.05.
    T_gov : float, optional
        Governor time constant (s). Default is 0.5.
    p_m_ref : callable, optional
        Mechanical power output reference (W). Default is 0.
    p_e : callable, optional
        Electrical power disturbance (W). Default is 0.

    References
    ----------
    .. [#ENT2013] ENTSO-E, Documentation on Controller Tests in Test Grid
        Configurations, Technical Report, 26.11.2013.
    
    """

    def __init__(
            self,
            w_gN,
            e_g_abs,
            S_grid,
            T_D=10,
            T_N=3,
            H_g=3,
            D_g=0,
            r_d=.05,
            T_gov=0.5,
            p_m_ref=lambda t: 0,
            p_e=lambda t: 0):
        super().__init__()
        self.p_m_ref = p_m_ref
        self.p_e = p_e
        self.par = SimpleNamespace(
            T_D=T_D,
            T_N=T_N,
            H_g=H_g,
            D_g=D_g,
            r_d=r_d*w_gN/S_grid,
            T_gov=T_gov,
            w_gN=w_gN,
            S_grid=S_grid,
            e_g_abs=e_g_abs,
        )
        # States
        self.state = SimpleNamespace(err_w_g=0, p_gov=0, x_turb=0, theta_g=0)
        # Store the solutions in these lists
        self.sol_states = SimpleNamespace(
            err_w_g=[],
            p_gov=[],
            x_turb=[],
            theta_g=[],
        )

    def voltages(self, t, theta_g):
        """
        Compute the grid voltage in stationary frame:
           
        Parameters
        ----------
        t : float
            Time.
        theta_g : float
            Grid electrical angle (rad).

        Returns
        -------
        e_gs: complex
            Grid complex voltage (V).

        """

        # Calculation of the three-phase voltages
        e_g_abs = self.par.e_g_abs(t) if callable(
            self.par.e_g_abs) else self.par.e_g_abs
        e_g_a = e_g_abs*np.cos(theta_g)
        e_g_b = e_g_abs*np.cos(theta_g - 2*np.pi/3)
        e_g_c = e_g_abs*np.cos(theta_g - 4*np.pi/3)

        e_gs = abc2complex([e_g_a, e_g_b, e_g_c])
        return e_gs

    def set_outputs(self, t):
        """Set output variables."""
        self.out.e_gs = self.voltages(t, wrap(self.state.theta_g.real))

    def set_inputs(self, t):
        """Set input variables."""
        self.inp.p_m_ref = self.p_m_ref(t)
        self.inp.p_e = self.p_e(t)

    def rhs(self):
        """
        Compute the state derivatives.

        Returns
        -------
        Complex list, length 4
            Time derivative of the complex state vector,
            [d_err_w_g, d_p_gov, d_x_turb, d_theta_g].

        """
        par, state, inp = self.par, self.state, self.inp
        err_w_g = state.err_w_g
        p_gov = state.p_gov
        x_turb = state.x_turb

        # calculation of mechanical power from the turbine output
        p_m = (par.T_N/par.T_D)*p_gov + (1 - (par.T_N/par.T_D))*x_turb
        # swing equation
        p_diff = (p_m - inp.p_e)/par.S_grid  # in per units
        d_err_w_g = par.w_gN*(p_diff - par.D_g*err_w_g)/(2*par.H_g)
        # governor dynamics
        d_p_gov = (inp.p_m_ref - (1/par.r_d)*err_w_g - p_gov)/par.T_gov
        # turbine dynamics (lead-lag)
        d_x_turb = (p_gov - x_turb)/par.T_D
        # integration of the angle
        d_theta_g = par.w_gN + err_w_g
        return [d_err_w_g, d_p_gov, d_x_turb, d_theta_g]

    def meas_voltages(self, t):
        """
        Measure the grid phase voltages.
        
        Parameters
        ----------
        t : float
            Time (s).

        Returns
        -------
        e_g_abc : 3-tuple of floats
            Phase voltages (V).

        """
        e_g_abc = complex2abc(self.voltages(t, self.state.theta_g.real))
        return e_g_abc

    def meas_freq(self):
        """
        Measure the grid frequency.

        Returns
        -------
        w_g : float
            Grid angular frequency (rad/s).

        """
        w_g = self.par.w_gN + self.state.err_w_g.real
        return w_g

    def meas_angle(self):
        """
        Measure the grid angle.

        Returns
        -------
        theta_g : float
            Grid electrical angle (rad).

        """
        theta_g = wrap(self.state.theta_g.real)
        return theta_g

    def post_process_states(self):
        """Post-process the solution."""
        self.data.w_g = self.par.w_gN + self.data.err_w_g.real
        self.data.theta_g = wrap(self.data.theta_g.real)
        self.data.e_gs = self.voltages(self.data.t, self.data.theta_g)
