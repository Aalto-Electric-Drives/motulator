"""Continuous-time models for mechanical subsystems."""

from types import SimpleNamespace

import numpy as np

from motulator.common.model import Subsystem


# %%
class StiffMechanicalSystem(Subsystem):
    """
    Stiff mechanical system.

    Parameters
    ----------
    J : float
        Total moment of inertia (kgm²).
    B_L : float | callable
        Friction coefficient (Nm/(rad/s)) that can be constant, corresponding
        to viscous friction, or an arbitrary function of the rotor speed. For
        example, choosing ``B_L = lambda w_M: k*abs(w_M)`` gives the quadratic
        load torque ``k*w_M**2``. The default is ``B_L = 0``.
    tau_L : callable
        External load torque (Nm) as a function of time, `tau_L_t(t)`. The
        default is zero, ``lambda t: 0*t``.

    """

    def __init__(self, J, B_L=0, tau_L=lambda t: 0*t):
        super().__init__()
        self.par = SimpleNamespace(J=J, B_L=B_L)
        self.tau_L = tau_L
        # Complex exponent is used to represent the rotor angle to provide
        # continuity in the angle representation and to avoid wrapping issues.
        self.state = SimpleNamespace(w_M=0, exp_j_theta_M=complex(1))
        self.sol_states = SimpleNamespace(w_M=[], exp_j_theta_M=[])

    @property
    def B_L(self):
        """Friction coefficient (Nm/(rad/s))."""
        if callable(self.par.B_L):
            return self.par.B_L(np.abs(self.state.w_M))
        return self.par.B_L

    def set_outputs(self, t):
        """Set output variables."""
        state, out = self.state, self.out
        out.w_M = state.w_M
        out.tau_L_tot = self.B_L*out.w_M + self.tau_L(t)

    def rhs(self):
        """Compute state derivatives."""
        state, inp, out = self.state, self.inp, self.out
        d_w_M = (inp.tau_M - out.tau_L_tot)/self.par.J
        d_exp_j_theta_M = 1j*state.w_M*state.exp_j_theta_M

        return [d_w_M, d_exp_j_theta_M]

    def meas_speed(self):
        """
        Measure the rotor speed.

        Returns
        -------
        w_M : float
            Rotor angular speed (mechanical rad/s).

        """
        return self.state.w_M.real

    def meas_position(self):
        """
        Measure the rotor angle.

        Returns
        -------
        theta_M : float
            Rotor angle (mechanical rad).

        """
        return np.angle(self.state.exp_j_theta_M)

    def post_process_states(self):
        """Post-process data."""
        self.data.w_M = self.data.w_M.real
        self.data.theta_M = np.angle(self.data.exp_j_theta_M)

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        B_L = self.par.B_L(np.abs(data.w_M)) if callable(
            self.par.B_L) else self.par.B_L
        data.tau_L_tot = self.tau_L(data.t) + B_L*data.w_M


# %%
class TwoMassMechanicalSystem(StiffMechanicalSystem):
    """
    Two-mass mechanical subsystem.

    Parameters
    ----------
    par : TwoMassMechanicalSystemPars
        Two-mass mechanical system parameters.
    tau_L : callable
        Load torque (Nm) as a function of time, `tau_L(t)`. The default is
        zero, ``lambda t: 0*t``.

    """

    def __init__(self, par, tau_L=lambda t: 0*t):
        super().__init__(J=None, B_L=None, tau_L=tau_L)
        self.par = par
        self.state = SimpleNamespace(w_M=0, exp_j_theta_M=0, w_L=0, theta_ML=0)
        self.sol_states = SimpleNamespace(
            w_M=[], exp_j_theta_M=[], w_L=[], theta_ML=[])

    @property
    def B_L(self):
        """Friction coefficient (Nm/(rad/s))."""
        # Overwrite the base class property
        if callable(self.par.B_L):
            return self.par.B_L(np.abs(self.state.w_L.real))
        return self.par.B_L

    def set_outputs(self, t):
        """Set output variables."""
        super().set_outputs(t)
        state, out = self.state, self.out
        out.w_L, out.theta_ML = state.w_L, state.theta_ML
        out.tau_L_tot = self.B_L*out.w_L + self.tau_L(t)
        out.tau_S = self.par.K_S*out.theta_ML + self.par.C_S*(
            out.w_M - out.w_L)

    def rhs(self):
        """Compute state derivatives."""
        state, inp, out = self.state, self.inp, self.out
        d_w_M = (inp.tau_M - out.tau_S)/self.par.J_M
        d_exp_j_theta_M = 1j*state.w_M*state.exp_j_theta_M
        d_w_L = (out.tau_S - out.tau_L_tot)/self.par.J_L
        d_theta_ML = state.w_M - state.w_L

        return [d_w_M, d_exp_j_theta_M, d_w_L, d_theta_ML]

    def meas_load_speed(self):
        """Measure the load speed."""
        return self.state.w_L.real

    def meas_load_position(self):
        """Measure the load angle."""
        theta_L = self.state.theta_M - self.state.theta_ML
        return theta_L.real

    def post_process_states(self):
        """Post-process data."""
        super().post_process_states()
        self.data.w_L = self.data.w_L.real
        self.data.theta_ML = self.data.theta_ML.real

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        B_L = self.par.B_L(np.abs(data.w_L)) if callable(
            self.par.B_L) else self.par.B_L
        data.tau_L_tot = self.tau_L(data.t) + B_L*data.w_L


# %%
class ExternalRotorSpeed(Subsystem):
    """
    Integrate the rotor angle from the externally given rotor speed.

    Parameters
    ----------
    w_M : callable
        Rotor speed (rad/s) as a function of time, `w_M(t)`. The default is
        zero, ``lambda t: 0*t``.

    """

    def __init__(self, w_M=lambda t: 0*t):
        super().__init__()
        self.w_M = w_M
        self.state = SimpleNamespace(exp_j_theta_M=complex(1))
        self.out = SimpleNamespace(w_M=w_M(0))  # Needed for direct feedthrough
        self.sol_states = SimpleNamespace(exp_j_theta_M=[])

    def set_outputs(self, t):
        """Set output variables."""
        self.out.w_M = self.w_M(t)

    def rhs(self):
        """Compute state derivatives."""
        d_exp_j_theta_M = 1j*self.out.w_M*self.state.exp_j_theta_M
        return [d_exp_j_theta_M]

    def meas_speed(self):
        """
        Measure the rotor speed.

        Returns
        -------
        w_M : float
            Rotor angular speed (mechanical rad/s).

        """
        return self.out.w_M

    def meas_position(self):
        """
        Measure the rotor angle.

        Returns
        -------
        theta_M : float
            Rotor angle (mechanical rad).

        """
        return np.angle(self.state.exp_j_theta_M)

    def post_process_states(self):
        """Post-process data."""
        self.data.theta_M = np.angle(self.data.exp_j_theta_M)
        self.data.w_M = self.w_M(self.data.t)
