"""Continuous-time models for mechanical subsystems."""

from types import SimpleNamespace

from motulator.common.model import Subsystem
from motulator.common.utils import wrap


# %%
class Mechanics(Subsystem):
    """
    Mechanics subsystem.

    This models an equation of motion for stiff mechanics.

    Parameters
    ----------
    J : float
        Total moment of inertia (kgm²).
    tau_L_w : callable
        Load torque (Nm) as a function of speed, `tau_L_w(w_M)`. For example,
        ``tau_L_w = b*w_M``, where `b` is the viscous friction coefficient. The
        default is zero, ``lambda w_M: 0*w_M``.
    tau_L_t : callable
        Load torque (Nm) as a function of time, `tau_L_t(t)`. The default is 
        zero, ``lambda t: 0*t``.

    """

    def __init__(self, J, tau_L_w=lambda w_M: 0*w_M, tau_L_t=lambda t: 0*t):
        super().__init__()
        self.J = J
        self.tau_L_t, self.tau_L_w = tau_L_t, tau_L_w
        self.state = SimpleNamespace(w_M=0, theta_M=0)
        self.sol_states = SimpleNamespace(w_M=[], theta_M=[])

    def set_outputs(self, t):
        """Set output variables."""
        state, out = self.state, self.out
        out.w_M, out.theta_M = state.w_M, state.theta_M
        # Total load torque
        out.tau_L = self.tau_L_w(state.w_M) + self.tau_L_t(t)

    def rhs(self):
        """Compute state derivatives."""
        state, inp, out = self.state, self.inp, self.out
        d_w_M = (inp.tau_M - out.tau_L)/self.J
        d_theta_M = state.w_M
        # Wrap the angle, take the real parts
        state.theta_M = wrap(state.theta_M.real)
        state.w_M = state.w_M.real

        return [d_w_M, d_theta_M]

    def meas_speed(self):
        """
        Measure the rotor speed.

        Returns
        -------
        w_M : float
            Rotor angular speed (mechanical rad/s).

        """
        # The quantization noise of an incremental encoder could be modeled.
        return self.state.w_M.real

    def meas_position(self):
        """
        Measure the rotor angle.

        Returns
        -------
        theta_M : float
            Rotor angle (mechanical rad).

        """
        return self.state.theta_M.real

    def post_process_states(self):
        """Post-process data."""
        data = self.data
        data.w_M = data.w_M.real
        data.theta_M = wrap(data.theta_M.real)

    def post_process_with_inputs(self):
        """Post-process data with inputs."""
        data = self.data
        data.tau_L = (self.tau_L_t(data.t) + self.tau_L_w(data.w_M))


# %%
class TwoMassMechanics(Mechanics):
    """
    Two-mass mechanics subsystem.

    This models an equation of motion for two-mass mechanics.

    Parameters
    ----------
    J_M : float
        Motor moment of inertia (kgm²).
    J_L : float
        Load moment of inertia (kgm²).
    K_S : float
        Shaft torsional stiffness (Nm).
    C_S : float
        Shaft torsional damping (Nms).
    tau_L_w : callable
        Load torque (Nm) as a function of the load speed, `tau_L_w(w_L)`, e.g., 
        ``tau_L_w = B*w_L``, where `B` is the viscous friction coefficient. The
        default is zero, ``lambda w_L: 0*w_L``.
    tau_L_t : callable
        Load torque (Nm) as a function of time, `tau_L_t(t)`. The default is
        zero, ``lambda t: 0*t``.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, J_M, J_L, K_S, C_S, tau_L_w=None, tau_L_t=None):
        # pylint: disable=super-init-not-called
        self.J_M, self.J_L, self.K_S, self.C_S = J_M, J_L, K_S, C_S
        self.tau_L_w = tau_L_w if tau_L_w is not None else lambda w_L: 0*w_L
        self.tau_L_t = tau_L_t if tau_L_t is not None else lambda t: 0*t
        # Initial values
        self.state = SimpleNamespace(w_M=0, theta_M=0, w_L=0, theta_ML=0)
        self.inp = SimpleNamespace(tau_M=None)
        self.out = SimpleNamespace()
        self.sol_states = SimpleNamespace(
            w_M=[], theta_M=[], w_L=[], theta_ML=[])
        self.data = SimpleNamespace()

    def set_outputs(self, t):
        """Set output variables."""
        super().set_outputs(t)
        state, out = self.state, self.out
        out.w_L, out.theta_ML = state.w_L, state.theta_ML
        out.tau_S = self.K_S*out.theta_ML + self.C_S*(out.w_M - out.w_L)

    def rhs(self):
        """Compute state derivatives."""
        state, inp, out = self.state, self.inp, self.out
        d_w_M = (inp.tau_M - out.tau_S)/self.J_M
        d_theta_M = state.w_M
        d_w_L = (out.tau_S - out.tau_L)/self.J_L
        d_theta_ML = state.w_M - state.w_L
        # Wrap the angle, take the real parts
        state.theta_M = wrap(state.theta_M.real)
        state.theta_ML = state.theta_ML.real
        state.w_M, state.w_L = state.w_M.real, state.w_L.real

        return [d_w_M, d_theta_M, d_w_L, d_theta_ML]

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
