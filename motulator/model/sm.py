# pylint: disable=invalid-name
"""
Continuous-time models for synchronous motors.

The motor models can be parametrized to represent permanent-magnet synchronous
motors and synchronous reluctance motors. Peak-valued complex space vectors are
used.

"""
import numpy as np
from scipy import optimize
from scipy.interpolate import LinearNDInterpolator
from motulator.helpers import complex2abc


# %%
class SynchronousMotor:
    """
    Synchronous motor model.

    This models a synchronous motor in rotor coordinates. The stator flux
    linkage is the state variable. The default values correspond to a 2.2-kW
    permanent-magnet synchronous motor.

    Parameters
    ----------
    p : int
        Number of pole pairs.
    R_s : float
        Stator resistance.
    L_d : float
        d-axis inductance.
    L_q : float
        q-axis inductance.
    psi_f : float
        PM-flux linkage.
    mech : Mechanics
        Model of the mechanical subsystem, needed only for the coordinate
        transformation in the measure_currents method.

    """

    def __init__(
            self, p=3, R_s=3.6, L_d=.036, L_q=.051, psi_f=.545, mech=None):
        # pylint: disable=too-many-arguments
        self.p, self.R_s = p, R_s
        self.L_d, self.L_q, self.psi_f = L_d, L_q, psi_f
        # Initial value
        self.psi_s0 = self.psi_f + 0j
        # For the coordinate transformation
        self._mech = mech

    def current(self, psi_s):
        """
        Compute the stator current.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage.

        Returns
        -------
        i_s : complex
            Stator current.

        """
        i_s = (psi_s.real - self.psi_f)/self.L_d + 1j*psi_s.imag/self.L_q
        return i_s

    def magnetic(self, psi_s):
        """
        Magnetic model.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage.

        Returns
        -------
        i_s : complex
            Stator current.
        tau_M : float
            Electromagnetic torque.

        """
        i_s = self.current(psi_s)
        tau_M = 1.5*self.p*np.imag(i_s*np.conj(psi_s))
        return i_s, tau_M

    def f(self, psi_s, u_s, w_M):
        """
        Compute the state derivative.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage.
        u_s : complex
            Stator voltage.
        w_M : float
            Rotor angular speed (in mechanical rad/s).

        Returns
        -------
        dpsi_s : complex list
            Time derivative of the stator flux linkage.
        i_s : complex
            Stator current.
        tau_M : float
            Electromagnetic torque.

        Notes
        -----
        In addition to the state derivative, this method also returns the
        output signals (stator current `i_ss` and torque `tau_M`) needed for
        interconnection with other subsystems. This avoids overlapping
        computation in simulation.

        """
        i_s, tau_M = self.magnetic(psi_s)
        dpsi_s = u_s - self.R_s*i_s - 1j*self.p*w_M*psi_s
        return [dpsi_s], i_s, tau_M

    def meas_currents(self):
        """
        Measure the phase currents at the end of the sampling period.

        Returns
        -------
        i_s_abc : 3-tuple of floats
            Phase currents.

        """
        i_s0 = self.current(self.psi_s0)
        theta_m0 = self.p*self._mech.theta_M0
        i_s_abc = complex2abc(np.exp(1j*theta_m0)*i_s0)
        return i_s_abc


# %%
class SynchronousMotorSaturated(SynchronousMotor):
    """
    Model of a saturated synchronous motor.

    This extends the SynchronousMotor class with an analytical saturation
    model [1]_, [2]_. The permanent magnets (PMs) are assumed to be along the
    d-axis. The default values correspond to a 6.7-kW synchronous reluctance
    motor.

    Parameters
    ----------
    p : int
        Number of pole pairs.
    R_s : float
        Stator resistance.
    i_f : float
        Constant current corresponding to the magnetomotive force (MMF) of PMs.
        In the magnetically linear case, `i_f = psi_f/L_d`.
    a_d0 : float
        Nonnegative parameter of the saturation model. In the magnetically
        linear case, `a_d0 = 1/L_d`.
    a_q0 : float
        Nonnegative parameter of the saturation model. In the magnetically
        linear case, `a_q0 = 1/L_q`.
    a_dd : float
        Nonnegative constant defining the d-axis self-saturation together with
        `S`. In the magnetically linear case, `a_dd = 0`.
    a_qq : float
        Nonnegative constant defining the q-axis self-saturation together with
        `T`. In the magnetically linear case, `a_qq = 0`.
    a_dq : float
        Nonnegative constant defining the cross-saturation together with `U`
        and `V`. In the magnetically linear case, `a_dq = 0`.
    S : float
        Nonnegative constant defining the d-axis self-saturation.
    T : float
        Nonnegative constant defining the q-axis self-saturation.
    U : float
        Nonnegative constant defining the cross-saturation.
    V : float
        Nonnegative constant defining the cross-saturation.
    mech : Mechanics
        Model of the mechanical subsystem, needed only for the coordinate
        transformation in the measure_currents method.

    Notes
    -----
    The magnetomotive force (MMF) of the PMs is modeled using constant current
    source `i_f` on the d-axis [3]_. Correspondingly, this approach assumes
    that the MMFs of the d-axis current and of the PMs are in series. This
    model cannot capture the desaturation phenomenon of thin iron ribs [4]_.
    For such motors, look-up tables can be used.

    References
    ----------
    .. [1] Hinkkanen, Pescetto, Mölsä, Saarakkala, Pellegrino, Bojoi,
       “Sensorless self-commissioning of synchronous reluctance motors at
       standstill without rotor locking, ”IEEE Trans. Ind. Appl., 2017,
       https://doi.org/10.1109/TIA.2016.2644624

    .. [2] Awan, Song, Saarakkala, Hinkkanen, "Optimal torque control of
       saturated synchronous motors: plug-and-play method," IEEE Trans. Ind.
       Appl., 2018, https://doi.org/10.1109/TIA.2018.2862410

    .. [3] Jahns, Kliman, Neumann, “Interior permanent-magnet synchronous
       motors for adjustable-speed drives,” IEEE Trans. Ind. Appl., 1986,
       https://doi.org/10.1109/TIA.1986.4504786

    .. [4] Armando, Guglielmi, Pellegrino, Pastorelli, Vagati, "Accurate
       modeling and performance analysis of IPM-PMASR motors," IEEE Trans. Ind.
       Appl., 2009, https://doi.org/10.1109/TIA.2008.2009493

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(
        self,
        p=2,
        R_s=.54,
        i_f=0,
        a_d0=17.4,
        a_q0=52.1,
        a_dd=373.,
        a_qq=658.,
        a_dq=1120.,
        S=5,
        T=1,
        U=1,
        V=0,
        mech=None,
    ):
        # pylint: disable=too-many-arguments, disable=super-init-not-called
        self.p, self.R_s = p, R_s
        self.i_f, self.a_d0, self.a_q0 = i_f, a_d0, a_q0
        self.a_dd, self.a_qq, self.a_dq = a_dd, a_qq, a_dq
        self.S, self.T, self.U, self.V = S, T, U, V

        # Initial value of the stator flux
        if i_f == 0:
            # No magnets
            self.psi_s0 = 0j
        else:
            # Solve the stator flux caused by the magnets @ i_s = 0
            psi_d0 = optimize.fmin(
                lambda psi_d: np.abs(
                    (a_d0 + a_dd*np.abs(psi_d)**S)*psi_d - i_f),
                0,
                disp=False)
            self.psi_s0 = complex(psi_d0)

        # For the coordinate transformation
        self._mech = mech

    def current(self, psi_s):
        """Override the base class method."""
        abs_psi_d = np.abs(psi_s.real)
        abs_psi_q = np.abs(psi_s.imag)

        G_dd = self.a_d0 + self.a_dd*abs_psi_d**self.S
        G_dq = self.a_dq/(self.V + 2)*abs_psi_d**self.U*abs_psi_q**(self.V + 2)
        G_qq = self.a_q0 + self.a_qq*abs_psi_q**self.T
        G_qd = self.a_dq/(self.U + 2)*abs_psi_d**(self.U + 2)*abs_psi_q**self.V

        i_d = (G_dd + G_dq)*psi_s.real - self.i_f
        i_q = (G_qq + G_qd)*psi_s.imag
        i_s = i_d + 1j*i_q

        return i_s


# %%
class SynchronousMotorSaturatedLUT(SynchronousMotor):
    """
    Look-up-table-based model of a saturated synchronous motor.

    This extends the SynchronousMotor class with a saturation model, where the
    stator current depends on the stator flux linkage. The coordinates assume
    the PMSM convention, i.e., that the PM flux is along the d-axis.
    Unstructured flux map data can be used.

    Parameters
    ----------
    p : int
        Number of pole pairs.
    R_s : float
        Stator resistance.
    psi_s_data : ndarray of complex
        Stator flux data points for creating the interpolant.
    i_s_data : ndarray of complex
        Stator current data values for creating the interpolant.
    mech : Mechanics
        Model of the mechanical subsystem, needed only for the coordinate
        transformation in the measure_currents method.

    """

    # pylint: disable=too-many-arguments, disable=super-init-not-called
    def __init__(
            self, p=2, R_s=.23, psi_s_data=None, i_s_data=None, mech=None):

        self.p, self.R_s = p, R_s

        # Create the interpolant
        self.interp_i_s = LinearNDInterpolator(
            list(zip(psi_s_data.real, psi_s_data.imag)), i_s_data)

        # Solve the PM flux for the initial value of the stator flux
        psi_d0 = optimize.fmin(
            lambda psi_d: np.abs(self.interp_i_s(psi_d, 0)), 0, disp=False)
        self.psi_s0 = complex(psi_d0)

        # For the coordinate transformation
        self._mech = mech

    def current(self, psi_s):
        """Override the base class method."""
        # Read the current as function of the flux linkage
        i_s = self.interp_i_s(psi_s.real, np.abs(psi_s.imag))
        # Take the sign of the q-axis flux into account
        i_s = i_s.real + 1j*np.sign(psi_s.imag)*i_s.imag
        return i_s
