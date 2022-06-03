# pylint: disable=C0103
# pylint: disable=R0903
"""
This module contains various helper functions and classes.

"""
# %%
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler

# Plotting parameters
plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
plt.rcParams['lines.linewidth'] = 1.
plt.rcParams['axes.grid'] = True
plt.rcParams.update({"text.usetex": False})


# %%
def abc2complex(u):
    """
    Transform three-phase quantities to a complex space vector.

    Parameters
    ----------
    u : array_like, shape (3,)
        Phase quantities.

    Returns
    -------
    complex
        Complex space vector (peak-value scaling).

    Examples
    --------
    >>> y = abc2complex([1, 2, 3])
    >>> y
    (-1-0.5773502691896258j)

    """
    return (2/3)*u[0] - (u[1] + u[2])/3 + 1j*(u[1] - u[2])/np.sqrt(3)


# %%
def complex2abc(u):
    """
    Transform a complex space vector to three-phase quantities.

    Parameters
    ----------
    u : complex
        Complex space vector (peak-value scaling).

    Returns
    -------
    ndarray, shape (3,)
        Phase quantities.

    Examples
    --------
    >>> y = complex2abc(1-.5j)
    >>> y
    array([ 1.       , -0.9330127, -0.0669873])

    """
    return np.array([u.real,
                     .5*(-u.real + np.sqrt(3)*u.imag),
                     .5*(-u.real - np.sqrt(3)*u.imag)])


# %%
class Sequence:
    """
    This class represents a sequence generator. The time array must be
    increasing. The output values are interpolated between the data points.

    """

    def __init__(self, times, values, periodic=False):
        self.times = times
        self.values = values
        if periodic is True:
            self.__period = times[-1] - times[0]
        else:
            self.__period = None

    def __call__(self, t):
        """
        Interpolates the output.

        Parameters
        ----------
        t : float
            Time.

        Returns
        -------
        float or complex
            Interpolated output.

        """
        return np.interp(t, self.times, self.values, period=self.__period)

    def __str__(self):
        desc = (('Sequence:\n    times={}\n    values={}')
                .format(self.times, self.values))
        return desc


# %%
class Step:
    """
    This data class represents a step function.

    """

    def __init__(self, step_time, step_value, initial_value=0):
        self.step_time = step_time
        self.step_value = step_value
        self.initial_value = initial_value

    def __call__(self, t):
        """
        Step function.

        Parameters
        ----------
        t : float
            Time.

        Returns
        -------
        float
            Step output.

        """
        return self.initial_value + (t >= self.step_time)*self.step_value

    def __str__(self):
        desc = (('Step(step_time={:.1f}, initial_value={:.1f},'
                ' step_value={:.1f})')
                .format(self.step_time, self.initial_value, self.step_value))
        return desc


# %%
def ref_ramp(mdl, w_max=2*np.pi*50, tau_max=14.6, t_max=4):
    """
    Generate an example ramp profile for the speed reference. The load torque
    changes stepwise.

    Parameters
    ----------
    mdl : object
        Model.
    w_max : float, optional
        Maximum speed in the profile. The default is 2*pi*50.
    tau_max : float, optional
        Maximum load torque in the profile. The default is 14.6.
    t_max : float, optional
        Length of the profile. The default is 4.

    """
    # Speed reference
    times = np.array([0, .125, .25, .375, .5, .625,  .75, .875, 1])*t_max
    values = np.array([0,  0, 1,   1, 0,  -1, -1,   0, 0])*w_max
    mdl.speed_ref = Sequence(times, values)

    # External load torque
    times = np.array([0, .125, .125, .875, .875, 1])*t_max
    values = np.array([0, 0, 1, 1, 0, 0])*tau_max
    mdl.mech.tau_L_ext = Sequence(times, values)

    # Stop time of the simulation
    mdl.t_stop = mdl.speed_ref.times[-1]

    # Print the profiles
    # np.set_printoptions(precision=1, suppress=True)
    # print('Profiles')
    # print('--------')
    # print('Speed reference:')
    # print('    {}'.format(mdl.speed_ref))
    # print('External load torque:')
    # print('    {}\n'.format(mdl.mech.tau_L_ext))


# %%
def ref_step(mdl, w_max=.8*2*np.pi*50, tau_max=14.6, t_max=1.5):
    """
    Generate an example stepwise profile for the speed reference and load
    torque.

    Parameters
    ----------
    mdl : object
        Model.
    w_max : float, optional
        Maximum speed in the profile. The default is .8*2*pi*50.
    tau_max : float, optional
        Maximum load torque in the profile. The default is 14.6.
    t_max : float, optional
        Length of the profile. The default is 1.5.

    """
    # Speed reference
    mdl.speed_ref = Step(.2*t_max, w_max)
    # External load torque
    mdl.mech.tau_L_ext = Step(2./3.*t_max, tau_max)
    # Stop time of the simulation
    mdl.t_stop = t_max

    # Print the profiles
    print('Profiles')
    print('--------')
    print('Speed reference:')
    print('    {}'.format(mdl.speed_ref))
    print('External load torque:')
    print('    {}'.format(mdl.mech.tau_L_ext))


# %%
def plot(mdl, ctrl, base):
    """
    Plot example figures.

    Parameters
    ----------
    mdl : object
        Continuous-time solution.
    ctrl : object
        Continuous-time solution.
    base : object
        Base values.

    """
    # Recognize the motor type by checking if the rotor flux data exist
    try:
        if mdl.psi_Rs is not None:
            motor_type = 'im'
    except AttributeError:
        motor_type = 'sm'

    t_range = (0, ctrl.t[-1])   # Time span

    fig, (ax1, ax2, ax3, ax4, ax5) = plt.subplots(5, 1, figsize=(8, 10))

    ax1.step(ctrl.t, ctrl.w_m_ref/base.w, '--', where='post')
    ax1.plot(mdl.t, mdl.w_m/base.w)
    if motor_type == 'sm':
        ax1.step(ctrl.t, ctrl.w_m/base.w, where='post')
        ax1.legend([r'$\omega_\mathrm{m,ref}$',
                    r'$\omega_\mathrm{m}$',
                    r'$\hat \omega_\mathrm{m}$'])
    else:
        ax1.step(ctrl.t, ctrl.w_s/base.w, where='post')
        ax1.legend([r'$\omega_\mathrm{m,ref}$',
                    r'$\omega_\mathrm{m}$',
                    r'$\hat \omega_\mathrm{m}$'])
    ax1.set_xlim(t_range)
    ax1.set_xticklabels([])
    ax1.set_ylabel('Speed (p.u.)')

    ax2.plot(mdl.t, mdl.tau_L/base.tau, '--')
    ax2.plot(mdl.t, mdl.tau_M/base.tau)
    try:
        ax2.step(ctrl.t, ctrl.tau_M/base.tau)  # Limited torque reference
        ax2.legend([r'$\tau_\mathrm{L}$', r'$\tau_\mathrm{M}$',
                    r'$\tau_\mathrm{M,ref}$'])
    except AttributeError:
        ax2.legend([r'$\tau_\mathrm{L}$', r'$\tau_\mathrm{M}$'])
    ax2.set_xlim(t_range)
    ax2.set_ylabel('Torque (p.u.)')
    ax2.set_xticklabels([])

    ax3.step(ctrl.t, ctrl.i_s_ref.real/base.i, '--', where='post')
    ax3.step(ctrl.t, ctrl.i_s.real/base.i, where='post')
    ax3.step(ctrl.t, ctrl.i_s_ref.imag/base.i, '--', where='post')
    ax3.step(ctrl.t, ctrl.i_s.imag/base.i, where='post')
    ax3.set_ylabel('Current (p.u.)')
    ax3.legend([r'$i_\mathrm{sd,ref}$', r'$i_\mathrm{sd}$',
                r'$i_\mathrm{sq,ref}$', r'$i_\mathrm{sq}$'])
    ax3.set_xlim(t_range)
    ax3.set_xticklabels([])

    ax4.step(ctrl.t, np.abs(ctrl.u_s)/base.u, where='post')
    ax4.step(ctrl.t, ctrl.u_dc/np.sqrt(3)/base.u, '--', where='post')
    ax4.set_ylabel('Voltage (p.u.)')
    ax4.set_xlim(t_range)
    # ax4.set_ylim(0, 1.2)
    ax4.legend([r'$u_\mathrm{s}$', r'$u_\mathrm{dc}/\sqrt{3}$'])
    ax4.set_xticklabels([])

    if motor_type == 'sm':
        ax5.plot(mdl.t, np.abs(mdl.psi_s)/base.psi)
        ax5.step(ctrl.t, np.abs(ctrl.psi_s)/base.psi, '--', where='post')
        ax5.legend([r'$\psi_\mathrm{s}$', r'$\hat\psi_\mathrm{s}$'])
    else:
        ax5.plot(mdl.t, np.abs(mdl.psi_ss)/base.psi)
        ax5.plot(mdl.t, np.abs(mdl.psi_Rs)/base.psi)
        try:
            ax5.plot(ctrl.t, np.abs(ctrl.psi_R)/base.psi)
            ax5.legend([r'$\psi_\mathrm{s}$', r'$\psi_\mathrm{R}$',
                        r'$\hat \psi_\mathrm{R}$'])
        except AttributeError:
            ax5.legend([r'$\psi_\mathrm{s}$', r'$\psi_\mathrm{R}$'])
    ax5.set_xlim(t_range)
    ax5.set_ylabel('Flux (p.u.)')
    ax5.set_xlabel('Time (s)')

    fig.align_ylabels()
    plt.tight_layout()


# %%
def plot_im_extra(mdl, ctrl, base):
    """
    Plots extra waveforms for induction motors.

    Parameters
    ----------
    mdl : object
        Continuous-time solution.
    ctrl : object
        Continuous-time solution.
    base : object
        Base values.

    """
    # Time span
    t_zoom = (.9, .925)

    # Quantities in stator coordinates
    ctrl.u_ss = np.exp(1j*ctrl.theta_s)*ctrl.u_s
    ctrl.i_ss = np.exp(1j*ctrl.theta_s)*ctrl.i_s

    fig1, (ax1, ax2) = plt.subplots(2, 1)
    ax1.plot(mdl.t, mdl.u_ss.real/base.u)
    ax1.plot(ctrl.t, ctrl.u_ss.real/base.u)
    ax1.set_xlim(t_zoom)
    ax1.set_ylim(-1.5, 1.5)
    ax1.legend([r'$u_\mathrm{sa}$', r'$\hat u_\mathrm{sa}$'])
    ax1.set_ylabel('Voltage (p.u.)')
    ax1.set_xticklabels([])
    ax2.plot(mdl.t, complex2abc(mdl.i_ss).T/base.i)
    ax2.step(ctrl.t, ctrl.i_ss.real/base.i, where='post')
    ax2.set_xlim(t_zoom)
    ax2.legend([r'$i_\mathrm{sa}$', r'$i_\mathrm{sb}$',
                r'$i_\mathrm{sc}$'])
    ax2.set_ylabel('Current (p.u.)')
    ax2.set_xlabel('Time (s)')
    fig1.align_ylabels()

    # Plots the DC bus and grid-side variables (if exist)
    try:
        mdl.i_L
    except AttributeError:
        mdl.i_L = None
    if mdl.i_L is not None:
        fig2, (ax1, ax2) = plt.subplots(2, 1)
        ax1.plot(mdl.t, mdl.u_di/base.u)
        ax1.plot(mdl.t, mdl.u_dc/base.u)
        ax1.plot(mdl.t, complex2abc(mdl.u_g).T/base.u)
        ax1.set_xlim(t_zoom)
        ax1.set_ylim(-1.5, 2)
        ax1.set_xticklabels([])
        ax1.legend([r'$u_\mathrm{di}$',
                    r'$u_\mathrm{dc}$',
                    r'$u_\mathrm{ga}$'])
        ax1.set_ylabel('Voltage (p.u.)')
        ax2.plot(mdl.t, mdl.i_L/base.i)
        ax2.plot(mdl.t, mdl.i_dc/base.i)
        ax2.plot(mdl.t, mdl.i_g.real/base.i)
        ax2.set_xlim(t_zoom)
        ax2.legend([r'$i_\mathrm{L}$',
                    r'$i_\mathrm{dc}$',
                    r'$i_\mathrm{ga}$'])
        ax2.set_ylabel('Current (p.u.)')
        ax2.set_xlabel('Time (s)')
        fig2.align_ylabels()

    plt.tight_layout()
    plt.show()
