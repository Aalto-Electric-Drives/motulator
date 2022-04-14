# pylint: disable=C0103
"""
This file contains computation and plotting of torque characteristics for
synchronous machines, including the MTPA and MTPV loci. The methods can be used
to define look-up tables for control as well as to analyze the characteristics.
In this version, the magnetic saturation is omitted.

"""
from sys import float_info
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler

# %%
plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
plt.rcParams['lines.linewidth'] = 1.
plt.rcParams.update({"text.usetex": False})  # Disable LaTeX in plots


# %%
class TorqueCharacteristics:
    """
    Computes MTPA and MTPV loci based on the motor parameters. The magnetic
    saturation is omitted.

    """

    def __init__(self, pars):
        """
        Parameters
        ----------
        pars : data object
            Motor parameters.

        """
        self.p = pars.p
        self.L_d = pars.L_d
        self.L_q = pars.L_q
        self.psi_f = pars.psi_f
        try:
            self.i_sd_min = pars.i_sd_min
        except AttributeError:
            pass

    def torque(self, psi_s):
        """
        Compute the torque as a function of the stator flux linkage.

        Parameters
        ----------
        psi_s : complex
            Stator flux.

        Returns
        -------
        tau_M : float
            Electromagnetic torque.

        """
        i_s = self.current(psi_s)
        tau_M = 1.5*self.p*np.imag(i_s*np.conj(psi_s))

        return tau_M

    def current(self, psi_s):
        """
        Compute the stator current as a function of the stator flux linkage.

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

    def flux(self, i_s):
        """
        Compute the stator flux linkage as a function of the current.

        Parameters
        ----------
        i_s : complex
            Stator current.

        Returns
        -------
        psi_s : complex
            Stator flux linkage.

        """
        psi_s = self.L_d*i_s.real + self.psi_f + 1j*self.L_q*i_s.imag

        return psi_s

    def mtpa(self, abs_i_s):
        """
        Compute the MTPA stator current angle.

        Parameters
        ----------
        abs_i_s : float
            Stator current magnitude.

        Returns
        -------
        beta : float
            MTPA angle of the stator current vector.

        """
        # Replace zeros with epsilon
        abs_i_s = (abs_i_s > 0)*abs_i_s + (abs_i_s <= 0)*float_info.epsilon

        if self.psi_f == 0:
            # SyRM (d-axis aligned with the maximum inductance)
            beta = .25*np.pi
        elif self.L_d == self.L_q:
            # Nonsalient machine
            beta = .5*np.pi
        else:
            # Salient machine
            a = self.psi_f/((self.L_q - self.L_d)*abs_i_s)

            if self.L_q > self.L_d:
                beta = np.arccos(.25*(a - np.sqrt(a**2 + 8)))
            else:
                beta = np.arccos(.25*(a + np.sqrt(a**2 + 8)))

        return beta

    def mtpv(self, abs_psi_s):
        """
        Compute the MTPV stator flux angle.

        Parameters
        ----------
        abs_psi_s : float
            Stator flux magnitude.

        Returns
        -------
        delta : float
            MTPV angle of the stator flux vector.

        """
        # Replace zeros with epsilon
        abs_psi_s = ((abs_psi_s > 0)*abs_psi_s
                     + (abs_psi_s <= 0)*float_info.epsilon)

        if self.psi_f == 0:
            # SyRM (d-axis aligned with the maximum inductance)
            delta = .25*np.pi
        elif self.L_d == self.L_q:
            # Nonsalient machine
            delta = .5*np.pi
        else:
            # Salient machine
            a = self.L_q/(self.L_q - self.L_d)*self.psi_f/abs_psi_s

            if self.L_q > self.L_d:
                delta = np.arccos(.25*(a - np.sqrt(a**2 + 8)))
            else:
                delta = np.arccos(.25*(a + np.sqrt(a**2 + 8)))

        return delta

    def mtpv_current(self, abs_i_s):
        """
        Compute the MTPV based on the current magnitude, i.e., the intersection
        of the MTPV current locus and the current limit circle. This method is
        not necessary for computing the control look-up tables. It is used here
        to "cut" the MTPV characteristics at the desired current. Alternatively
        just a large enough maximum flux magnitude could be used.

        Parameters
        ----------
        abs_i_s : float
            Stator current magnitude.

        Returns
        -------
        i_s : complex
            MTPV stator current.

        """
        if self.psi_f == 0:
            # SyRM
            i_s = abs_i_s*np.exp(1j*(np.arctan(self.L_d/self.L_q)))
        elif self.psi_f/self.L_d < abs_i_s:
            if self.L_d == self.L_q:
                # Nonsalient machine
                i_sd = -self.psi_f/self.L_d
                i_sq = np.sqrt(abs_i_s**2 - i_sd**2)
                i_s = i_sd + 1j*i_sq
            else:
                # Salient machine
                k = self.L_q/(self.L_d - self.L_q)
                a = self.L_d**2 + self.L_q**2
                b = (2 + k)*self.psi_f*self.L_d
                c = (1 + k)*self.psi_f**2 - (self.L_q*abs_i_s)**2
                if self.L_q > self.L_d:
                    i_sd = .5*(-b - np.sqrt(b**2 - 4*a*c))/a
                else:
                    i_sd = .5*(-b + np.sqrt(b**2 - 4*a*c))/a
                i_sq = np.sqrt(abs_i_s**2 - i_sd**2)
                i_s = i_sd + 1j*i_sq
        else:
            # No MTPV for the given current magnitude
            i_s = None  # float('nan')

        return i_s

    def mtpa_locus(self, i_s_max, N=20):
        """
        Compute the MTPA locus.

        Parameters
        ----------
        i_s_max : float
            Maximum current at which the locus is computed.
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        i_s : complex
            Current space vectors at the MTPA locus.

        """
        abs_i_s = np.linspace(0, i_s_max, N)
        beta = self.mtpa(abs_i_s)
        i_s = abs_i_s*np.exp(1j*beta)

        # Minimum d-axis current (needed for sensorless SyRM drives)
        try:
            i_s.real = ((i_s.real < self.i_sd_min)*self.i_sd_min
                        + (i_s.real >= self.i_sd_min)*i_s.real)
        except AttributeError:
            pass

        return i_s

    def mtpv_locus(self, psi_s_max, N=20):
        """
        Compute the MTPV locus.

        Parameters
        ----------
        psi_s_max : float
            Maximum stator flux at which the locus is computed.
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        psi_s : complex
            MTPV stator flux.

        """
        abs_psi_s = np.linspace(0, psi_s_max, N)
        delta = self.mtpv(abs_psi_s)
        psi_s = abs_psi_s*np.exp(1j*delta)

        return psi_s

    def delta_at_zero_torque(self, abs_psi_s):
        """
        Compute the "nontrivial" load angle value corresponding to the zero
        electromagnetic torque.

        Parameters
        ----------
        abs_psi_s : float
            Stator flux magnitude.

        Returns
        -------
        delta : float
            Load angle at the zero torque.

        """
        if self.psi_f > 0:
            c = ((self.L_q - self.L_d)/self.L_q*abs_psi_s/self.psi_f)**2 - 1
            if c > 0:
                if self.L_q > self.L_d:
                    delta = np.arctan((np.sqrt(c)))
                else:
                    delta = np.pi - np.arctan((np.sqrt(c)))
            else:
                delta = 0
        else:
            delta = 0

        return delta

    def plot_flux_loci(self, i_s_max, base, N=20):
        """
        Plot stator flux linkage loci using per-unit quantities.

        Parameters
        ----------
        psi_s_max : float
            Maximum flux at which the loci are evaluated.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        Returns
        -------
        None.

        """
        # Compute the constant current locus
        theta = np.linspace(0, np.pi, 2*N)
        i_s_clim = i_s_max*np.exp(1j*theta)
        psi_s_clim = self.flux(i_s_clim)

        # Compute the characteristics
        i_s_mtpa = self.mtpa_locus(i_s_max, N)
        psi_s_mtpa = self.flux(i_s_mtpa)
        i_s_mtpv_max = self.mtpv_current(i_s_max)
        psi_s_mtpv_max = self.flux(i_s_mtpv_max)
        psi_s_mtpv = self.mtpv_locus(np.abs(psi_s_mtpv_max), N)

        # Plot the i_sd--i_sq current plane
        _, ax = plt.subplots()
        ax.plot(psi_s_mtpa.real/base.psi, psi_s_mtpa.imag/base.psi,
                label='MTPA')
        try:
            ax.plot(psi_s_mtpv.real/base.psi, psi_s_mtpv.imag/base.psi,
                    label='MTPV')
            ax.plot(psi_s_mtpv_max.real/base.psi,
                    psi_s_mtpv_max.imag/base.psi, 'o')
        except AttributeError:
            pass
        ax.plot(psi_s_mtpa.real[-1]/base.psi,
                psi_s_mtpa.imag[-1]/base.psi, 'o')
        ax.plot(psi_s_clim.real/base.psi, psi_s_clim.imag/base.psi,
                label='Const current')
        ax.legend()
        ax.set_xlabel(r'$\psi_\mathrm{sd}$ (p.u.)')
        ax.set_ylabel(r'$\psi_\mathrm{sq}$ (p.u.)')
        ax.set_ylim(0, None)
        ax.set_aspect('equal')

    def plot_current_loci(self, i_s_max, base, N=20):
        """
        Plot current loci using per-unit quantities.

        Parameters
        ----------
        i_s_max : float
            Maximum current at which the loci are evaluated.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        Returns
        -------
        None.

        """
        # Compute the current limit for plotting
        theta = np.linspace(0, np.pi, 2*N)
        i_s_clim = i_s_max*np.exp(1j*theta)

        # Compute the characteristics
        i_s_mtpa = self.mtpa_locus(i_s_max, N)
        i_s_mtpv_max = self.mtpv_current(i_s_max)
        psi_s_mtpv_max = self.flux(i_s_mtpv_max)
        psi_s_mtpv = self.mtpv_locus(np.abs(psi_s_mtpv_max), N)
        i_s_mtpv = self.current(psi_s_mtpv)

        # Plot the i_sd--i_sq current plane
        _, ax = plt.subplots()
        ax.plot(i_s_mtpa.real/base.i, i_s_mtpa.imag/base.i, label='MTPA')
        ax.plot(i_s_mtpa.real[-1]/base.i, i_s_mtpa.imag[-1]/base.i, 'o')
        try:
            ax.plot(i_s_mtpv.real/base.i, i_s_mtpv.imag/base.i, label='MTPV')
            ax.plot(i_s_mtpv_max.real/base.i, i_s_mtpv_max.imag/base.i, 'o')
        except AttributeError:
            pass
        ax.plot(-i_s_clim.real/base.i, i_s_clim.imag/base.i,
                label='Const current')
        ax.set_xlabel(r'$i_\mathrm{sd}$ (p.u.)')
        ax.set_ylabel(r'$i_\mathrm{sq}$ (p.u.)')
        ax.legend()
        if self.psi_f == 0:
            ax.axis([0, i_s_max/base.i, 0, i_s_max/base.i])
        elif self.L_q > self.L_d:
            ax.axis([-i_s_max/base.i, 0, 0, i_s_max/base.i])
        else:
            ax.axis([-i_s_max/base.i, i_s_max/base.i, 0, i_s_max/base.i])
        ax.set_aspect('equal', 'box')

    def plot_torque_current(self, i_s_max, base, N=20):
        """
        Plot torque vs. current characteristics using per-unit quantities.

        Parameters
        ----------
        i_s_max : float
            Maximum current at which the loci are evaluated.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        Returns
        -------
        None.

        """
        # Compute the characteristics
        i_s_mtpa = self.mtpa_locus(i_s_max, N)
        psi_s_mtpa = self.flux(i_s_mtpa)
        tau_M_mtpa = self.torque(psi_s_mtpa)
        i_s_mtpv_max = self.mtpv_current(i_s_max)
        psi_s_mtpv_max = np.abs(self.flux(i_s_mtpv_max))
        psi_s_mtpv = self.mtpv_locus(psi_s_mtpv_max, N)
        i_s_mtpv = self.current(psi_s_mtpv)
        if i_s_mtpv.any():  # is not None:
            tau_M_mtpv = self.torque(psi_s_mtpv)
        else:
            tau_M_mtpv = None     # No MTPV in finite speed drives

        # Plot tau_M vs. abs(i_s)
        _, (ax1, ax2) = plt.subplots(2, 1)
        ax1.plot(tau_M_mtpa/base.tau, np.abs(i_s_mtpa)/base.i)
        try:
            ax1.plot(tau_M_mtpv/base.tau, np.abs(i_s_mtpv)/base.i)
        except TypeError:
            pass
        ax1.set_xlim(0, i_s_max/base.i)
        ax1.set_ylim(0, None)
        ax1.legend(['MTPA', 'MTPV'])
        ax1.set_ylabel(r'$i_\mathrm{s}$ (p.u.)')

        # Plot tau_M vs. i_sd
        ax2.plot(tau_M_mtpa/base.tau, np.real(i_s_mtpa)/base.i)
        try:
            ax2.plot(tau_M_mtpv/base.tau, i_s_mtpv.real/base.i)
        except AttributeError:
            pass
        ax2.set_xlim(0, i_s_max/base.i)
        if self.psi_f == 0:
            ax2.set_ylim(0, None)
        elif self.L_q > self.L_d:
            ax2.set_ylim(None, 0)
        ax2.legend(['MTPA', 'MTPV'])
        ax2.set_xlabel(r'$\tau_\mathrm{M}$ (p.u.)')
        ax2.set_ylabel(r'$i_\mathrm{sd}$ (p.u.)')

    def plot_angle_torque(self, abs_psi_s, base, N=100):
        """
        Plot the electromagnetic torque as a function of the load angle.

        Parameters
        ----------
        abs_psi_s : float
            Stator flux magnitude.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 100.

        Returns
        -------
        None.

        """
        delta = np.linspace(-np.pi, np.pi, N)
        psi_s = abs_psi_s*np.exp(1j*delta)
        tau_M = self.torque(psi_s)

        delta_mtpv = self.mtpv(abs_psi_s)
        psi_s_mtpv = abs_psi_s*np.exp(1j*delta_mtpv)
        tau_M_mtpv = self.torque(psi_s_mtpv)

        delta0 = self.delta_at_zero_torque(abs_psi_s)
        psi_s0 = abs_psi_s*np.exp(1j*delta0)
        tau_M0 = self.torque(psi_s0)

        _, ax = plt.subplots()
        ax.plot(180*delta/np.pi, tau_M/base.tau)
        ax.plot(180*delta_mtpv/np.pi, tau_M_mtpv/base.tau, 'o')
        ax.plot(180*delta0/np.pi, tau_M0/base.tau, 'x')
        ax.set_xlim([-180, 180])
        ax.set_xticks([-180, -135, -90, -45, 0, 45, 90, 135, 180])
        ax.set_xlabel(r'$\delta$ (deg)')
        ax.set_ylabel(r'$\tau_\mathrm{m}$ (p.u.)')
        ax.set_title(r'$\psi_\mathrm{s}=$ %1.2f p.u.' % (abs_psi_s/base.psi))
