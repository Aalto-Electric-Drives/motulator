# pylint: disable=C0103
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler

# %%
plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
plt.rcParams['lines.linewidth'] = 1.
plt.rcParams.update({"text.usetex": False})  # Disable LaTeX in plots


# %%
class OptimalLoci:
    """
    Computes MTPA and MTPV loci based on the motor parameters. The magnetic
    saturation is omitted. Note that these loci could be better implemented
    using normalized quantities.

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

    def torque(self, i_s):
        """
        Computes the torque.

        Parameters
        ----------
        i_s : complex
            Stator current space vector.

        Returns
        -------
        tau_M : float
            Electromagnetic torque.

        """
        psi_s = self.L_d*i_s.real + 1j*self.L_q*i_s.imag + self.psi_f
        tau_M = 1.5*self.p*np.imag(i_s*np.conj(psi_s))
        return tau_M

    def mtpa(self, i_s_max, N=20):
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
        if self.psi_f == 0:
            # Magnetically linear SyRM
            i_sd = np.linspace(0, i_s_max/np.sqrt(2), N)
            i_sq = i_sd
            # Minimum d-axis current
            i_sd = ((i_sd < self.i_sd_min)*self.i_sd_min
                    + (i_sd >= self.i_sd_min)*i_sd)
            i_s = i_sd + 1j*i_sq
        else:
            # IPMSM
            abs_i_s = np.linspace(0, i_s_max, N)
            i_a = self.psi_f/(self.L_d - self.L_q)
            i_sd = -i_a/4 - np.sqrt((i_a**2)/16 + (abs_i_s**2)/2)
            i_sq = np.sqrt(abs_i_s**2 - i_sd**2)
            i_s = i_sd + 1j*i_sq
        return i_s

    def mtpv(self, i_s_max, N=20):
        if self.psi_f == 0:
            # Magnetically linear SyRM
            abs_i_s = np.linspace(0, i_s_max, N)
            i_s = abs_i_s*np.exp(1j*(np.arctan(self.L_d/self.L_q)))
        elif self.psi_f/self.L_d < i_s_max:
            # IPMSM
            abs_i_s = np.linspace(self.psi_f/self.L_d, i_s_max, N)
            k = self.L_q/(self.L_d - self.L_q)
            a = self.L_d**2 + self.L_q**2
            b = (2 + k)*self.psi_f*self.L_d
            c = (1 + k)*self.psi_f**2 - (self.L_q*abs_i_s)**2
            i_sd = .5*(-b - np.sqrt(b**2 - 4*a*c))/a
            i_sq = np.sqrt(abs_i_s**2 - i_sd**2)
            i_s = i_sd + 1j*i_sq
        return i_s

    def plot(self, i_s_max, base, N=20):
        """
        Plots control loci using per-unit quantities.

        Parameters
        ----------
        i_s_max : float
            Maximum current at which the loci are evaluated.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        """
        # Compute the current limit for plotting
        theta = np.linspace(0, np.pi, 2*N)
        i_s_lim = i_s_max*np.exp(1j*theta)

        # Compute the characteristics
        i_s_mtpa = self.mtpa(i_s_max, N)
        tau_M_mtpa = self.torque(i_s_mtpa)
        i_s_mtpv = self.mtpv(i_s_max, N)
        if i_s_mtpv.any():  # is not None:
            tau_M_mtpv = self.torque(i_s_mtpv)
        else:
            tau_M_mtpv = None     # No MTPV in finite speed drives

        # Plot the i_sd--i_sq current plane
        fig1, ax = plt.subplots(1, 1)
        ax.plot(i_s_mtpa.real/base.i, i_s_mtpa.imag/base.i)
        try:
            ax.plot(i_s_mtpv.real/base.i, i_s_mtpv.imag/base.i)
        except AttributeError:
            pass
        ax.plot(-i_s_lim.real/base.i, i_s_lim.imag/base.i)
        ax.set_xlabel(r'$i_\mathrm{sd}$ (p.u.)')
        ax.set_ylabel(r'$i_\mathrm{sq}$ (p.u.)')
        ax.legend(['MTPA', 'MTPV'])
        if self.psi_f == 0:
            # SyRM
            ax.axis([0, i_s_max/base.i, 0, i_s_max/base.i])
        else:
            ax.axis([-i_s_max/base.i, 0, 0, i_s_max/base.i])
        ax.set_aspect('equal', 'box')

        # Plot i_sd vs. tau_M
        fig2, ax = plt.subplots(1, 1)
        ax.plot(tau_M_mtpa/base.tau, np.real(i_s_mtpa)/base.i)
        try:
            ax.plot(tau_M_mtpv/base.tau, i_s_mtpv.real/base.i)
        except AttributeError:
            pass
        ax.legend(['MTPA', 'MTPV'])
        ax.set_xlabel(r'$\tau_\mathrm{M}$ (p.u.)')
        ax.set_ylabel(r'$i_\mathrm{sd}$ (p.u.)')
        ax.set_xlim(0, None)
        if self.psi_f == 0:
            # SyRM
            ax.set_ylim(0, None)
        else:
            ax.set_ylim(None, 0)
        ax.set_xlim(0, np.max(tau_M_mtpa)/base.tau)

        # Plot tau_M vs. abs(i_s)
        fig3, ax = plt.subplots(1, 1)
        ax.plot(np.abs(i_s_mtpa)/base.i, tau_M_mtpa/base.tau)
        try:
            ax.plot(np.abs(i_s_mtpv)/base.i, tau_M_mtpv/base.tau)
        except TypeError:
            pass
        ax.legend(['MTPA', 'MTPV'])
        ax.set_xlabel(r'$i_\mathrm{s}$ (p.u.)')
        ax.set_ylabel(r'$\tau_\mathrm{M}$ (p.u.)')
        ax.set_xlim(0, i_s_max//base.i)
        ax.set_ylim(0, None)
