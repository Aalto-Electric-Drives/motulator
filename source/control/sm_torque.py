# pylint: disable=C0103
"""
Torque characteristics for synchronous machines.

This contains computation and plotting of torque characteristics for
synchronous machines, including the MTPA and MTPV loci. The methods can be used
to define look-up tables for control as well as to analyze the characteristics.
In this version, the magnetic saturation is omitted.

"""
from sys import float_info
import numpy as np
import matplotlib.pyplot as plt
from cycler import cycler
from scipy.interpolate import interp1d
from sklearn.utils import Bunch

# %%
plt.rcParams['axes.prop_cycle'] = cycler(color='brgcmyk')
plt.rcParams['lines.linewidth'] = 1.
plt.rcParams.update({"text.usetex": False})  # Disable LaTeX in plots


# %%
class TorqueCharacteristics:
    """
    Compute MTPA and MTPV loci based on the motor parameters.

    The magnetic saturation is omitted.

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
        try:
            self.i_sd_max = pars.i_sd_max
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
        Compute the MTPV based on the current magnitude.

        This computes the MTPV based on the current magnitude, i.e., the
        intersection of the MTPV current locus and the current limit circle.
        This method is not necessary for computing the control look-up tables.
        It is used here to "cut" the MTPV characteristics at the desired
        current. Alternatively just a large enough maximum flux magnitude could
        be used.

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
            i_s = np.nan

        return i_s

    def mtpa_locus(self, i_s_max=1, N=20):
        """
        Compute the MTPA locus.

        Parameters
        ----------
        i_s_max : float, optional
            Maximum stator current magnitude at which the locus is computed.
            The default is 1.
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        Bunch object with the following fields defined:
        psi_s : complex
            Stator flux.
        i_s : complex
            Stator current.
        tau_m : float
            Electromagnetic torque.
        abs_psi_s_vs_tau_M : interp1d object
            Stator flux magnitude as a function of the torque.
        i_sd_vs_tau_M : interp1d object
            d-axis current as a function of the torque.

        """
        # Current  magnitudes
        abs_i_s = np.linspace(0, i_s_max, N)

        # MTPA locus expressed with different quantities
        beta = self.mtpa(abs_i_s)
        i_s = abs_i_s*np.exp(1j*beta)

        # Minimum d-axis current for sensorless SyRM drives
        try:
            i_s.real = ((i_s.real < self.i_sd_min)*self.i_sd_min
                        + (i_s.real >= self.i_sd_min)*i_s.real)
        except AttributeError:
            pass
        # Maximum d-axis current for sensorless PM-SyRM drives
        try:
            i_s.real = ((i_s.real > self.i_sd_max)*self.i_sd_max
                        + (i_s.real <= self.i_sd_max)*i_s.real)
        except AttributeError:
            pass

        psi_s = self.flux(i_s)
        tau_M = self.torque(psi_s)

        # Create an interpolant that can be used as a look-up table. If needed,
        # more interpolants can be easily added.
        abs_psi_s_vs_tau_M = interp1d(tau_M, np.abs(psi_s),
                                      fill_value="extrapolate")
        i_sd_vs_tau_M = interp1d(tau_M, i_s.real,
                                 fill_value="extrapolate")

        # Return the result as a bunch object
        return Bunch(psi_s=psi_s, i_s=i_s, tau_M=tau_M,
                     abs_psi_s_vs_tau_M=abs_psi_s_vs_tau_M,
                     i_sd_vs_tau_M=i_sd_vs_tau_M)

    def mtpv_locus(self, psi_s_max=1, i_s_max=None, N=20):
        """
        Compute the MTPV locus.

        Parameters
        ----------
        psi_s_max : float, optional
            Maximum stator flux magnitude at which the locus is computed. The
            default is 1.
        i_s_max : float, optional
            Maximum stator current magnitude at which the locus is computed.
            The default is None.
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        Bunch object with the following fields defined:
        psi_s : complex
            Stator flux.
        i_s : complex
            Stator current.
        tau_m : float
            Electromagnetic torque.
        tau_M_vs_abs_psi_s : interp1d object
            Torque as a function of the flux magnitude.

        """
        # If i_s_max is given, compute the corresponding MTPV stator flux
        if i_s_max:
            i_s_mtpv = self.mtpv_current(i_s_max)
            psi_s_max = np.abs(self.flux(i_s_mtpv))

        # Flux magnitudes
        abs_psi_s = np.linspace(0, psi_s_max, N)

        # MTPV locus expressed with different quantities
        delta = self.mtpv(abs_psi_s)
        psi_s = abs_psi_s*np.exp(1j*delta)
        i_s = self.current(psi_s)
        tau_M = self.torque(psi_s)

        # Create an interpolant that can be used as a look-up table. If needed,
        # more interpolants can be easily added.
        tau_M_vs_abs_psi_s = interp1d(np.abs(psi_s), tau_M, bounds_error=False)

        # Return the result as a bunch object
        return Bunch(psi_s=psi_s, i_s=i_s, tau_M=tau_M,
                     tau_M_vs_abs_psi_s=tau_M_vs_abs_psi_s)

    def current_limit(self, i_s_max=1, gamma1=np.pi, gamma2=0, N=20):
        """
        Compute the current limit.

        Parameters
        ----------
        i_s_max : float, optional
            Current limit. The default is 1.
        gamma1 : float, optional
            Starting angle in radians. The default is 0.
        gamm21 : float, optional
            End angle in radians. The defauls in np.pi.
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        Bunch object with the following fields defined:
        psi_s : complex
            Stator flux.
        i_s : complex
            Stator current.
        tau_m : float
            Electromagnetic torque.
        tau_M_vs_abs_psi_s : interp1d object
            Torque as a function of the flux magnitude.

        """
        if np.isnan(gamma1):
            # No MTPV
            gamma1 = np.pi

        gamma = np.linspace(gamma1, gamma2, N)

        # MTPA locus expressed with different quantities
        i_s = i_s_max*np.exp(1j*gamma)
        psi_s = self.flux(i_s)
        tau_M = self.torque(psi_s)

        # Create an interpolant that can be used as a look-up table. If needed,
        # more interpolants can be easily added.
        tau_M_vs_abs_psi_s = interp1d(np.abs(psi_s), tau_M,
                                      bounds_error=False,
                                      fill_value=(tau_M[0], tau_M[-1]))

        # Return the result as a bunch object
        return Bunch(psi_s=psi_s, i_s=i_s, tau_M=tau_M,
                     tau_M_vs_abs_psi_s=tau_M_vs_abs_psi_s)

    def mtpv_and_current_limits(self, i_s_max=1, N=20):
        """
        Merge the MTPV and current limits into a single interpolant.

        Parameters
        ----------
        i_s_max : float, optional
            Current limit. The default is 1.
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        Bunch object with the following fields defined:
        tau_M_vs_abs_psi_s : interp1d object
            Torque as a function of the flux magnitude.
        i_sd_vs_tau_M : interp1d object
            d-axis current as a function of the torque.

        """
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(i_s_max=i_s_max, N=N,
                                  gamma1=np.angle(mtpv.i_s[-1]),
                                  gamma2=np.angle(mtpa.i_s[-1]))

        if np.isnan(mtpv.i_s).any():
            # No MTPV, only the current limit
            psi_s = clim.psi_s
            tau_M = clim.tau_M
            i_sd = clim.i_s.real
        else:
            # Concatenate the MTPV and current limits
            psi_s = np.concatenate((mtpv.psi_s, clim.psi_s))
            tau_M = np.concatenate((mtpv.tau_M, clim.tau_M))
            i_sd = np.concatenate((mtpv.i_s.real, clim.i_s.real))

        # Create an interpolant that can be used as a look-up table
        tau_M_vs_abs_psi_s = interp1d(np.abs(psi_s), tau_M,
                                      bounds_error=False,
                                      fill_value=(tau_M[0], tau_M[-1]))
        i_sd_vs_tau_M = interp1d(tau_M, i_sd,
                                 fill_value="extrapolate")

        # Return the result as a bunch object
        return Bunch(tau_M_vs_abs_psi_s=tau_M_vs_abs_psi_s,
                     i_sd_vs_tau_M=i_sd_vs_tau_M)

    def delta_at_zero_torque(self, abs_psi_s):
        """
        Compute the load angle value at the zero torque.

        This computes the "nontrivial" load angle value corresponding to the
        zero electromagnetic torque.

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
        Plot the stator flux linkage loci.

        Per-unit quantities are used.

        Parameters
        ----------
        i_s_max : float
            Maximum current at which the loci are evaluated.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        """
        # Compute the characteristics
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(i_s_max=i_s_max, N=N,
                                  gamma1=np.angle(mtpv.i_s[-1]),
                                  gamma2=np.angle(mtpa.i_s[-1]))

        # Plot the i_sd--i_sq current plane
        _, ax = plt.subplots()
        ax.plot(mtpa.psi_s.real/base.psi, mtpa.psi_s.imag/base.psi,
                label='MTPA')
        try:
            ax.plot(mtpv.psi_s.real/base.psi, mtpv.psi_s.imag/base.psi,
                    label='MTPV')
        except AttributeError:
            pass
        ax.plot(clim.psi_s.real/base.psi, clim.psi_s.imag/base.psi,
                label='Const current')

        ax.legend()
        ax.set_xlabel(r'$\psi_\mathrm{sd}$ (p.u.)')
        ax.set_ylabel(r'$\psi_\mathrm{sq}$ (p.u.)')
        ax.set_ylim(0, None)
        ax.set_aspect('equal')

    def plot_current_loci(self, i_s_max, base, N=20):
        """
        Plot the current loci.

        Per-unit quantities are used.

        Parameters
        ----------
        i_s_max : float
            Maximum current at which the loci are evaluated.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        """
        # Compute the characteristics
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(i_s_max=i_s_max, N=N,
                                  gamma1=np.angle(mtpv.i_s[-1]),
                                  gamma2=np.angle(mtpa.i_s[-1]))

        # Plot the i_sd--i_sq current plane
        _, ax = plt.subplots()
        ax.plot(mtpa.i_s.real/base.i, mtpa.i_s.imag/base.i, label='MTPA')
        try:
            ax.plot(mtpv.i_s.real/base.i, mtpv.i_s.imag/base.i, label='MTPV')
        except AttributeError:
            pass
        ax.plot(clim.i_s.real/base.i, clim.i_s.imag/base.i,
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
        ax.set_aspect('equal')

    def plot_torque_current(self, i_s_max, base, N=20):
        """
        Plot torque vs. current characteristics.

        Per-unit quantities are used.

        Parameters
        ----------
        i_s_max : float
            Maximum current at which the loci are evaluated.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        """
        # Compute the characteristics
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(i_s_max=i_s_max, N=N,
                                  gamma1=np.angle(mtpv.i_s[-1]),
                                  gamma2=np.angle(mtpa.i_s[-1]))

        # Plot i_sd vs. tau_M
        _, (ax1, ax2) = plt.subplots(2, 1)
        ax1.plot(mtpa.tau_M/base.tau, mtpa.i_s.real/base.i)
        try:
            ax1.plot(mtpv.tau_M/base.tau, mtpv.i_s.real/base.i)
        except AttributeError:
            pass
        ax1.plot(clim.tau_M/base.tau, clim.i_s.real/base.i)

        ax1.set_xlim(0, i_s_max/base.i)
        if self.psi_f == 0:
            ax1.set_ylim(0, None)
        elif self.L_q > self.L_d:
            ax1.set_ylim(None, 0)
        ax1.legend(['MTPA', 'MTPV', 'Const current'])
        ax1.set_ylabel(r'$i_\mathrm{sd}$ (p.u.)')

        # Plot i_sq vs. tau_M
        ax2.plot(mtpa.tau_M/base.tau, mtpa.i_s.imag/base.i)
        try:
            ax2.plot(mtpv.tau_M/base.tau, mtpv.i_s.imag/base.i)
        except TypeError:
            pass
        ax2.plot(clim.tau_M/base.tau, clim.i_s.imag/base.i)

        ax2.set_xlim(0, i_s_max/base.i)
        ax2.set_ylim(0, None)
        ax2.legend(['MTPA', 'MTPV', 'Const current'])
        ax2.set_ylabel(r'$i_\mathrm{sq}$ (p.u.)')
        ax2.set_xlabel(r'$\tau_\mathrm{M}$ (p.u.)')

    def plot_torque_flux(self, i_s_max, base, N=20):
        """
        Plot torque vs. flux magnitude characteristics.

        Per-unit quantities are used.

        Parameters
        ----------
        i_s_max : float
            Maximum current at which the loci are evaluated.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        """
        # Compute the characteristics
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(i_s_max=i_s_max, N=N,
                                  gamma1=np.angle(mtpv.i_s[-1]),
                                  gamma2=np.angle(mtpa.i_s[-1]))

        # Plot
        _, ax = plt.subplots(1, 1)
        ax.plot(np.abs(mtpa.psi_s)/base.psi, mtpa.tau_M/base.tau)
        try:
            ax.plot(np.abs(mtpv.psi_s)/base.psi, mtpv.tau_M/base.tau)
        except AttributeError:
            pass
        ax.plot(np.abs(clim.psi_s)/base.psi, clim.tau_M/base.tau)

        ax.legend(['MTPA', 'MTPV', 'Const current'])
        ax.set_xlabel(r'$\psi_\mathrm{s}$ (p.u.)')
        ax.set_ylabel(r'$\tau_\mathrm{m}$ (p.u.)')

    def plot_angle_torque(self, abs_psi_s, base, N=100):
        """
        Plot the electromagnetic torque as a function of the load angle.

        Per-unit quantities are used.

        Parameters
        ----------
        abs_psi_s : float
            Stator flux magnitude.
        base : object
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 100.

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
