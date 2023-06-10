"""
Torque characteristics for synchronous machines.

This contains computation and plotting of torque characteristics for synchronous 
machines, including the MTPA and MTPV loci [#Mor1990]_. The methods can be used 
to define look-up tables for control and to analyze the characteristics. This 
version omits the magnetic saturation.

References
----------
.. [#Mor1990] Morimoto, Takeda, Hirasa, Taniguchi, "Expansion of operating 
   limits for permanent magnet motor by current vector control considering 
   inverter capacity," IEEE Trans. Ind. Appl., 1990,
   https://doi.org/10.1109/28.60058

"""

from sys import float_info
import numpy as np
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt
from cycler import cycler
from motulator._utils import Bunch

plt.rcParams["axes.prop_cycle"] = cycler(color="brgcmyk")
plt.rcParams["lines.linewidth"] = 1.
plt.rcParams.update({"text.usetex": False})  # Disable LaTeX in plots


# %%
class TorqueCharacteristics:
    """
    Compute MTPA and MTPV loci based on the machine parameters.

    The magnetic saturation is omitted.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.

    """

    def __init__(self, par):
        self.n_p = par.n_p
        self.L_d = par.L_d
        self.L_q = par.L_q
        self.psi_f = par.psi_f

    def torque(self, psi_s):
        """
        Compute the torque as a function of the stator flux linkage.

        Parameters
        ----------
        psi_s : complex
            Stator flux (Vs).

        Returns
        -------
        tau_M : float
            Electromagnetic torque (Nm).

        """
        i_s = self.current(psi_s)
        tau_M = 1.5*self.n_p*np.imag(i_s*np.conj(psi_s))

        return tau_M

    def current(self, psi_s):
        """
        Compute the stator current as a function of the stator flux linkage.

        Parameters
        ----------
        psi_s : complex
            Stator flux linkage (Vs).

        Returns
        -------
        i_s : complex
            Stator current (A).

        """
        i_s = (psi_s.real - self.psi_f)/self.L_d + 1j*psi_s.imag/self.L_q

        return i_s

    def flux(self, i_s):
        """
        Compute the stator flux linkage as a function of the current.

        Parameters
        ----------
        i_s : complex
            Stator current (A).

        Returns
        -------
        psi_s : complex
            Stator flux linkage (Vs).

        """
        psi_s = self.L_d*i_s.real + self.psi_f + 1j*self.L_q*i_s.imag

        return psi_s

    def mtpa(self, abs_i_s):
        """
        Compute the MTPA stator current angle.

        Parameters
        ----------
        abs_i_s : float
            Stator current magnitude (A).

        Returns
        -------
        beta : float
            MTPA angle of the stator current vector (electrical rad).

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
            Stator flux magnitude (Vs).

        Returns
        -------
        delta : float
            MTPV angle of the stator flux vector (electrical rad).

        """
        # Replace zeros with epsilon
        abs_psi_s = ((abs_psi_s > 0)*abs_psi_s +
                     (abs_psi_s <= 0)*float_info.epsilon)

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
            Stator current magnitude (A).

        Returns
        -------
        i_s : complex
            MTPV stator current (A).

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

    def mtpa_locus(self, i_s_max, psi_s_min=None, N=20):
        """
        Compute the MTPA locus.

        Parameters
        ----------
        i_s_max : float
            Maximum stator current magnitude (A) at which the locus is computed.
        psi_s_min : float, optional
            Minimum stator flux magnitude (Vs) at which the locus is computed.
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        Bunch object with the following fields defined:
        psi_s : complex
            Stator flux (Vs).
        i_s : complex
            Stator current (A).
        tau_M : float
            Electromagnetic torque (Nm).
        abs_psi_s_vs_tau_M : callable
            Stator flux magnitude (Vs) as a function of the torque (Nm).
        i_sd_vs_tau_M : callable
            d-axis current (A) as a function of the torque (Nm).

        """
        # Current  magnitudes
        abs_i_s = np.linspace(0, i_s_max, N)

        # MTPA locus expressed with different quantities
        beta = self.mtpa(abs_i_s)
        i_s = abs_i_s*np.exp(1j*beta)

        if (psi_s_min is not None) and (self.psi_f == 0):
            # Minimum d-axis current for sensorless SyRM drives
            i_sd_min = psi_s_min/self.L_d
            i_s.real = ((i_s.real < i_sd_min)*i_sd_min +
                        (i_s.real >= i_sd_min)*i_s.real)

        psi_s = self.flux(i_s)
        abs_psi_s = np.abs(psi_s)
        tau_M = self.torque(psi_s)

        # Create an interpolant that can be used as a look-up table. If needed,
        # more interpolants can be easily added.
        abs_psi_s_vs_tau_M = interp1d(
            tau_M, abs_psi_s, fill_value=abs_psi_s[-1], bounds_error=False)
        i_sd_vs_tau_M = interp1d(
            tau_M, i_s.real, fill_value=i_s.real[-1], bounds_error=False)

        # Return the result as a bunch object
        return Bunch(
            psi_s=psi_s,
            i_s=i_s,
            tau_M=tau_M,
            abs_psi_s_vs_tau_M=abs_psi_s_vs_tau_M,
            i_sd_vs_tau_M=i_sd_vs_tau_M)

    def mtpv_locus(self, psi_s_max=None, i_s_max=None, N=20):
        """
        Compute the MTPV locus.

        Parameters
        ----------
        psi_s_max : float, optional
            Maximum stator flux magnitude (Vs) at which the locus is computed. 
            Either `psi_s_max` or `i_s_max` must be given.
        i_s_max : float, optional
            Maximum stator current magnitude (A) at which the locus is computed.
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        Bunch object with the following fields defined:
        psi_s : complex
            Stator flux (Vs).
        i_s : complex
            Stator current (A).
        tau_M : float
            Electromagnetic torque (Nm).
        tau_M_vs_abs_psi_s : interp1d object
            Torque (Nm) as a function of the flux magnitude (Vs).

        """
        # If i_s_max is given, compute the corresponding MTPV stator flux
        if i_s_max is not None:
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
        return Bunch(
            psi_s=psi_s,
            i_s=i_s,
            tau_M=tau_M,
            tau_M_vs_abs_psi_s=tau_M_vs_abs_psi_s)

    def current_limit(self, i_s_max, gamma1=np.pi, gamma2=0, N=20):
        """
        Compute the current limit.

        Parameters
        ----------
        i_s_max : float
            Current limit (A). 
        gamma1 : float, optional
            Starting angle (electrical rad). The default is 0.
        gamma2 : float, optional
            End angle (electrical rad). The defauls in pi.
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        Bunch object with the following fields defined:
        psi_s : complex
            Stator flux (Vs).
        i_s : complex
            Stator current (A).
        tau_M : float
            Electromagnetic torque (Nm).
        tau_M_vs_abs_psi_s : interp1d object
            Torque (Nm) as a function of the flux magnitude (Vs).

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
        tau_M_vs_abs_psi_s = interp1d(
            np.abs(psi_s),
            tau_M,
            bounds_error=False,
            fill_value=(tau_M[0], tau_M[-1]))

        # Return the result as a bunch object
        return Bunch(
            psi_s=psi_s,
            i_s=i_s,
            tau_M=tau_M,
            tau_M_vs_abs_psi_s=tau_M_vs_abs_psi_s)

    def mtpv_and_current_limits(self, i_s_max, N=20):
        """
        Merge the MTPV and current limits into a single interpolant.

        Parameters
        ----------
        i_s_max : float
            Current limit (A).
        N : int, optional
            Amount of points. The default is 20.

        Returns
        -------
        Bunch object with the following fields defined:
        tau_M_vs_abs_psi_s : interp1d object
            Torque (Nm) as a function of the flux magnitude (Vs).
        i_sd_vs_tau_M : interp1d object
            d-axis current (A) as a function of the torque (Nm).

        """
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(
            i_s_max=i_s_max,
            N=N,
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
        tau_M_vs_abs_psi_s = interp1d(
            np.abs(psi_s),
            tau_M,
            bounds_error=False,
            fill_value=(tau_M[0], tau_M[-1]))
        i_sd_vs_tau_M = interp1d(tau_M, i_sd, fill_value="extrapolate")

        # Return the result as a bunch object
        return Bunch(
            tau_M_vs_abs_psi_s=tau_M_vs_abs_psi_s, i_sd_vs_tau_M=i_sd_vs_tau_M)

    def plot_flux_loci(self, i_s_max, base, N=20):
        """
        Plot the stator flux linkage loci.

        Per-unit quantities are used.

        Parameters
        ----------
        i_s_max : float
            Maximum current (A) at which the loci are evaluated.
        base : BaseValues
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        """
        # Compute the characteristics
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(
            i_s_max=i_s_max,
            N=N,
            gamma1=np.angle(mtpv.i_s[-1]),
            gamma2=np.angle(mtpa.i_s[-1]))

        # Plot the i_sd--i_sq current plane
        _, ax = plt.subplots()
        ax.plot(
            mtpa.psi_s.real/base.psi, mtpa.psi_s.imag/base.psi, label="MTPA")
        try:
            ax.plot(
                mtpv.psi_s.real/base.psi,
                mtpv.psi_s.imag/base.psi,
                label="MTPV")
        except AttributeError:
            pass
        ax.plot(
            clim.psi_s.real/base.psi,
            clim.psi_s.imag/base.psi,
            label="Const current")

        ax.legend()
        ax.set_xlabel(r"$\psi_\mathrm{sd}$ (p.u.)")
        ax.set_ylabel(r"$\psi_\mathrm{sq}$ (p.u.)")
        ax.set_ylim(0, None)
        ax.set_aspect("equal")

    def plot_current_loci(self, i_s_max, base, N=20):
        """
        Plot the current loci.

        Per-unit quantities are used.

        Parameters
        ----------
        i_s_max : float
            Maximum current (A) at which the loci are evaluated.
        base : BaseValues
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        """
        # Compute the characteristics
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(
            i_s_max=i_s_max,
            N=N,
            gamma1=np.angle(mtpv.i_s[-1]),
            gamma2=np.angle(mtpa.i_s[-1]))

        # Plot the i_sd--i_sq current plane
        _, ax = plt.subplots()
        ax.plot(mtpa.i_s.real/base.i, mtpa.i_s.imag/base.i, label="MTPA")
        try:
            ax.plot(mtpv.i_s.real/base.i, mtpv.i_s.imag/base.i, label="MTPV")
        except AttributeError:
            pass
        ax.plot(
            clim.i_s.real/base.i, clim.i_s.imag/base.i, label="Const current")

        ax.set_xlabel(r"$i_\mathrm{sd}$ (p.u.)")
        ax.set_ylabel(r"$i_\mathrm{sq}$ (p.u.)")
        ax.legend()
        if self.psi_f == 0:
            ax.axis([0, i_s_max/base.i, 0, i_s_max/base.i])
        elif self.L_q > self.L_d:
            ax.axis([-i_s_max/base.i, 0, 0, i_s_max/base.i])
        else:
            ax.axis([-i_s_max/base.i, i_s_max/base.i, 0, i_s_max/base.i])
        ax.set_aspect("equal")

    def plot_torque_current(self, i_s_max, base, N=20):
        """
        Plot torque vs. current characteristics.

        Per-unit quantities are used.

        Parameters
        ----------
        i_s_max : float
            Maximum current (A) at which the loci are evaluated.
        base : BaseValues
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        """
        # Compute the characteristics
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(
            i_s_max=i_s_max,
            N=N,
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
        ax1.legend(["MTPA", "MTPV", "Const current"])
        ax1.set_ylabel(r"$i_\mathrm{sd}$ (p.u.)")

        # Plot i_sq vs. tau_M
        ax2.plot(mtpa.tau_M/base.tau, mtpa.i_s.imag/base.i)
        try:
            ax2.plot(mtpv.tau_M/base.tau, mtpv.i_s.imag/base.i)
        except TypeError:
            pass
        ax2.plot(clim.tau_M/base.tau, clim.i_s.imag/base.i)

        ax2.set_xlim(0, i_s_max/base.i)
        ax2.set_ylim(0, None)
        ax2.legend(["MTPA", "MTPV", "Const current"])
        ax2.set_ylabel(r"$i_\mathrm{sq}$ (p.u.)")
        ax2.set_xlabel(r"$\tau_\mathrm{M}$ (p.u.)")

    def plot_torque_flux(self, i_s_max, base, N=20):
        """
        Plot torque vs. flux magnitude characteristics.

        Per-unit quantities are used.

        Parameters
        ----------
        i_s_max : float
            Maximum current (A) at which the loci are evaluated.
        base : BaseValues
            Base values.
        N : int, optional
            Amount of points to be evaluated. The default is 20.

        """
        # Compute the characteristics
        mtpa = self.mtpa_locus(i_s_max=i_s_max, N=N)
        mtpv = self.mtpv_locus(i_s_max=i_s_max, N=N)
        clim = self.current_limit(
            i_s_max=i_s_max,
            N=N,
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

        ax.legend(["MTPA", "MTPV", "Const current"])
        ax.set_xlabel(r"$\psi_\mathrm{s}$ (p.u.)")
        ax.set_ylabel(r"$\tau_\mathrm{m}$ (p.u.)")
