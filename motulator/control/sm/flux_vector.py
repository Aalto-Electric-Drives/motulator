"""
Flux-vector control of synchronous machine drives.

This module cointains a version of stator-flux-vector control [Pel2009]_. Rotor
coordinates as well as decoupling between the stator flux and torque channels 
are used [Awa2019]_. Here, the stator flux magnitude and the electromagnetic 
torque are selected as controllable variables. Proportional controllers are 
used for simplicity. The magnetic saturation is not considered in this
implementation.

References
----------
.. [Pel2009] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented control 
   of IPM drives with variable DC link in the field-weakening region,” IEEE 
   Trans.Ind. Appl., 2009, https://doi.org/10.1109/TIA.2009.2027167

.. [Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control 
   of synchronous motors: A systematic design procedure," IEEE Trans. Ind. 
   Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

"""

from dataclasses import dataclass, InitVar
import numpy as np
from motulator.helpers import abc2complex, Bunch
from motulator.control.common import Ctrl, PWM, SpeedCtrl
from motulator.control.sm.torque import TorqueCharacteristics
from motulator.control.sm.vector import SensorlessObserver, ModelPars


# %%
class FluxVectorCtrl(Ctrl):
    """
    Flux-vector control of synchronous machine drives.

    This class interconnects the subsystems of the control system and provides 
    the interface to the solver.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    ref : FluxTorqueReferencePars
        Reference generation parameters.
    alpha_psi : float, optional
        Bandwidth of the flux controller (rad/s). The default is 2*np.pi*150.
    alpha_tau : float, optional
        Bandwidth of the torque controller (rad/s). The default is 2*np.pi*50.
    T_s : float
        Sampling period (s). The default is 250e-6.
    sensorless : bool, optional
        If True, sensorless control is used. The default is True.

    Attributes
    ----------
    observer : Observer | SensorlessObserver
        Observer.
    flux_torque_ref : FluxTorqueReference
        Flux and torque reference generator.
    speed_ctrl : SpeedCtrl
        Speed controller.
    w_m_ref : float
        Speed reference (electrical rad/s) as a function of time (s).

    """

    # pylint: disable=too-many-instance-attributes, too-many-arguments
    def __init__(
            self,
            par,
            ref,
            alpha_psi=2*np.pi*150,
            alpha_tau=2*np.pi*50,
            T_s=250e-6,
            sensorless=True):
        super().__init__()
        self.T_s = T_s
        self.sensorless = sensorless
        # Machine model parameters
        self.n_p = par.n_p
        self.R_s, self.L_d, self.L_q = par.R_s, par.L_d, par.L_q
        # Subsystems
        if sensorless:
            self.observer = SensorlessObserver(par, w_o=2*np.pi*100)
        else:
            self.observer = Observer(par, g=2*np.pi*15)
        self.flux_torque_ref = FluxTorqueReference(ref)
        self.pwm = PWM()
        self.speed_ctrl = SpeedCtrl(par.J, 2*np.pi*4)
        # Bandwidths
        self.alpha_psi = alpha_psi
        self.alpha_tau = alpha_tau
        # Speed reference
        self.w_m_ref = callable

    # pylint: disable=too-many-locals
    def __call__(self, mdl):
        """
        Run the main control loop.

        Parameters
        ----------
        mdl : Drive
            Continuous-time system model for getting the feedback signals.

        Returns
        -------
        T_s : float
            Sampling period (s).
        d_abc : ndarray, shape (3,)
            Duty ratios.

        """
        # Get the speed reference
        w_m_ref = self.w_m_ref(self.clock.t)

        # Feedback signals
        i_s_abc = mdl.machine.meas_currents()  # Phase currents
        u_dc = mdl.converter.meas_dc_voltage()  # DC-bus voltage
        u_s = self.pwm.realized_voltage  # Realized voltage from PWM
        #
        if self.sensorless:
            # Get the rotor speed and position estimates
            w_m, theta_m = self.observer.w_m, self.observer.theta_m
        else:
            # Measure the rotor speed
            w_m = self.n_p*mdl.mechanics.meas_speed()
            # Limit the electrical rotor position into [-pi, pi)
            theta_m = np.mod(
                self.n_p*mdl.mechanics.meas_position() + np.pi,
                2*np.pi) - np.pi

        # Current vector in estimated rotor coordinates
        i_s = np.exp(-1j*theta_m)*abc2complex(i_s_abc)

        # Flux and torque estimates
        psi_s = self.observer.psi_s
        tau_M = 1.5*self.n_p*np.imag(i_s*np.conj(psi_s))

        # Outputs
        tau_M_ref = self.speed_ctrl.output(w_m_ref/self.n_p, w_m/self.n_p)
        psi_s_ref, tau_M_ref_lim = self.flux_torque_ref(tau_M_ref, w_m, u_dc)

        # Auxiliary current
        i_a = psi_s.real/self.L_q + 1j*psi_s.imag/self.L_d - i_s

        # Torque-production factor (c_tau = 0 corresponds to the MTPV condition)
        c_tau = np.real(i_a*np.conj(psi_s))

        # References for the flux and torque controllers
        v_psi = self.alpha_psi*(psi_s_ref - np.abs(psi_s))
        v_tau = self.alpha_tau*(tau_M_ref_lim - tau_M)
        if c_tau > 0:
            v = (np.abs(psi_s)*i_a*v_psi + 1j*psi_s*v_tau)/c_tau
        else:
            v = v_psi

        # Stator voltage reference
        u_s_ref = self.R_s*i_s + 1j*w_m*psi_s + v

        # Data logging
        data = Bunch(
            i_s=i_s,
            psi_s=psi_s,
            psi_s_ref=psi_s_ref,
            t=self.clock.t,
            tau_M_ref_lim=tau_M_ref_lim,
            theta_m=theta_m,
            u_dc=u_dc,
            u_s=u_s,
            w_m=w_m,
            w_m_ref=w_m_ref,
        )
        self.save(data)

        # Update states
        self.observer.update(self.T_s, u_s, i_s, w_m)
        self.speed_ctrl.update(self.T_s, tau_M_ref_lim)
        self.clock.update(self.T_s)

        # PWM output
        d_abc = self.pwm(self.T_s, u_s_ref, u_dc, theta_m, w_m)

        return self.T_s, d_abc


# %%
@dataclass
class FluxTorqueReferencePars:
    """
    Parameters for reference generation.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    i_s_max : float
        Maximum stator current (A). 
    psi_s_min : float, optional
        Minimum stator flux (Vs). The default is `psi_f`.
    psi_s_max : float, optional
        Maximum stator flux (Vs). The default is `inf`.
    k_u : float, optional
        Voltage utilization factor. The default is 0.95.

    Attributes
    ----------
    psi_s_mtpa : callable
        MTPA stator flux linkage (Vs) as a function of the torque (Nm).
    tau_M_lim : callable
        Torque limit (Nm) as a function of the stator flux linkage (Vs). This limit merges the MTPV and current limits. 

    """
    par: InitVar[ModelPars | None]
    i_s_max: float = None
    psi_s_min: float = None
    psi_s_max: float = np.inf
    k_u: float = 0.95

    def __post_init__(self, par):
        self.psi_s_min = par.psi_f if self.psi_s_min is None else self.psi_s_min
        # Generate LUTs
        tq = TorqueCharacteristics(par)
        mtpa = tq.mtpa_locus(self.i_s_max, self.psi_s_min)
        lim = tq.mtpv_and_current_limits(self.i_s_max)
        # MTPA locus
        self.psi_s_mtpa = mtpa.abs_psi_s_vs_tau_M
        # Merged MTPV and current limits
        self.tau_M_lim = lim.tau_M_vs_abs_psi_s


# %%
class FluxTorqueReference:
    """
    Flux and torque references.

    The current and MTPV limits as well as the MTPA locus are implemented as
    look-up tables, which are generated based on the constant machine model
    parameters.

    Parameters
    ----------
    ref : FluxTorqueReferencePars
        Reference generation parameters.

    """

    # pylint: disable=too-many-arguments
    def __init__(self, ref):
        self.psi_s_min, self.psi_s_max = ref.psi_s_min, ref.psi_s_max
        self.k_u = ref.k_u
        # Merged MTPV and current limits
        self.tau_M_lim = ref.tau_M_lim
        # MTPA locus
        self.psi_s_mtpa = ref.psi_s_mtpa

    def __call__(self, tau_M_ref, w_m, u_dc):
        """
        Calculate the stator flux reference and limit the torque reference.

        Parameters
        ----------
        tau_M_ref : float
            Unlimited torque reference (Nm).
        w_m : float
            Rotor speed or its reference (electrical rad/s).
        u_dc : float
            DC-bus voltage (V).

        Returns
        -------
        psi_s_ref : float
            Stator flux reference (Vs).
        tau_M_ref_lim : float
            Limited torque reference (Nm).

        """
        # Get the MTPA flux
        psi_s_mtpa = self.psi_s_mtpa(np.abs(tau_M_ref))
        psi_s_mtpa = np.clip(psi_s_mtpa, self.psi_s_min, self.psi_s_max)

        # Field weakening
        u_s_max = self.k_u*u_dc/np.sqrt(3)
        psi_s_max = u_s_max/np.abs(w_m) if np.abs(w_m) > 0 else np.inf

        # Flux reference
        psi_s_ref = np.min([psi_s_max, psi_s_mtpa])

        # Limit the torque reference according to the MTPV and current limits
        tau_M_lim = self.tau_M_lim(psi_s_ref)
        tau_M_ref_lim = np.min([tau_M_lim, np.abs(tau_M_ref)
                                ])*np.sign(tau_M_ref)

        return psi_s_ref, tau_M_ref_lim


# %%
class Observer:
    """
    Sensored stator flux observer.

    Parameters
    ----------
    par : ModelPars
        Machine model parameters.
    g : float, optional
        Observer gain (rad/s). The default is 2*pi*15.

    """

    # pylint: disable=too-many-instance-attributes
    def __init__(self, par, g=2*np.pi*15):
        self.R_s = par.R_s
        self.L_d, self.L_q, self.psi_f = par.L_d, par.L_q, par.psi_f
        self.n_p = par.n_p
        self.g = g
        # State
        self.psi_s = par.psi_f

    def update(self, T_s, u_s, i_s, w_m):
        """
        Update the states for the next sampling period.

        Parameters
        ----------
        T_s : float
            Sampling period (s).
        u_s : complex
            Stator voltage (V) in estimated rotor coordinates.
        i_s : complex
            Stator current (A) in estimated rotor coordinates.
        w_m : float
            Rotor angular speed (electrical rad/s).

        """
        # Estimation error
        e = self.L_d*i_s.real + 1j*self.L_q*i_s.imag + self.psi_f - self.psi_s

        # Update the state
        self.psi_s += T_s*(u_s - self.R_s*i_s - 1j*w_m*self.psi_s + self.g*e)
