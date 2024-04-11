:py:mod:`motulator.control.im`
==============================

.. py:module:: motulator.control.im

.. autoapi-nested-parse::

   
   This package contains example controllers for induction machines.
















   ..
       !! processed by numpydoc !!


Package Contents
----------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.im.CurrentCtrl
   motulator.control.im.CurrentReferencePars
   motulator.control.im.CurrentReference
   motulator.control.im.ModelPars
   motulator.control.im.VectorCtrl
   motulator.control.im.Observer
   motulator.control.im.FluxObserver
   motulator.control.im.ObserverBasedVHzCtrl
   motulator.control.im.ObserverBasedVHzCtrlPars
   motulator.control.im.VHzCtrl




.. py:class:: CurrentCtrl(par, alpha_c)


   Bases: :py:obj:`motulator.control._common.ComplexPICtrl`

   
   2DOF PI current controller for induction machines.

   This class provides an interface for a current controller for induction
   machines. The gains are initialized based on the desired closed-loop
   bandwidth and the leakage inductance.

   :param par: Machine parameters, contains the leakage inductance `L_sgm` (H).
   :type par: ModelPars
   :param alpha_c: Closed-loop bandwidth (rad/s).
   :type alpha_c: float















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentReferencePars


   
   Parameters for reference generation.

   This dataclass stores the nominal and limit values needed for reference
   generation. For calculating the rotor flux reference, the machine
   parameters are also required.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param u_s_nom: Nominal stator voltage (V). The default is sqrt(2/3)*400.
   :type u_s_nom: float, optional
   :param w_s_nom: Nominal stator angular frequency (rad/s). The default is 2*pi*50.
   :type w_s_nom: float, optional
   :param psi_R_nom: Nominal rotor flux linkage (Vs). The default is
                     `(u_s_nom/w_s_nom)/(1 + L_sgm/L_M)`.
   :type psi_R_nom: float, optional
   :param k_fw: Field-weakening gain (1/H). The default is `2*R_R/(w_s_nom*L_sgm**2)`.
   :type k_fw: float, optional
   :param k_u: Voltage utilization factor. The default is 0.95.
   :type k_u: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentReference(par, ref)


   
   Current reference generation.

   In the base-speed region, the current reference in rotor-flux coordinates
   is given by::

       i_s_ref = psi_R_nom/L_M + 1j*tau_M_ref/(1.5*n_p*abs(psi_R))

   where `psi_R_nom` is the nominal rotor flux magnitude and `psi_R` is the
   estimated rotor flux. The field-weakening operation is based on adjusting
   the flux-producing current component::

       i_s_ref.real = (k_fw/s)*(u_s_max - abs(u_s_ref))

   where `1/s` refers to integration, ``u_s_max = k_u*u_dc/sqrt(3)`` is the
   maximum stator voltage in the linear modulation region, `u_s_ref` is the
   (unlimited) stator voltage reference, and `k_fw` is the field-weakening
   gain. The field-weakening method and its tuning corresponds roughly to
   [#Hin2006]_. Furthermore, the torque-producing current component
   `i_s_ref.imag` is limited based on the maximum stator current and the
   breakdown slip.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param ref: Reference generation parameters.
   :type ref: CurrentReferencePars

   .. rubric:: References

   .. [#Hin2006] Hinkkanen, Luomi, "Braking scheme for vector-controlled
      induction motor drives equipped with diode rectifier without braking
      resistor," IEEE Trans. Ind. Appl., 2006,
      https://doi.org/10.1109/TIA.2006.880852















   ..
       !! processed by numpydoc !!
   .. py:method:: output(tau_M_ref, psi_R)

      
      Compute the stator current reference.

      :param tau_M_ref: Torque reference (Nm).
      :type tau_M_ref: float
      :param psi_R: Rotor flux magnitude (Vs).
      :type psi_R: float

      :returns: * **i_s_ref** (*complex*) -- Stator current reference (A).
                * **tau_M_ref_lim** (*float*) -- Limited torque reference (Nm).















      ..
          !! processed by numpydoc !!

   .. py:method:: update(T_s, u_s_ref, u_dc)

      
      Field-weakening based on the unlimited reference voltage.

      :param T_s: Sampling period (s)
      :type T_s: float
      :param u_s_ref: Unlimited stator voltage reference (V).
      :type u_s_ref: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float















      ..
          !! processed by numpydoc !!


.. py:class:: ModelPars


   
   Inverse-Γ model parameters of an induction machine.

   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_R: Rotor resistance (Ω).
   :type R_R: float
   :param L_sgm: Leakage inductance (H).
   :type L_sgm: float
   :param L_M: Magnetizing inductance (H).
   :type L_M: float
   :param n_p: Number of pole pairs.
   :type n_p: int
   :param J: Moment of inertia (kgm²).
   :type J: float















   ..
       !! processed by numpydoc !!

.. py:class:: VectorCtrl(par, ref, T_s=0.00025, sensorless=True)


   Bases: :py:obj:`motulator.control._common.Ctrl`

   
   Vector control for induction machine drives.

   This class interconnects the subsystems of the control system and provides
   the interface to the solver.

   :param par: Machine model parameters.
   :type par: InductionMachinePars
   :param ref: Parameters for reference generation.
   :type ref: CurrentReferencePars
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float, optional
   :param sensorless: If True, sensorless control is used. The default is True.
   :type sensorless: bool, optional

   .. attribute:: current_ref

      Current reference generator.

      :type: CurrentReference

   .. attribute:: observer

      Flux and speed observer.

      :type: Observer

   .. attribute:: current_ctrl

      Current controller.

      :type: CurrentCtrl

   .. attribute:: speed_ctrl

      Speed controller.

      :type: SpeedCtrl

   .. attribute:: pwm

      Pulse-width modulation.

      :type: PWM

   .. attribute:: w_m_ref

      Speed reference (electrical rad/s) as a function of time (s).

      :type: callable















   ..
       !! processed by numpydoc !!

.. py:class:: Observer(par, k=None, alpha_o=2 * np.pi * 40, sensorless=True)


   
   Reduced-order flux observer for induction machines in estimated
   rotor flux coordinates.

   This class implements a reduced-order flux observer for induction machines.
   Both sensored and sensorless operation are supported. The observer
   structure is similar to [#Hin2010]_. The observer operates in estimated
   rotor flux coordinates.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param k: Observer gain as a function of the rotor angular speed. The default is
             ``lambda w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)`` if
             `sensorless` else ``lambda w_m: 1 + 0.2*abs(w_m)/(R_R/L_M - 1j*w_m)``.
   :type k: callable, optional
   :param alpha_o: Observer bandwidth (rad/s). The default is 2*pi*40.
   :type alpha_o: float, optional
   :param sensorless: If True, sensorless mode is used. The default is True.
   :type sensorless: bool, optional

   .. attribute:: psi_R

      Rotor flux magnitude estimate (Vs).

      :type: float

   .. attribute:: theta_s

      Rotor flux angle estimate (rad).

      :type: float

   .. attribute:: w_m

      Rotor angular speed estimate (electrical rad/s). In sensored mode, this
      is the measured low-pass-filtered speed.

      :type: float

   .. rubric:: Notes

   The pure voltage model corresponds to ``k = lambda w_m: 0``, resulting in
   the marginally stable estimation-error dynamics. The current model is
   obtained by setting ``k = lambda w_m: 1``.

   .. rubric:: References

   .. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers
      with stator-resistance adaptation for speed-sensorless induction motor
      drives," IEEE Trans. Power Electron., 2010,
      https://doi.org/10.1109/TPEL.2009.2039650















   ..
       !! processed by numpydoc !!

.. py:class:: FluxObserver(par, alpha_o, b=None)


   
   Sensorless reduced-order flux observer in external coordinates.

   This is a sensorless reduced-order flux observer in synchronous coordinates
   for an induction machine. The observer gain decouples the electrical and
   mechanical dynamics and allows placing the poles of the linearized
   estimation error dynamics. This implementation operates in external
   coordinates (typically synchronous coordinates defined by reference signals
   of a control system).

   :param par: Machine model parameters.
   :type par: ModelPars
   :param alpha_o: Speed-estimation bandwidth (rad/s).
   :type alpha_o: float
   :param b: Coefficient (rad/s) of the characteristic polynomial as a function of
             the rotor angular speed estimate. The default is
             ``lambda w_m: R_R/L_M + .4*abs(w_m)``.
   :type b: callable, optional

   .. attribute:: psi_R

      Rotor flux estimate (Vs).

      :type: complex

   .. attribute:: w_m

      Rotor angular speed estimate (electrical rad/s).

      :type: float

   .. rubric:: Notes

   The characteristic polynomial of the observer in synchronous coordinates is
   ``s**2 + b*s + w_s**2``.















   ..
       !! processed by numpydoc !!
   .. py:method:: update(T_s, u_s, i_s, w_s)

      
      Update the states.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_s: Stator voltage (V) in synchronous coordinates.
      :type u_s: complex
      :param i_s: Stator current (A) in synchronous coordinates.
      :type i_s: complex
      :param w_s: Angular frequency (rad/s) of the coordinate system.
      :type w_s: float















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedVHzCtrl(par, ctrl_par, T_s=0.00025)


   Bases: :py:obj:`motulator.control._common.Ctrl`

   
   Observer-based V/Hz control for induction machines.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param ctrl_par: Control system parameters.
   :type ctrl_par: ObserverBasedVHzCtrlPars
   :param T_s: Sampling period (s). The default is 250e-6.
   :type T_s: float, optional

   .. attribute:: observer

      Sensorless reduced-order flux observer in external coordinates.

      :type: SensorlessObserverExtCoord

   .. attribute:: rate_limiter

      Rate limiter for the speed reference.

      :type: RateLimiter

   .. attribute:: pwm

      Pulse-width modulation.

      :type: PWM

   .. attribute:: w_m_ref

      Speed reference (electrical rad/s) as a function of time (s).

      :type: callable















   ..
       !! processed by numpydoc !!

.. py:class:: ObserverBasedVHzCtrlPars


   
   Parameters for the control system.

   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A). The default is inf.
   :type i_s_max: float, optional
   :param k_tau: Torque controller gain. The default is 3.
   :type k_tau: float, optional
   :param alpha_psi: Stator flux control bandwidth (rad/s). The default is 2*pi*20.
   :type alpha_psi: float, optional
   :param alpha_f: Torque high-pass filter bandwidth (rad/s). The default is 2*pi*1.
   :type alpha_f: float, optional
   :param alpha_r: Low-pass-filter bandwidth (rad/s) for slip angular frequency. The
                   default is 2*pi*1.
   :type alpha_r: float, optional
   :param slip_compensation: Enable slip compensation. The default is False.
   :type slip_compensation: bool, optional















   ..
       !! processed by numpydoc !!

.. py:class:: VHzCtrl(T_s, par, psi_s_nom, k_u=1.0, k_w=4.0, six_step=False)


   Bases: :py:obj:`motulator.control._common.Ctrl`

   
   V/Hz control with the stator current feedback.

   :param par: Control parameters.
   :type par: ModelPars















   ..
       !! processed by numpydoc !!

