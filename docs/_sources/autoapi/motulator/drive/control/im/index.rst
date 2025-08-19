motulator.drive.control.im
==========================

.. py:module:: motulator.drive.control.im

.. autoapi-nested-parse::

   Controls for induction machine drives.

   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.control.im.CurrentController
   motulator.drive.control.im.CurrentReferenceGenerator
   motulator.drive.control.im.CurrentVectorController
   motulator.drive.control.im.CurrentVectorControllerCfg
   motulator.drive.control.im.FluxObserver
   motulator.drive.control.im.FluxVectorController
   motulator.drive.control.im.FluxVectorControllerCfg
   motulator.drive.control.im.InductionMachineInvGammaPars
   motulator.drive.control.im.ObserverBasedVHzController
   motulator.drive.control.im.ObserverBasedVHzControllerCfg
   motulator.drive.control.im.PIController
   motulator.drive.control.im.ReferenceGenerator
   motulator.drive.control.im.SpeedController
   motulator.drive.control.im.SpeedFluxObserver
   motulator.drive.control.im.VHzControlSystem
   motulator.drive.control.im.VectorControlSystem


Module Contents
---------------

.. py:class:: CurrentController(par, alpha_c, alpha_i = None)

   Bases: :py:obj:`motulator.common.control.ComplexPIController`


   
   Current controller for induction machines.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param alpha_c: Reference-tracking bandwidth (rad/s).
   :type alpha_c: float, optional
   :param alpha_i: Integral action bandwidth (rad/s), defaults to `alpha_c`.
   :type alpha_i: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentReferenceGenerator(par, psi_s_nom, i_s_max, w_s_nom, k_u = 1.0, k_fw = 0.0)

   
   Current reference generator.

   In the base-speed region, the current reference in rotor-flux coordinates is::

       i_s_ref = i_sd_nom + 1j*tau_M_ref/(1.5*n_p*abs(psi_R))

   where `psi_R` is the estimated rotor flux. The nominal flux-producing current
   component is computed from the nominal stator flux in the no-load condition::

       i_sd_nom = psi_s_nom/(L_M + L_sgm)

   In the field-weakening operation, the flux-producing current component is::

       i_s_ref.real = (k_fw/s)*(u_s_max - abs(u_s_ref))

   where `1/s` refers to integration, ``u_s_max = k_u*u_dc/sqrt(3)`` is the maximum
   stator voltage in the linear modulation region, `u_s_ref` is the (unlimited) stator
   voltage reference, and `k_fw` is the field-weakening gain. The field-weakening
   method and its tuning corresponds roughly to [#Hin2006]_. Furthermore, the torque-
   producing current component `i_s_ref.imag` is limited based on the maximum stator
   current and the breakdown slip.

   :param machine_pars: Machine model parameters.
   :type machine_pars: InductionMachineInvGammaPars | InductionMachinePars
   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param w_s_nom: Nominal stator angular frequency (rad/s).
   :type w_s_nom: float, optional
   :param k_u: Voltage utilization factor, defaults to 1.
   :type k_u: float, optional
   :param k_fw: Field-weakening gain (1/H), defaults to `2*R_R/(w_s_nom*L_sgm**2)`.
   :type k_fw: float, optional

   .. rubric:: References

   .. [#Hin2006] Hinkkanen, Luomi, "Braking scheme for vector-controlled induction
      motor drives equipped with diode rectifier without braking resistor," IEEE Trans.
      Ind. Appl., 2006, https://doi.org/10.1109/TIA.2006.880852















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(tau_M_ref, psi_R)

      
      Compute the stator current reference.

      :param tau_M_ref: Torque reference (Nm).
      :type tau_M_ref: float
      :param psi_R: Estimated rotor flux magnitude (Vs).
      :type psi_R: float

      :returns: Stator current reference (A) and limited torque reference (Nm).
      :rtype: tuple[complex, float]















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, u_s_ref, u_dc)

      
      Field-weakening based on the unlimited reference voltage.

      :param T_s: Sampling time (s).
      :type T_s: float
      :param u_s_ref: Realized (limited) stator voltage reference (V).
      :type u_s_ref: complex
      :param u_dc: DC-link voltage (V).
      :type u_dc: float















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentVectorController(par, cfg, sensorless = True, T_s = 0.000125)

   
   Current-vector controller for induction machine drives.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param cfg: Current-vector controller configuration.
   :type cfg: CurrentVectorControllerCfg
   :param sensorless: If True, sensorless control is used, defaults to True.
   :type sensorless: bool, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(tau_M_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(u_s_ab, i_s_ab)

      
      Get the feedback signals with motion sensors.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_sensored_feedback(u_s_ab, i_s_ab, w_M, theta_M)

      
      Get the feedback signals with motion sensors.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update states.
















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentVectorControllerCfg

   
   Current vector controller configuration.

   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param alpha_c: Current control reference-tracking bandwidth (rad/s), defaults to 2*pi*200.
   :type alpha_c: float, optional
   :param alpha_i: Current control integral-action bandwidth (rad/s), defaults to `alpha_c`.
   :type alpha_i: float, optional
   :param alpha_o: Speed estimation poles (rad/s). Defaults to 2*pi*60 if `J` is None, otherwise
                   2*pi*30, keeping the default speed observer gain the same.
   :type alpha_o: float, optional
   :param k_o: Observer gain as a function of the rotor angular speed.
   :type k_o: Callable[[float], complex], optional
   :param w_s_nom: Nominal stator angular frequency (rad/s), defaults to 2*pi*50.
   :type w_s_nom: float, optional
   :param k_u: Voltage utilization factor, defaults to 0.95.
   :type k_u: float, optional
   :param k_fw: Field-weakening gain (1/H), defaults to `2*R_R/(w_s_nom*L_sgm**2)`.
   :type k_fw: float, optional
   :param J: Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
             used in speed estimation.
   :type J: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: FluxObserver(par, k_o1, k_o2)

   
   Reduced-order flux observer.

   This class implements a reduced-order flux observer for induction machines. The
   observer structure is similar to [#Hin2010]_. The observer operates in synchronous
   coordinates rotating at `w_c` (but not locked to any particular vector). The main-
   flux saturation can be taken into account by providing the saturation model via
   `InductionMachinePars`.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param k_o1: Observer gains as functions of the electrical angular speed of the rotor.
   :type k_o1: Callable[[float], complex]
   :param k_o2: Observer gains as functions of the electrical angular speed of the rotor.
   :type k_o2: Callable[[float], complex]

   .. rubric:: Notes

   The pure voltage model corresponds to ``k_o1 = lambda w_m: 0`` and `k_o2 = lambda
   w_m: 0``, resulting in the marginally stable estimation-error dynamics. The current
   model is obtained by setting ``k_o1 = lambda w_m: 1`` and `k_o2 = lambda w_m: 0``.

   .. rubric:: References

   .. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers with
      stator-resistance adaptation for speed-sensorless induction motor drives," IEEE
      Trans. Power Electron., 2010, https://doi.org/10.1109/TPEL.2009.2039650















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(u_s_ab, i_s_ab, w_M)

      
      Compute the feedback signals for the control system.

      :param u_s_ab: Stator voltage (V) in stator coordinates.
      :type u_s_ab: complex
      :param i_s_ab: Stator current (A) in stator coordinates.
      :type i_s_ab: complex
      :param w_M: Rotor speed (mechanical rad/s), either measured or estimated.
      :type w_M: float, optional

      :returns: **out** -- Estimated feedback signals for the control system.
      :rtype: ObserverOutputs















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, out)

      
      Update the state estimates.
















      ..
          !! processed by numpydoc !!


.. py:class:: FluxVectorController(par, cfg, sensorless = True, T_s = 0.000125)

   
   Flux-vector controller for induction machine drives.

   This class implements a variant of flux-vector control. Decoupling between the
   stator flux and torque channels is used [#Tii2025b]_.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param cfg: Flux-vector control configuration.
   :type cfg: FluxVectorControllerCfg
   :param sensorless: If True, sensorless control is used, defaults to True.
   :type sensorless: bool, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional

   .. rubric:: References

   .. [#Tii2025b] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control
      framework: An extension for induction machines," IEEE Trans. Ind. Electron.,
      2025, https://doi.org/10.1109/TIE.2025.3559958















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(tau_M_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(u_s_ab, i_s_ab)

      
      Get feedback signals without motion sensors.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_sensored_feedback(u_s_ab, i_s_ab, w_M, theta_M)

      
      Get the feedback signals with motion sensors.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update states.
















      ..
          !! processed by numpydoc !!


.. py:class:: FluxVectorControllerCfg

   
   Flux-vector controller configuration.

   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param alpha_tau: Torque-control bandwidth (rad/s), defaults to 2*pi*100.
   :type alpha_tau: float, optional
   :param alpha_psi: Flux-control bandwidth (rad/s), defaults to `alpha_tau`.
   :type alpha_psi: float, optional
   :param alpha_i: Integral action bandwidth (rad/s), defaults to `alpha_tau`.
   :type alpha_i: float, optional
   :param alpha_o: Speed estimation poles (rad/s). Defaults to 2*pi*60 if `J` is None, otherwise
                   2*pi*30, keeping the default speed observer gain the same.
   :type alpha_o: float, optional
   :param k_o: Observer gain as a function of the rotor angular speed.
   :type k_o: Callable[[float], complex], optional
   :param alpha_c: Transparent current-control bandwidth (rad/s), defaults to `alpha_tau`.
   :type alpha_c: float, optional
   :param tau_M_max: Maximum torque reference (Nm).
   :type tau_M_max: float
   :param k_u: Voltage utilization factor, defaults to 0.9.
   :type k_u: float, optional
   :param k_b: Breakdown torque margin, defaults to 0.9.
   :type k_b: float, optional
   :param J: Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
             used in speed estimation.
   :type J: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: InductionMachineInvGammaPars

   
   Constant inverse-Γ model parameters of an induction machine.

   This contains constant inverse-Γ model parameters of an induction machine. To model
   the main-flux saturation, use the `InductionMachinePars` class instead.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_R: Rotor resistance (Ω).
   :type R_R: float
   :param L_sgm: Leakage inductance (H).
   :type L_sgm: float
   :param L_M: Magnetizing inductance (H).
   :type L_M: float

   .. attribute:: R_sgm

      Inverse-Γ total resistance `R_s` plus `R_R` (Ω).

      :type: float

   .. attribute:: alpha

      Inverse rotor time constant (rad/s).

      :type: float

   .. attribute:: w_rb

      Breakdown slip angular frequency (rad/s).

      :type: float















   ..
       !! processed by numpydoc !!

   .. py:method:: from_gamma_pars(par)
      :classmethod:


      
      Compute inverse-Γ model parameters from Γ model parameters.

      This transformation assumes that the parameters are constant.

      :param par: Γ-model parameters.
      :type par: InductionMachinePars

      :returns: Inverse-Γ model parameters.
      :rtype: InductionMachineInvGammaPars















      ..
          !! processed by numpydoc !!


   .. py:method:: update_psi_s(psi_s)

      
      Update the stator flux linkage magnitude state.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedVHzController(par, cfg, T_s = 0.00025)

   
   Observer-based V/Hz controller for induction machine drives.

   This class implements sensorless observer-based V/Hz control. Decoupling between
   the stator flux and torque channels is used [#Tii2025b]_.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param cfg: Observer-based V/Hz controller configuration.
   :type cfg: ObserverBasedVHzControllerCfg
   :param T_s: Sampling period (s), defaults to 250e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(u_s_ab, i_s_ab, w_M_ref)

      
      Get feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process(ts)

      
      Post-process controller time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update states.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedVHzControllerCfg

   
   Observer-based V/Hz controller configuration.

   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param alpha_psi: Flux-control bandwidth (rad/s), defaults to 2*pi*100.
   :type alpha_psi: float, optional
   :param alpha_tau: Torque-control bandwidth (rad/s), defaults to 2*pi*20.
   :type alpha_tau: float, optional
   :param alpha_f: Low-pass-filter bandwidth (rad/s), defaults to 2*pi*1.
   :type alpha_f: float, optional
   :param k_o: Observer gain as a function of the rotor angular speed.
   :type k_o: Callable[[float], complex], optional
   :param k_u: Voltage utilization factor, defaults to 0.9.
   :type k_u: float, optional
   :param k_b: Breakdown torque margin, defaults to 0.9.
   :type k_b: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: PIController(k_p, k_i, k_t = None, u_max = inf)

   
   2DOF PI controller.

   This implements a discrete-time 2DOF PI controller, whose continuous-time
   counterpart is::

       u = k_t*y_ref - k_p*y + (k_i/s)*(y_ref - y) + u_ff

   where `u` is the controller output, `y_ref` is the reference signal, `y` is the
   feedback signal, `u_ff` is the feedforward signal, and `1/s` refers to integration.
   The standard PI controller is obtained by choosing ``k_t = k_p``. The integrator
   anti-windup is implemented based on the realized controller output.

   :param k_p: Proportional gain.
   :type k_p: float
   :param k_i: Integral gain.
   :type k_i: float
   :param k_t: Reference-feedforward gain, defaults to `k_p`.
   :type k_t: float, optional
   :param u_max: Maximum controller output, defaults to `inf`.
   :type u_max: float, optional

   .. rubric:: Notes

   This controller can be used, e.g., as a speed controller. In this case, `y`
   corresponds to the rotor angular speed `w_M` and `u` to the torque reference
   `tau_M_ref`.















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(y_ref, y, u_ff = 0.0)

      
      Compute the controller output.

      :param y_ref: Reference signal.
      :type y_ref: float
      :param y: Feedback signal.
      :type y: float
      :param u_ff: Feedforward signal, defaults to 0.
      :type u_ff: float, optional

      :returns: **u** -- Controller output.
      :rtype: float















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, u)

      
      Update the integral state.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u: Realized (limited) controller output.
      :type u: float















      ..
          !! processed by numpydoc !!


.. py:class:: ReferenceGenerator(par, psi_s_nom, i_s_max, tau_M_max = inf, k_u = 1.0, k_b = 1.0)

   
   Reference generator for flux-vector control.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param tau_M_max: Maximum torque reference (Nm), defaults to `inf`.
   :type tau_M_max: float, optional
   :param k_u: Voltage utilization factor, defaults to 1.
   :type k_u: float, optional
   :param k_b: Breakdown torque margin, defaults to 1.
   :type k_b: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(tau_M_ref, w_s, psi_R, u_dc)

      
      Simple field-weakening strategy.
















      ..
          !! processed by numpydoc !!


.. py:class:: SpeedController(J, alpha_s, alpha_i = None, tau_M_max = inf)

   Bases: :py:obj:`motulator.common.control._controllers.PIController`


   
   2DOF PI speed controller.

   This is an interface for a speed controller. The gains are initialized based on the
   desired closed-loop bandwidth and the rotor inertia estimate.

   :param J: Total inertia of the rotor (kgm²).
   :type J: float
   :param alpha_s: Reference-tracking bandwidth (rad/s).
   :type alpha_s: float
   :param alpha_i: Integral action bandwidth (rad/s), defaults to `alpha_s`.
   :type alpha_i: float, optional
   :param tau_M_max: Maximum motor torque (Nm), defaults to `inf`.
   :type tau_M_max: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: SpeedFluxObserver(par, k_o1, k_o2, alpha_o, J = None)

   Bases: :py:obj:`FluxObserver`


   
   Observer with speed estimation.

   This class implements a reduced-order flux observer for induction machines with
   speed estimation. If the inertia of the mechanical system is provided, the observer
   also estimates the load torque, to avoid the lag in the speed estimate.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param k_o1: Observer gains as functions of the electrical angular speed of the rotor.
   :type k_o1: Callable[[float], complex]
   :param k_o2: Observer gains as functions of the electrical angular speed of the rotor.
   :type k_o2: Callable[[float], complex]
   :param alpha_o: Speed estimation pole (rad/s).
   :type alpha_o: float
   :param J: Inertia of the mechanical system (kgm²). Defaults to None, which means the
             mechanical system model is not used.
   :type J: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(u_s_ab, i_s_ab, w_M=None)

      
      Compute feedback signals with speed estimation.

      :param u_s_ab: Stator voltage (V) in stator coordinates.
      :type u_s_ab: complex
      :param i_s_ab: Stator current (A) in stator coordinates.
      :type i_s_ab: complex

      :returns: **out** -- Estimated feedback signals for the control system, including speed estimate.
      :rtype: ObserverOutputs















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, out)

      
      Extend the update method to include the speed estimate.
















      ..
          !! processed by numpydoc !!


.. py:class:: VHzControlSystem(vhz_ctrl, slew_rate = inf)

   Bases: :py:obj:`motulator.common.control._base.ControlSystem`


   
   V/Hz control system.

   :param vhz_ctrl: V/Hz controller to be used in the drive control system.
   :type vhz_ctrl: VHzController
   :param slew_rate: Slew rate (mechanical rad/s**2) for the speed reference, defaults to `inf`.
   :type slew_rate: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute controller output based on feedback.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_measurement(mdl)

      
      Get measurements.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Extend the post-process method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_speed_ref(ref_fcn)

      
      Set the external speed reference.

      :param ref_fcn: Speed reference (mechanical rad/s) as a function of time.
      :type ref_fcn: Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update controller states.
















      ..
          !! processed by numpydoc !!


.. py:class:: VectorControlSystem(vector_ctrl, speed_ctrl = None)

   Bases: :py:obj:`motulator.common.control._base.ControlSystem`


   
   Vector control system.

   This class defines the interface for drive control systems. It is a generic class
   that can be used with different inner controllers (such as current-vector control
   and flux-vector control).

   :param vector_ctrl: Vector controller whose input is the torque reference.
   :type vector_ctrl: VectorController
   :param speed_ctrl: Speed controller. If not given or None, torque-control mode is used.
   :type speed_ctrl: SpeedController | PIController | None















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute controller output based on feedback.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_measurement(mdl)

      
      Get measurements from sensors.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Extend the post-process method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_speed_ref(ref_fcn)

      
      Set the external speed reference for speed-control mode.

      :param ref_fcn: Speed reference (mechanical rad/s) as a function of time.
      :type ref_fcn: Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: set_torque_ref(ref_fcn)

      
      Set the external torque reference for torque-control mode.

      :param ref_fcn: Torque reference (Nm) as a function of time.
      :type ref_fcn: Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update controller states.
















      ..
          !! processed by numpydoc !!


