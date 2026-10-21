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
   motulator.drive.control.im.DiscreteCurrentController
   motulator.drive.control.im.DriveControlSystem
   motulator.drive.control.im.FluxObserver
   motulator.drive.control.im.FluxVectorController
   motulator.drive.control.im.FluxVectorControllerCfg
   motulator.drive.control.im.InductionMachineInvGammaPars
   motulator.drive.control.im.InductionMachinePars
   motulator.drive.control.im.InverterNonlinearity
   motulator.drive.control.im.ObserverBasedVHzController
   motulator.drive.control.im.ObserverBasedVHzControllerCfg
   motulator.drive.control.im.ObserverOutputs
   motulator.drive.control.im.PIController
   motulator.drive.control.im.PWM
   motulator.drive.control.im.ReferenceGenerator
   motulator.drive.control.im.SpeedController
   motulator.drive.control.im.SpeedFluxObserver
   motulator.drive.control.im.SpeedObserver
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
   :type alpha_c: float
   :param alpha_i: Integral action bandwidth (rad/s), defaults to `alpha_c`.
   :type alpha_i: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentReferenceGenerator(par, psi_s_nom, i_s_max, w_s_nom, k_u, k_fw)

   
   Current reference generator.

   In the base-speed region, the current reference in rotor-flux coordinates is::

       i_s_ref = i_sd_nom + 1j*tau_M_ref/(1.5*n_p*abs(psi_R))

   where `psi_R` is the estimated rotor flux. The nominal flux-producing current
   component is computed from the nominal stator flux in the no-load condition::

       i_sd_nom = psi_s_nom/(L_M + L_sgm)

   If the saturation is modeled, the inductances are evaluated at `psi_s_nom`. In the
   field-weakening operation, the flux-producing current component is::

       i_s_ref.real = (k_fw/s)*(u_s_max - abs(u_s_ref))

   where `1/s` refers to integration, ``u_s_max = k_u*u_dc/sqrt(3)`` is the maximum
   stator voltage in the linear modulation region, `u_s_ref` is the (unlimited) stator
   voltage reference, and `k_fw` is the field-weakening gain. The field-weakening
   method and its tuning corresponds roughly to [#Hin2006]_. Furthermore, the torque-
   producing current component `i_s_ref.imag` is limited based on the maximum stator
   current and the breakdown slip.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param w_s_nom: Nominal stator angular frequency (rad/s).
   :type w_s_nom: float
   :param k_u: Voltage utilization factor.
   :type k_u: float
   :param k_fw: Field-weakening gain (1/H). If None, ``2*R_R/(w_s_nom*L_sgm**2)`` is used.
   :type k_fw: float | None

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
      :param u_s_ref: Unlimited stator voltage reference (V).
      :type u_s_ref: complex
      :param u_dc: DC-link voltage (V).
      :type u_dc: float















      ..
          !! processed by numpydoc !!


.. py:class:: CurrentVectorController(par, cfg)

   
   Current-vector controller for induction machine drives.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param cfg: Current-vector controller configuration.
   :type cfg: CurrentVectorControllerCfg















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(tau_M_ref, fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(u_s_ab, i_s_ab, w_M_meas, theta_M_meas)

      
      Get the feedback signals.
















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
   :param alpha_i: Current control integral-action bandwidth (rad/s), defaults to `alpha_c`. Not
                   used if `discrete` is True.
   :type alpha_i: float, optional
   :param alpha_o: Speed estimation poles (rad/s). Defaults to 2*pi*60 if `sensorless` is True,
                   otherwise 2*pi*400. If `J` is given, the default is halved, which keeps the
                   speed estimation gain approximately the same.
   :type alpha_o: float | None, optional
   :param k_o: Observer gain as a function of the electrical rotor angular speed. If None,
               defaults to ``lambda w_m: (0.5*alpha + 0.2*abs(w_m))/(alpha - 1j*w_m)`` if
               `sensorless` is True, otherwise ``lambda w_m: 1 + 0.2*abs(w_m)/(alpha -
               1j*w_m)``, where ``alpha = R_R/L_M``.
   :type k_o: Callable[[float], complex] | None, optional
   :param w_s_nom: Nominal stator angular frequency (rad/s), defaults to 2*pi*50.
   :type w_s_nom: float, optional
   :param k_u: Voltage utilization factor, defaults to 0.95.
   :type k_u: float, optional
   :param k_fw: Field-weakening gain (1/H), defaults to `2*R_R/(w_s_nom*L_sgm**2)`.
   :type k_fw: float | None, optional
   :param J: Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
             used in speed estimation.
   :type J: float | None, optional
   :param sensorless: If True, sensorless control is used, defaults to True.
   :type sensorless: bool, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional
   :param discrete: If True, the direct discrete-time current controller is used instead of the
                    continuous-time design, defaults to False.
   :type discrete: bool, optional















   ..
       !! processed by numpydoc !!

.. py:class:: DiscreteCurrentController(par, alpha_c, T_s)

   Bases: :py:obj:`motulator.common.control.DiscreteComplexController`


   
   Direct discrete-time current controller for induction machines.

   The currents are mapped to the leakage flux linkages, ``psi = L_sgm*i``. The
   resistive voltage drop and the back-emf are disturbances, which the integral action
   compensates for. Since the design includes the delays, the angle of the voltage
   reference is not advanced (`k_comp = 0` in `CurrentVectorController`).

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param alpha_c: Reference-tracking bandwidth (rad/s).
   :type alpha_c: float
   :param T_s: Sampling period (s).
   :type T_s: float















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(i_ref, i)

      
      Compute the controller output.

      :param i_ref: Reference signal.
      :type i_ref: complex
      :param i: Feedback signal.
      :type i: complex

      :returns: **u** -- Controller output.
      :rtype: complex















      ..
          !! processed by numpydoc !!


.. py:class:: DriveControlSystem(pwm, k_comp)

   Bases: :py:obj:`motulator.common.control._base.ControlSystem`


   
   Base class for drive control systems.

   This class implements the parts common to the drive control systems: the feedback
   voltages, the modulation, and the speed reference. Subclasses implement
   `get_measurement`, `update`, `post_process`, and the hooks `_get_inner_feedback`,
   `_compute_inner_output`, and `_compute_inner_standby_output`.

   The inner controller gets the realized voltage of the ongoing sampling period for
   its observer, while the feedback voltage `u_s` for the control laws is the realized
   voltage averaged over the previous and the ongoing sampling periods. Before the
   modulation, the angle of the voltage reference is advanced by `k_comp*T_s*w_c`,
   where `k_comp` is given by the inner controller. The value 1.5 compensates for the
   computational delay of one sampling period and the delay of half a sampling period
   caused by the zero-order hold of the PWM [#Bae2003]_. The value 0 means that the
   angle is not advanced, which is used if the controller takes the delays into
   account itself, e.g., in a direct discrete-time design.

   While the gates are blocked (see `set_enable`), the control system is in standby:
   the observers run on their open-circuit model, since the converter voltage is
   unknown, the inner controller computes its output for zero torque, and the speed
   controller is not updated.

   :param pwm: Pulse-width modulator.
   :type pwm: PWM
   :param k_comp: Angle-advance factor of the voltage reference.
   :type k_comp: float

   .. rubric:: References

   .. [#Bae2003] Bae, Sul, "A compensation method for time delay of full-digital
      synchronous frame current regulator of PWM AC drives," IEEE Trans. Ind. Appl.,
      2003, https://doi.org/10.1109/TIA.2003.810660















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute controller output based on feedback.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_standby_output(fbk)

      
      Compute controller output while the gates are blocked.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback(meas)

      
      Get feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: modulate(T_s, u_s_ref, fbk)

      
      Compute the duty ratios from the stator voltage reference.

      The reference is in the controller coordinates. Its angle is advanced by
      `k_comp*T_s*w_c`. Subclasses can override this method, e.g., to control an
      output filter between the converter and the machine.















      ..
          !! processed by numpydoc !!


   .. py:method:: set_speed_ref(ref_fcn)

      
      Set the external speed reference.

      :param ref_fcn: Speed reference (mechanical rad/s), constant or a function of time.
      :type ref_fcn: float | Callable[[float], float]















      ..
          !! processed by numpydoc !!


.. py:class:: FluxObserver(par, k_o)

   
   Reduced-order flux observer.

   This class implements a reduced-order flux observer for induction machines. The
   observer structure is similar to [#Hin2010]_. The observer operates in synchronous
   coordinates rotating at `w_c` (but not locked to any particular vector). The rotor
   speed error signal is either computed internally from the flux estimation error or
   supplied externally, e.g., from the speed measurement. The weight `h` of the
   external signal, given to `compute_output`, selects between these: `h = 0` gives
   purely model-based (sensorless) operation, `h = 1` relies on the external signal
   alone, and intermediate values blend the two. The conjugate-error observer gain
   follows the same weight. The main-flux saturation can be taken into account by
   providing the saturation model via `InductionMachinePars`.

   The flux estimate is corrected and predicted using a hold-equivalent discrete-time
   model, and the estimation error is the difference between the rotor flux
   predictions of the current model and the voltage model, cf. [#Bot1993]_,
   [#Hin2018]_.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param k_o: Observer gain as a function of the electrical angular speed of the rotor.
   :type k_o: Callable[[float], complex]

   .. rubric:: Notes

   The pure voltage model corresponds to ``k_o = lambda w_m: 0``, resulting in the
   marginally stable estimation-error dynamics. The current model is obtained by
   setting ``k_o = lambda w_m: 1`` together with `h = 1`.

   .. rubric:: References

   .. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers with
      stator-resistance adaptation for speed-sensorless induction motor drives," IEEE
      Trans. Power Electron., 2010, https://doi.org/10.1109/TPEL.2009.2039650

   .. [#Bot1993] Bottura, Silvino, de Resende, "A flux observer for induction machines
      based on a time-variant discrete model," IEEE Trans. Ind. Appl., 1993,
      https://doi.org/10.1109/28.216543

   .. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
      sensorless synchronous motor drives: Framework for design and analysis," IEEE
      Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(u_s_ab, i_s_ab, w_M, eps_ext = 0.0, h = 0.0)

      
      Compute the feedback signals for the control system.

      :param u_s_ab: Stator voltage (V) in stator coordinates, averaged over the ongoing
                     sampling period, i.e., from this sampling instant to the next one. If
                     None (unknown voltage, e.g., with blocked gates), the flux estimate
                     follows the current model alone.
      :type u_s_ab: complex | None
      :param i_s_ab: Stator current (A) in stator coordinates.
      :type i_s_ab: complex
      :param w_M: Rotor speed (mechanical rad/s), typically from the speed observer.
      :type w_M: float
      :param eps_ext: External rotor speed error signal (mechanical rad/s), defaults to 0.
      :type eps_ext: float, optional
      :param h: Weight of `eps_ext` in the range [0, 1], defaults to 0, i.e., the model-
                based error signal is used exclusively.
      :type h: float, optional

      :returns: **out** -- Estimated feedback signals for the control system.
      :rtype: ObserverOutputs















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, out)

      
      Update the state estimates.
















      ..
          !! processed by numpydoc !!


.. py:class:: FluxVectorController(par, cfg)

   
   Flux-vector controller for induction machine drives.

   This class implements a variant of flux-vector control. Decoupling between the
   stator flux and torque channels is used [#Tii2025b]_.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param cfg: Flux-vector control configuration.
   :type cfg: FluxVectorControllerCfg

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


   .. py:method:: get_feedback(u_s_ab, i_s_ab, w_M_meas, theta_M_meas)

      
      Get the feedback signals.
















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
   :param alpha_o: Speed estimation poles (rad/s). Defaults to 2*pi*60 if `sensorless` is True,
                   otherwise 2*pi*400. If `J` is given, the default is halved, which keeps the
                   speed estimation gain approximately the same.
   :type alpha_o: float | None, optional
   :param k_o: Observer gain as a function of the electrical rotor angular speed. If None,
               defaults to ``lambda w_m: (0.5*alpha + 0.2*abs(w_m))/(alpha - 1j*w_m)`` if
               `sensorless` is True, otherwise ``lambda w_m: 1 + 0.2*abs(w_m)/(alpha -
               1j*w_m)``, where ``alpha = R_R/L_M``.
   :type k_o: Callable[[float], complex] | None, optional
   :param tau_M_max: Maximum torque reference (Nm), defaults to `inf`.
   :type tau_M_max: float, optional
   :param k_u: Voltage utilization factor, defaults to 0.9.
   :type k_u: float, optional
   :param k_b: Breakdown torque margin, defaults to 0.9.
   :type k_b: float, optional
   :param J: Inertia (kgm²). Defaults to None, meaning the mechanical system model is not
             used in speed estimation.
   :type J: float, optional
   :param sensorless: If True, sensorless control is used, defaults to True.
   :type sensorless: bool, optional
   :param T_s: Sampling period (s), defaults to 125e-6.
   :type T_s: float, optional















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
   :param G_c: Core-loss conductance (S), defaults to 0 (no core losses). It is connected
               across the stator EMF `u_s - R_s*i_s`, i.e., between the stator resistance and
               the leakage inductance, which corresponds to the core-loss branch in parallel
               with the magnetizing branch of the Γ model.
   :type G_c: float, optional

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


.. py:class:: InductionMachinePars

   
   Γ-model parameters of an induction machine.

   This contains Γ-model parameters of an induction machine. The main-flux saturation
   can also be modeled by providing a callable `L_s` parameter. For convenience, the
   class also provides the corresponding inverse-Γ model parameters, which can be used
   in control systems. If the saturation is modeled, these inverse-Γ parameters depend
   on the stator flux linkage magnitude `psi_s` that should be updated by calling the
   `update_psi_s` method.

   :param n_p: Number of pole pairs.
   :type n_p: int
   :param R_s: Stator resistance (Ω).
   :type R_s: float
   :param R_r: Rotor resistance (Ω).
   :type R_r: float
   :param L_ell: Leakage inductance (H).
   :type L_ell: float
   :param L_s: Stator inductance (H). If callable, it should be a function of the stator flux
               linkage magnitude (Vs).
   :type L_s: float | Callable[[float], float]
   :param G_c: Core-loss conductance (S), modeled in parallel with the magnetizing branch,
               defaults to 0 (no core losses).
   :type G_c: float, optional
   :param psi_s: Stator flux linkage magnitude (Vs), at which the callable `L_s` is evaluated,
                 defaults to 0. It can be updated by calling the `update_psi_s` method.
   :type psi_s: float, optional

   .. attribute:: gamma

      Magnetic coupling factor.

      :type: float

   .. attribute:: L_M

      Inverse-Γ magnetizing inductance (H).

      :type: float

   .. attribute:: L_sgm

      Inverse-Γ leakage inductance (H).

      :type: float

   .. attribute:: R_R

      Inverse-Γ rotor resistance (Ω).

      :type: float

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

   .. py:method:: from_inv_gamma_pars(par)
      :classmethod:


      
      Compute Γ-model parameters from inverse-Γ model parameters.

      This transformation assumes that the parameters are constant.

      :param par: Inverse-Γ model parameters.
      :type par: InductionMachineInvGammaPars

      :returns: Γ model parameters.
      :rtype: InductionMachinePars















      ..
          !! processed by numpydoc !!


   .. py:method:: update_psi_s(psi_s)

      
      Update the stator flux linkage magnitude state.
















      ..
          !! processed by numpydoc !!


.. py:class:: InverterNonlinearity(t_d = 0.0, u_th = 0.0, sign = np.sign)

   
   Duty-ratio error due to the dead time and the threshold voltages.

   The dead time blanks a leg for `t_d` after each switching instant, and the current
   direction determines the leg state during the blanking. Away from the duty-ratio
   limits, the resulting error is `sign(i)*t_d/(2*T_half)`, where `T_half` is the
   half carrier period. Pulses shorter than the dead time are suppressed, and clamped
   legs have no dead-time error. The threshold voltage `u_th` of the power devices,
   assumed equal for the switches and the diodes, causes the voltage drop
   `sign(i)*u_th` in each phase, i.e., the duty-ratio error `sign(i)*u_th/u_dc`,
   also in the clamped legs. The realized duty ratios are `d_abc - d_err`. This model
   is consistent with `motulator.common.model.VoltageSourceConverter`.

   :param t_d: Dead time (s), defaults to 0.
   :type t_d: float, optional
   :param u_th: Threshold voltage (V) of the power devices, defaults to 0.
   :type u_th: float, optional
   :param sign: Current-direction function, defaults to `np.sign`. A smooth approximation may
                be used, matching the converter model.
   :type sign: Callable[[np.ndarray], np.ndarray], optional

   .. rubric:: Notes

   This switching-cycle averaged model assumes constant duty ratios and currents
   over a carrier period. It does not include transients due to changes in the
   switching commands.















   ..
       !! processed by numpydoc !!

.. py:class:: ObserverBasedVHzController(par, cfg)

   
   Observer-based V/Hz controller for induction machine drives.

   This class implements sensorless observer-based V/Hz control. Decoupling between
   the stator flux and torque channels is used [#Tii2025b]_.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param cfg: Observer-based V/Hz controller configuration.
   :type cfg: ObserverBasedVHzControllerCfg















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute references.
















      ..
          !! processed by numpydoc !!


   .. py:method:: compute_standby_output(fbk)

      
      Compute references while the gates are blocked.
















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
   :param k_o: Observer gain as a function of the electrical rotor angular speed. If None,
               defaults to ``lambda w_m: (0.5*alpha + 0.2*abs(w_m))/(alpha - 1j*w_m)``, where
               ``alpha = R_R/L_M``.
   :type k_o: Callable[[float], complex] | None, optional
   :param k_u: Voltage utilization factor, defaults to 0.9.
   :type k_u: float, optional
   :param k_b: Breakdown torque margin, defaults to 0.9.
   :type k_b: float, optional
   :param T_s: Sampling period (s), defaults to 250e-6.
   :type T_s: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: ObserverOutputs

   
   Feedback signals for the control system.
















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


.. py:class:: PWM(overmodulation = 'MPE', d_err = None, feedforward = True, k_pred = 1.5, d_min = 0.0, single_update = False)

   
   Duty ratios and realized voltage for three-phase space-vector PWM.

   This computes the duty ratios corresponding to standard space-vector PWM and
   overmodulation [#Hav1999]_. The realized voltage is reconstructed from the duty
   ratios and the measured DC-bus voltage, taking into account the computational delay
   of one sampling period. The angle of the voltage reference is not advanced here:
   the control systems compensate for the delays, if needed by the controller.

   Optionally, the duty-ratio error caused by the inverter nonlinearities (such as the
   dead time and the threshold voltages of the power devices) is modeled as a function
   of the phase currents, duty ratios, DC-bus voltage, and half carrier period. The
   realized voltage is corrected for this error using the measured currents, which
   correspond to the same instant as the realized voltage, and the duty ratios of the
   corresponding sampling periods.
   Furthermore, the error can be compensated for by feedforward, in which case the
   currents are predicted to the middle of the sampling period in which the duty ratios
   are applied, assuming that they rotate at the angular speed of the synchronous
   coordinates. Near the duty-ratio limits, the feedforward may not fully cancel a
   duty-dependent error.

   Optionally, the pulses are limited to a minimum width, e.g., due to the gate
   drivers [#Wel2006]_: the duty ratio of a switching leg is in the range
   `[d_min, 1 - d_min]`, while the duty ratios 0 and 1 (a leg not switching) are not
   limited. A duty ratio in the range `(0, d_min)` is rounded to 0 (the pulse is
   dropped) or to `d_min`, and in the range `(1 - d_min, 1)` to `1 - d_min` or 1,
   whichever gives the realized duty ratio nearer to the duty ratio of the voltage
   reference. If the duty-ratio error is compensated for, the realized duty ratios are
   those given by `d_err`, so that the compensation also accounts for the minimum
   pulses. The limited voltage reference and the realized voltage are computed from
   the limited duty ratios.

   :param overmodulation: Overmodulation method, defaults to "MPE". Valid options are:
                          - "MPE": minimum phase error
                          - "MME": minimum magnitude error
                          - "six_step": six-step operation
   :type overmodulation: Literal["MPE", "MME", "six_step"], optional
   :param d_err: Duty-ratio error as a function of the phase currents (A), duty ratios, DC-bus
                 voltage (V), and half carrier period (s), i.e., the realized duty ratios are
                 `d_abc - d_err(i_abc, d_abc, u_dc, T_half)`, defaults to None (no error). See
                 `InverterNonlinearity` for the dead time and the threshold voltages.
   :type d_err: Callable[[np.ndarray, np.ndarray, float, float], np.ndarray], optional
   :param feedforward: Compensate for `d_err` in the duty ratios, defaults to True.
   :type feedforward: bool, optional
   :param k_pred: Prediction factor of the currents for the feedforward compensation, defaults to
                  1.5, which corresponds to the middle of the sampling period in which the duty
                  ratios are applied after the computational delay of one sampling period.
   :type k_pred: float, optional
   :param d_min: Minimum duty ratio of a switching leg, in the range [0, 0.5), defaults to 0
                 (no limit). The duty ratio applies to a half carrier period, so the shortest
                 pulse is `d_min` times the half carrier period when a leg starts or stops
                 switching and about twice that otherwise.
   :type d_min: float, optional
   :param single_update: Duty ratios are updated once per carrier period, i.e., the half carrier period
                         is `T_s/2`, defaults to False (twice per carrier period, the half carrier
                         period is `T_s`). Must match the system model.
   :type single_update: bool, optional

   .. rubric:: References

   .. [#Hav1999] Hava, Sul, Kerkman, Lipo, "Dynamic overmodulation characteristics of
      triangle intersection PWM methods," IEEE Trans. Ind. Appl., 1999,
      https://doi.org/10.1109/28.777199

   .. [#Wel2006] Welchko, Schulz, Hiti, "Effects and compensation of dead-time and
      minimum pulse-width limitations in two-level PWM voltage source inverters,"
      Proc. IEEE IAS Annu. Meeting, 2006, https://doi.org/10.1109/IAS.2006.256630















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(T_s, u_c_ref_ab, u_dc, w)

      
      Compute the duty ratios and the corresponding voltage.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_c_ref_ab: Converter voltage reference (V) in stationary coordinates.
      :type u_c_ref_ab: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float
      :param w: Angular speed of the synchronous coordinates (rad/s), used for predicting
                the currents in the feedforward compensation of `d_err`.
      :type w: float

      :returns: * **d_abc** (*list[float]*) -- Duty ratios for the next sampling period.
                * **u_c_ab** (*complex*) -- Voltage (V) in stationary coordinates corresponding to the duty ratios
                  `d_abc`, i.e., the limited voltage reference with the duty-ratio error
                  compensation (if used) included.















      ..
          !! processed by numpydoc !!


   .. py:method:: duty_ratios(u_c_ref_ab, u_dc)

      
      Compute the duty ratios for three-phase space-vector PWM.

      :param u_c_ref_ab: Converter voltage reference (V) in stationary coordinates.
      :type u_c_ref_ab: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **d_abc** -- Duty ratios.
      :rtype: list[float]















      ..
          !! processed by numpydoc !!


   .. py:method:: get_realized_voltage(i_c_ab, u_dc, *, average = True)

      
      Get the realized voltage.

      This method is to be called at the sampling instant before the next duty ratios
      are computed, and a computational delay of one sampling period is assumed. The
      voltage is reconstructed from the duty ratios of the previous and the ongoing
      sampling periods and the measured DC-bus voltage, i.e., the variations of the
      DC-bus voltage and of the currents within the periods are not known. The
      duty-ratio error is evaluated with the half carrier period of the previous
      sampling period, which is assumed to equal that of the ongoing period. The
      measured currents are also stored for the feedforward compensation of the next
      duty ratios.

      :param i_c_ab: Measured converter current (A) in stationary coordinates.
      :type i_c_ab: complex
      :param u_dc: Measured DC-bus voltage (V).
      :type u_dc: float
      :param average: If True, the average voltage of the previous and the ongoing sampling
                      periods is returned. If False, the voltage of the ongoing sampling period
                      is returned. Defaults to True.
      :type average: bool, optional

      :returns: Realized converter voltage (V) in stationary coordinates. If `d_err` is
                given, the voltage is corrected for it using the measured currents.
      :rtype: complex















      ..
          !! processed by numpydoc !!


   .. py:method:: limit_pulses(d_abc, d_ref, d_err = None)

      
      Limit the duty ratios of the switching legs to `[d_min, 1 - d_min]`.

      A duty ratio in the range `(0, d_min)` is replaced by 0 or `d_min`, and in the
      range `(1 - d_min, 1)` by `1 - d_min` or 1, whichever gives the realized duty
      ratio nearer to the reference (if equally near, the leg switches). The duty
      ratios 0 and 1 are not changed.

      :param d_abc: Duty ratios, including the compensation of the duty-ratio error.
      :type d_abc: list[float]
      :param d_ref: Duty ratios of the voltage reference, i.e., the realized duty ratios aimed
                    at.
      :type d_ref: ndarray, shape (3,)
      :param d_err: Duty-ratio error as a function of the duty ratios, if it is compensated
                    for, in which case the realized duty ratios are `d_abc - d_err(d_abc)`.
                    Defaults to None, in which case the realized duty ratios are `d_abc`.
      :type d_err: Callable[[np.ndarray], np.ndarray], optional

      :returns: * **d_abc** (*list[float]*) -- Limited duty ratios.
                * **d_realized** (*ndarray, shape (3,)*) -- Realized duty ratios: `d_ref` for the legs whose duty ratios were not
                  changed and the realized duty ratios of the limited legs.















      ..
          !! processed by numpydoc !!


   .. py:method:: six_step_overmodulation(u_c_ref_ab, u_dc)
      :staticmethod:


      
      Overmodulation up to six-step operation.

      This method modifies the angle of the voltage reference vector in the
      overmodulation region such that the six-step operation is reached [#Bol1997]_.

      :param u_c_ref_ab: Converter voltage reference (V) in stationary coordinates.
      :type u_c_ref_ab: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **u_c_ref_ab** -- Modified converter voltage reference (V) in stationary coordinates.
      :rtype: complex

      .. rubric:: References

      .. [#Bol1997] Bolognani, Zigliotto, "Novel digital continuous control of SVM
         inverters in the overmodulation range," IEEE Trans. Ind. Appl., 1997,
         https://doi.org/10.1109/28.568019















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, d_abc)

      
      Store the next duty ratios and the ongoing half carrier period.
















      ..
          !! processed by numpydoc !!


.. py:class:: ReferenceGenerator(par, psi_s_nom, i_s_max, tau_M_max, k_u, k_b)

   
   Reference generator for flux-vector control.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param psi_s_nom: Nominal stator flux linkage (Vs).
   :type psi_s_nom: float
   :param i_s_max: Maximum stator current (A).
   :type i_s_max: float
   :param tau_M_max: Maximum torque reference (Nm).
   :type tau_M_max: float
   :param k_u: Voltage utilization factor.
   :type k_u: float
   :param k_b: Breakdown torque margin.
   :type k_b: float















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

.. py:class:: SpeedFluxObserver(par, alpha_o, T_s, k_o, J = None)

   
   Flux observer with speed estimation.

   This class implements a reduced-order flux observer for induction machines with
   speed estimation. If the inertia of the mechanical system is provided, the observer
   also estimates the load torque, to avoid the lag in the speed estimate. The rotor
   speed error signal is defined in :class:`FluxObserver`.

   :param par: Machine model parameters.
   :type par: InductionMachineInvGammaPars | InductionMachinePars
   :param alpha_o: Speed estimation pole (rad/s).
   :type alpha_o: float
   :param T_s: Sampling period (s). The poles of the ideal sensored estimation-error dynamics
               are placed at ``exp(-alpha_o*T_s)``.
   :type T_s: float
   :param k_o: Observer gain as a function of the electrical angular speed of the rotor.
   :type k_o: Callable[[float], complex]
   :param J: Inertia of the mechanical system (kgm²). Defaults to None, which means the
             mechanical system model is not used.
   :type J: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(u_s_ab, i_s_ab, eps_ext = 0.0, h = 0.0)

      
      Compute feedback signals with speed estimation.

      :param u_s_ab: Stator voltage (V) in stator coordinates, or None if unknown, see
                     :class:`FluxObserver`.
      :type u_s_ab: complex | None
      :param i_s_ab: Stator current (A) in stator coordinates.
      :type i_s_ab: complex
      :param eps_ext: External rotor speed error signal (mechanical rad/s), defaults to 0.
      :type eps_ext: float, optional
      :param h: Weight of `eps_ext` in the range [0, 1], defaults to 0, i.e., the model-
                based error signal is used exclusively.
      :type h: float, optional

      :returns: **out** -- Estimated feedback signals for the control system.
      :rtype: ObserverOutputs















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, out)

      
      Update state observers.
















      ..
          !! processed by numpydoc !!


   .. py:property:: w_M
      :type: float

      
      Mechanical angular rotor speed estimate (rad/s), also an initial value.
















      ..
          !! processed by numpydoc !!


.. py:class:: SpeedObserver(k_w, k_tau, J)

   
   Speed observer.

   This observer estimates the mechanical rotor speed based on the mechanical system
   model and the error signal. If the inertia of the mechanical system is provided, the
   load torque is also estimated, avoiding lag in the speed estimate [#Lor1991]_.

   :param k_w: Speed-estimation gain (rad/s or (rad/s)² for induction machines or for
               synchronous machines, respectively).
   :type k_w: float
   :param k_tau: Load-torque estimation gain (Nm or Nm/s for induction machines or for
                 synchronous machines, respectively).
   :type k_tau: float
   :param J: Inertia of the mechanical system (kgm²). If None, the load torque estimation is
             not used.
   :type J: float | None

   .. rubric:: References

   .. [#Lor1991] Lorenz, Van Patten, "High-resolution velocity estimation for
      all-digital, AC servo drives," IEEE Trans. Ind. Appl., 1991,
      https://doi.org/10.1109/28.85485















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output()

      
      Compute outputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, eps, tau_M = 0.0)

      
      Update mechanical state estimates.

      :param T_s: Sample time (s).
      :type T_s: float
      :param eps: Estimation error signal: mechanical speed (rad/s) or mechanical position
                  (rad) for induction machines or synchronous machines, respectively.
      :type eps: float
      :param tau_M: Electromagnetic torque estimate (Nm).
      :type tau_M: float, optional















      ..
          !! processed by numpydoc !!


.. py:class:: VHzControlSystem(vhz_ctrl, slew_rate = inf, pwm = None)

   Bases: :py:obj:`DriveControlSystem`


   
   V/Hz control system.

   :param vhz_ctrl: V/Hz controller to be used in the drive control system.
   :type vhz_ctrl: VHzController
   :param slew_rate: Slew rate (mechanical rad/s**2) for the speed reference, defaults to `inf`.
   :type slew_rate: float, optional
   :param pwm: Pulse-width modulator, defaults to `PWM(overmodulation="MME")`.
   :type pwm: PWM, optional















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


   .. py:method:: update(ref, fbk)

      
      Update controller states.
















      ..
          !! processed by numpydoc !!


.. py:class:: VectorControlSystem(vector_ctrl, speed_ctrl = None, pwm = None)

   Bases: :py:obj:`DriveControlSystem`


   
   Vector control system.

   This class defines the interface for drive control systems. It is a generic class
   that can be used with different inner controllers (such as current-vector control
   and flux-vector control).

   A sensorless drive stays in standby with the zero voltage vector also at the first
   sampling instant after the release of the gates. Due to the computational delay,
   the first two sampling periods are then zero-voltage periods, and the observer of
   a synchronous machine catches the rotating machine from the current response to
   the first one (a flying start).

   :param vector_ctrl: Vector controller whose input is the torque reference.
   :type vector_ctrl: VectorController
   :param speed_ctrl: Speed controller. If None (default), torque-control mode is used.
   :type speed_ctrl: SpeedController | PIController | None, optional
   :param pwm: Pulse-width modulator, defaults to `PWM()`.
   :type pwm: PWM, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_output(fbk)

      
      Compute controller output based on feedback.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_measurement(mdl)

      
      Get measurements from sensors.
















      ..
          !! processed by numpydoc !!


   .. py:method:: modulate(T_s, u_s_ref, fbk)

      
      Compute the duty ratios, the zero vector in standby if sensorless.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Extend the post-process method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_speed_ref(ref_fcn)

      
      Set the external speed reference for speed-control mode.

      :param ref_fcn: Speed reference (mechanical rad/s), constant or a function of time.
      :type ref_fcn: float | Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: set_torque_ref(ref_fcn)

      
      Set the external torque reference for torque-control mode.

      :param ref_fcn: Torque reference (Nm), constant or a function of time.
      :type ref_fcn: float | Callable[[float], float]















      ..
          !! processed by numpydoc !!


   .. py:method:: update(ref, fbk)

      
      Update controller states.
















      ..
          !! processed by numpydoc !!


