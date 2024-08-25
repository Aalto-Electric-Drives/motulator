motulator.grid.control
======================

.. py:module:: motulator.grid.control

.. autoapi-nested-parse::

   
   Controllers for grid-connected converters.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.grid.control.CurrentController
   motulator.grid.control.CurrentLimiter
   motulator.grid.control.CurrentRefCalc
   motulator.grid.control.DCBusVoltageController
   motulator.grid.control.GFLControl
   motulator.grid.control.GFLControlCfg
   motulator.grid.control.GridConverterControlSystem
   motulator.grid.control.ObserverBasedGFMControl
   motulator.grid.control.ObserverBasedGFMControlCfg
   motulator.grid.control.PLL
   motulator.grid.control.RFPSCControl
   motulator.grid.control.RFPSCControlCfg


Package Contents
----------------

.. py:class:: CurrentController(cfg)

   Bases: :py:obj:`motulator.common.control.ComplexPIController`


   
   2DOF PI current controller for grid converters.

   This class provides an interface for a current controller for grid
   converters. The gains are initialized based on the desired closed-loop
   bandwidth and the filter inductance.

   :param cfg:
               Control configuration parameters:

                   filter_par.L_fc : float
                       Converter-side filter inductance (H).
                   alpha_c : float
                       Closed-loop bandwidth (rad/s) of the current controller.
   :type cfg: GFLControlCfg















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentLimiter(max_i)

   
   Limit the amplitude of the input signal.

   :param max_i: Maximum current (A).
   :type max_i: float

   :returns: Limited signal.
   :rtype: complex















   ..
       !! processed by numpydoc !!

.. py:class:: CurrentRefCalc(cfg)

   
   Current controller reference generator

   This class is used to generate the current references for the current
   controllers based on the active and reactive power references. The current
   limiting algorithm is used to limit the current references.















   ..
       !! processed by numpydoc !!

   .. py:method:: get_current_reference(ref)

      
      Current reference generator.
















      ..
          !! processed by numpydoc !!


.. py:class:: DCBusVoltageController(zeta=1, alpha_dc=2 * np.pi * 30, p_max=np.inf)

   Bases: :py:obj:`motulator.common.control.PIController`


   
   DC-bus voltage controller.

   This provides an interface for a DC-bus voltage controller. The gains are
   initialized based on the desired closed-loop bandwidth and the DC-bus
   capacitance estimate. The controller regulates the square of the DC-bus
   voltage in order to have a linear closed-loop system [#Hur2001]_.

   :param zeta: Damping ratio of the closed-loop system. The default is 1.
   :type zeta: float, optional
   :param alpha_dc: Closed-loop bandwidth (rad/s). The default is 2*np.pi*30.
   :type alpha_dc: float, optional
   :param p_max: Maximum converter power (W). The default is `inf`.
   :type p_max: float, optional

   .. rubric:: References

   .. [#Hur2001] Hur, Jung, Nam, "A fast dynamic DC-link power-balancing
      scheme for a PWM converter-inverter system," IEEE Trans. Ind. Electron.,
      2001, https://doi.org/10.1109/41.937412















   ..
       !! processed by numpydoc !!

.. py:class:: GFLControl(cfg)

   Bases: :py:obj:`motulator.grid.control._common.GridConverterControlSystem`


   
   Grid-following control for power converters.

   :param cfg: Control configuration.
   :type cfg: GFLControlCfg

   .. attribute:: current_ctrl

      Current controller.

      :type: CurrentController

   .. attribute:: pll

      Phase locked loop.

      :type: PLL

   .. attribute:: current_reference

      Current reference calculator.

      :type: CurrentRefCalc















   ..
       !! processed by numpydoc !!

   .. py:method:: get_feedback_signals(mdl)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: output(fbk)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: GFLControlCfg

   
   Grid-following control configuration

   :param grid_par: Grid model parameters.
   :type grid_par: GridPars
   :param filter_par: Filter parameters.
   :type filter_par: FilterPars
   :param max_i: Maximum current (A).
   :type max_i: float
   :param T_s: Sampling period (s). The default is 100e-6.
   :type T_s: float, optional
   :param alpha_c: Current-control bandwidth (rad/s). The default is 2*pi*400.
   :type alpha_c: float, optional
   :param alpha_pll: PLL frequency-tracking bandwidth (rad/s). The default is 2*pi*20.
   :type alpha_pll: float, optional
   :param C_dc: DC-bus capacitance (F). The default is None.
   :type C_dc: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: GridConverterControlSystem(grid_par, C_dc, T_s)

   Bases: :py:obj:`motulator.common.control.ControlSystem`, :py:obj:`abc.ABC`


   
   Base class for control of grid-connected converters.

   This base class provides typical functionalities for control of
   grid-connected converters. This can be used both in power control and
   DC-bus voltage control modes.

   :param grid_par: Grid model parameters.
   :type grid_par: GridPars
   :param C_dc: DC-bus capacitance (F). The default is None.
   :type C_dc: float, optional
   :param T_s: Sampling period (s).
   :type T_s: float

   .. attribute:: ref

      References, possibly containing the following fields:

          v : float | callable
              Converter output voltage reference (V). Can be given either as
              a constant or a function of time (s).
          p_g : callable
              Active power reference (W) as a function of time (s). This
              signal is needed in power control mode.
          q_g : callable
              Reactive power reference (VAr) as a function of time (s). This
              signal is needed if grid-following control is used.
          u_dc : callable
              DC-voltage reference (V) as a function of time (s). This signal
              is needed in DC-bus voltage control mode.

      :type: SimpleNamespace

   .. attribute:: dc_bus_volt_ctrl

      DC-bus voltage controller. The default is None.

      :type: DCBusVoltageController | None















   ..
       !! processed by numpydoc !!

   .. py:method:: get_electrical_measurements(fbk, mdl)

      
      Measure the currents and voltages.

      :param fbk: Measured signals are added to this object.
      :type fbk: SimpleNamespace
      :param mdl: Continuous-time system model.
      :type mdl: Model

      :returns: **fbk** --

                Measured signals, containing the following fields:

                    u_dc : float
                        DC-bus voltage (V).
                    i_cs : complex
                        Converter current (A) in stationary coordinates.
                    u_cs : complex
                        Realized converter output voltage (V) in stationary
                        coordinates. This signal is obtained from the PWM.
                    u_gs : complex
                        PCC voltage (V) in stationary coordinates.
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: get_feedback_signals(mdl)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: get_power_reference(fbk, ref)

      
      Get the active power reference in DC bus voltage control mode.

      :param fbk: Feedback signals.
      :type fbk: SimpleNamespace
      :param ref: Reference signals, containing the digital time `t`.
      :type ref: SimpleNamespace

      :returns: **ref** --

                Reference signals, containing the following fields:

                    u_dc : float
                        DC-bus voltage reference (V).
                    p_g : float
                        Active power reference (W).
                    q_g : float
                        Reactive power reference (VAr).
      :rtype: SimpleNamespace















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedGFMControl(cfg)

   Bases: :py:obj:`motulator.grid.control._common.GridConverterControlSystem`


   
   Disturbance-observer-based grid-forming control for grid converters.

   This implements the RFPSC-type grid-forming mode of the control method
   described in [#Nur2024]_. Transparent current control is also implemented.

   :param cfg: Controller configuration parameters.
   :type cfg: ObserverBasedGFMControlCfg

   .. rubric:: Notes

   In this implementation, the control system operates in synchronous
   coordinates rotating at the nominal grid angular frequency, which is worth
   noticing when plotting the results. For other implementation options, see
   [#Nur2024]_.

   .. rubric:: References

   .. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional
       grid-forming converter control based on a disturbance observer, "IEEE
       Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503















   ..
       !! processed by numpydoc !!

   .. py:method:: get_feedback_signals(mdl)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: output(fbk)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: ObserverBasedGFMControlCfg

   
   Disturbance-observer-based grid-forming control configuration.

   :param grid_par: Grid model parameters.
   :type grid_par: GridPars
   :param filter_par: Filter model parameters.
   :type filter_par: FilterPars
   :param max_i: Maximum current modulus (A).
   :type max_i: float
   :param R_a: Active resistance (Ω).
   :type R_a: float
   :param T_s: Sampling period of the controller (s). Default is 100e-6.
   :type T_s: float, optional
   :param k_v: Voltage gain. The default is 1.
   :type k_v: float, optional
   :param alpha_c: Current control bandwidth (rad/s). The default is 2*pi*400.
   :type alpha_c: float, optional
   :param alpha_o: Observer gain (rad/s). The default is 2*pi*50.
   :type alpha_o: float, optional
   :param C_dc: DC-bus capacitance (F). The default is None.
   :type C_dc: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: PLL(alpha_pll, abs_u_g0, w_g0, theta_c0=0)

   
   Phase-locked loop including the voltage-magnitude filtering.

   This class provides a simple frequency-tracking phase-locked loop. The
   magnitude of the measured PCC voltage is also filtered.

   :param alpha_pll: Frequency-tracking bandwidth.
   :type alpha_pll: float
   :param abs_u_g0: Initial value for the grid voltage estimate.
   :type abs_u_g0: float
   :param w_g0: Initial value for the grid angular frequency estimate.
   :type w_g0: float















   ..
       !! processed by numpydoc !!

   .. py:method:: output(fbk)

      
      Output the estimates and coordinate transformed quantities.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, fbk)

      
      Update the integral states.
















      ..
          !! processed by numpydoc !!


.. py:class:: RFPSCControl(cfg)

   Bases: :py:obj:`motulator.grid.control._common.GridConverterControlSystem`


   
   Reference-feedforward power synchronization control for grid converters.

   This implements the reference-feedforward power synchronization control
   (RFPSC) method [#Har2020]_.

   :param cfg: Model and controller configuration parameters.
   :type cfg: PSCControlCfg

   .. rubric:: References

   .. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward
       power-synchronization control," IEEE Trans. Power Electron., 2020,
       https://doi.org/10.1109/TPEL.2020.2970991















   ..
       !! processed by numpydoc !!

   .. py:method:: get_feedback_signals(mdl)

      
      Get the feedback signals.
















      ..
          !! processed by numpydoc !!


   .. py:method:: output(fbk)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(fbk, ref)

      
      Extend the base class method.
















      ..
          !! processed by numpydoc !!


.. py:class:: RFPSCControlCfg

   
   Power synchronization control configuration.

   :param grid_par: Grid model parameters.
   :type grid_par: GridPars
   :param filter_par: Filter model parameters.
   :type filter_par: FilterPars
   :param max_i: Maximum current modulus (A).
   :type max_i: float
   :param R_a: Damping resistance (Ω).
   :type R_a: float
   :param T_s: Sampling period of the controller (s). The default is 100e-6.
   :type T_s: float, optional
   :param w_b: Current low-pass filter bandwidth (rad/s). The default is 2*pi*5.
   :type w_b: float, optional
   :param C_dc: DC-bus capacitance (F). The default is None.
   :type C_dc: float, optional















   ..
       !! processed by numpydoc !!

