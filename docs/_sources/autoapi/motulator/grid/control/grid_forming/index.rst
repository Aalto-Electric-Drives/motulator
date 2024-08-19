motulator.grid.control.grid_forming
===================================

.. py:module:: motulator.grid.control.grid_forming

.. autoapi-nested-parse::

   
   Controls for grid-forming converters.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.grid.control.grid_forming.ObserverBasedGFMControl
   motulator.grid.control.grid_forming.ObserverBasedGFMControlCfg
   motulator.grid.control.grid_forming.RFPSCControl
   motulator.grid.control.grid_forming.RFPSCControlCfg


Package Contents
----------------

.. py:class:: ObserverBasedGFMControl(cfg)

   Bases: :py:obj:`motulator.grid.control.GridConverterControlSystem`


   
   Disturbance observer-based grid-forming control for grid converters.

   This implements the disturbance observer-based control method described in
   [#Nur2024]_. More specifically, the grid-forming mode using RFPSC-type
   gains is implemented, with transparent current control.

   :param cfg: Model and controller configuration parameters.
   :type cfg: ObserverBasedGFMControlCfg

   .. attribute:: observer

      Disturbance observer.

      :type: DisturbanceObserver

   .. rubric:: References

   .. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional
       Grid-Forming Converter Control Based on a Disturbance Observer, "IEEE
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

   
   Observer GFM control configuration.

   :param grid_par: Grid model parameters.
   :type grid_par: GridPars
   :param filter_par: Filter model parameters.
   :type filter_par: FilterPars
   :param max_i: Maximum current modulus (A).
   :type max_i: float
   :param R_a: Active resistance (Ω).
   :type R_a: float
   :param T_s: Sampling period of the controller (s). Default is 1/(16e3).
   :type T_s: float, optional
   :param k_v: Voltage gain. Default is 1.
   :type k_v: float, optional
   :param alpha_c: Current control bandwidth (rad/s). The default is 2*pi*400.
   :type alpha_c: float, optional
   :param alpha_o: Observer gain (rad/s). The default is 2*pi*50.
   :type alpha_o: float, optional
   :param C_dc: DC-bus capacitance (F). Default is None.
   :type C_dc: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: RFPSCControl(cfg)

   Bases: :py:obj:`motulator.grid.control.GridConverterControlSystem`


   
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
   :param R_a: Damping resistance (Ω). Default is 4.6.
   :type R_a: float, optional
   :param T_s: Sampling period of the controller (s). The default is 100e-6.
   :type T_s: float, optional
   :param w_b: Current low-pass filter bandwidth (rad/s). The default is 2*pi*5.
   :type w_b: float, optional
   :param C_dc: DC-bus capacitance (F). The default is None.
   :type C_dc: float, optional















   ..
       !! processed by numpydoc !!

