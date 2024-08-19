motulator.grid.control
======================

.. py:module:: motulator.grid.control

.. autoapi-nested-parse::

   
   Controllers for grid-connected converters.
















   ..
       !! processed by numpydoc !!


Subpackages
-----------

.. toctree::
   :maxdepth: 1

   /autoapi/motulator/grid/control/grid_following/index
   /autoapi/motulator/grid/control/grid_forming/index


Classes
-------

.. autoapisummary::

   motulator.grid.control.CurrentLimiter
   motulator.grid.control.DCBusVoltageController
   motulator.grid.control.GridConverterControlSystem
   motulator.grid.control.PLL


Package Contents
----------------

.. py:class:: CurrentLimiter(max_i)

   
   Limit the amplitude of the input signal.

   :param max_i: Maximum current (A).
   :type max_i: float

   :returns: Limited signal.
   :rtype: complex















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


.. py:class:: PLL(k_p, k_i, w_g0, theta_g0=0)

   
   Phase-locked loop.

   :param k_p: Proportional gain.
   :type k_p: float
   :param k_i: Integral gain.
   :type k_i: float
   :param w_g0: Initial value for the grid angular frequency estimate.
   :type w_g0: float















   ..
       !! processed by numpydoc !!

   .. py:method:: output(fbk)

      
      Compute the frequency and phase angle estimates.
















      ..
          !! processed by numpydoc !!


   .. py:method:: update(T_s, fbk)

      
      Update the integral states.
















      ..
          !! processed by numpydoc !!


