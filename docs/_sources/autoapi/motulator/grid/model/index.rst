motulator.grid.model
====================

.. py:module:: motulator.grid.model

.. autoapi-nested-parse::

   
   Continuous-time grid converter models.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.grid.model.CapacitiveDCBusConverter
   motulator.grid.model.GridConverterSystem
   motulator.grid.model.LCLFilter
   motulator.grid.model.LFilter
   motulator.grid.model.Simulation
   motulator.grid.model.ThreePhaseSource
   motulator.grid.model.VoltageSourceConverter


Package Contents
----------------

.. py:class:: CapacitiveDCBusConverter(u_dc, C_dc)

   Bases: :py:obj:`VoltageSourceConverter`


   
   Lossless voltage-source converter with capacitive DC-bus dynamics.

   :param u_dc: DC-bus voltage (V).
   :type u_dc: float
   :param C_dc: DC-bus capacitance (F).
   :type C_dc: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute state derivatives for DC-bus voltage.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_dc_current(i_dc)

      
      Set external DC current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables for interconnection.
















      ..
          !! processed by numpydoc !!


.. py:class:: GridConverterSystem(converter, ac_filter, ac_source, pwm = False, delay = 1)

   Bases: :py:obj:`motulator.common.model._base.Model`


   
   Continuous-time model for grid-converter systems.

   :param converter: Converter model.
   :type converter: VoltageSourceConverter | CapacitiveDCBusConverter
   :param ac_filter: AC filter model.
   :type ac_filter: LFilter | LCLFilter
   :param ac_source: Three-phase voltage source.
   :type ac_source: ThreePhaseSource
   :param pwm: Enable PWM model, defaults to False.
   :type pwm: bool, optional
   :param delay: Computational delay (samples), defaults to 1.
   :type delay: int, optional















   ..
       !! processed by numpydoc !!

.. py:class:: LCLFilter(L_fc, L_fg, C_f, R_fc = 0.0, R_fg = 0.0, L_g = 0.0, R_g = 0.0, u_f0_ab = 0j)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Model of an LCL filter and an inductive-resistive grid.

   An LCL filter and an inductive-resistive grid impedance, between the converter and
   grid voltage sources, are modeled. The point-of-common-coupling (PCC) voltage
   between the LCL filter and the grid impedance is also calculated.

   :param L_fc: Converter-side filter inductance (H).
   :type L_fc: float
   :param L_fg: Grid-side filter inductance (H).
   :type L_fg: float
   :param C_f: Filter capacitance (F).
   :type C_f: float
   :param R_fc: Series resistance (Ω) of the converter-side inductor, defaults to 0.
   :type R_fc: float, optional
   :param R_fg: Series resistance (Ω) of the grid-side inductor, defaults to 0.
   :type R_fg: float, optional
   :param L_g: Grid inductance (H), defaults to 0.
   :type L_g: float, optional
   :param R_g: Grid resistance (Ω), defaults to 0.
   :type R_g: float, optional
   :param u_f0_ab: Initial value of the filter capacitor voltage (V), defaults to 0.
   :type u_f0_ab: complex, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_capacitor_voltages()

      
      Measure the capacitor phase voltages (V).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure the converter phase currents (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_grid_currents()

      
      Measure the grid phase currents (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_pcc_voltages()

      
      Measure the phase voltages (V) at point of common coupling (PCC).
















      ..
          !! processed by numpydoc !!


   .. py:method:: pcc_voltage(state, inp)

      
      Compute the voltage at the point of common coupling (PCC).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: LFilter(L_f, R_f = 0.0, L_g = 0.0, R_g = 0.0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Model of an L filter and an inductive-resistive grid.

   An L filter and an inductive-resistive grid, between the converter and grid voltage
   sources, are modeled combining their inductances and series resistances. The point-
   of-common-coupling (PCC) voltage between the L filter and the grid impedance is
   calculated.

   :param L_f: Filter inductance (H).
   :type L_f: float
   :param R_f: Series resistance (Ω) of the filter inductor, defaults to 0.
   :type R_f: float, optional
   :param L_g: Grid inductance (H), defaults to 0.
   :type L_g: float, optional
   :param R_g: Grid resistance (Ω), defaults to 0.
   :type R_g: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_currents()

      
      Measure the converter phase currents (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_pcc_voltages()

      
      Measure the phase voltages (V) at the PCC.
















      ..
          !! processed by numpydoc !!


   .. py:method:: pcc_voltage(state, inp)

      
      Compute the voltage at the point of common coupling (PCC).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: Simulation(mdl, ctrl, show_progress = True, cfg = None)

   
   Simulation environment.

   :param mdl: Continuous-time system model.
   :type mdl: Model
   :param ctrl: Discrete-time control system.
   :type ctrl: ControlSystem
   :param show_progress: Show progress during simulation, defaults to True.
   :type show_progress: bool, optional
   :param cfg: Simulation configuration parameters.
   :type cfg: SimulationCfg, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: simulate(t_stop = 1.0)

      
      Solve continuous-time system model and call control system.

      :param t_stop: Simulation stop time, defaults to 1.
      :type t_stop: float, optional















      ..
          !! processed by numpydoc !!


.. py:class:: ThreePhaseSource(w_g = 2 * pi * 50, e_g = sqrt(2 / 3) * 400, phi = 0.0, e_g_neg = 0.0, phi_neg = 0.0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Three-phase source model.

   The frequency, phase shift, and magnitude can be given either as constants or
   functions of time. An unbalanced source can be modeled by specifying a negative-
   sequence component. The zero-sequence component is not included in this model.

   :param w_g: Angular frequency (rad/s), defaults to 2*pi*50.
   :type w_g: float | Callable[[float], float], optional
   :param e_g: Peak-valued magnitude of positive-sequence component, defaults to sqrt(2/3)*400.
   :type e_g: float | Callable[[float], float], optional
   :param phi: Phase shift (rad) of positive-sequence component, defaults to 0.
   :type phi: float | Callable[[float], float], optional
   :param e_g_neg: Peak-valued magnitude of negative-sequence component, defaults to 0.
   :type e_g_neg: float | Callable[[float], float], optional
   :param phi_neg: Phase shift (rad) of negative-sequence component, defaults to 0.
   :type phi_neg: float | Callable[[float], float], optional

   .. rubric:: Notes

   This model is typically used to represent a voltage source, but it can be configured
   to represent, e.g., a current source as well.















   ..
       !! processed by numpydoc !!

   .. py:method:: create_time_series(t)

      
      Create time series from state list.
















      ..
          !! processed by numpydoc !!


   .. py:method:: generate_space_vector(t, exp_j_theta_g)

      
      Generate the space vector in stationary coordinates.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Compute the state derivative.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: VoltageSourceConverter(u_dc)

   Bases: :py:obj:`motulator.common.model._base.Subsystem`


   
   Lossless three-phase voltage-source converter with constant DC-bus voltage.

   :param u_dc: DC-bus voltage (V).
   :type u_dc: float















   ..
       !! processed by numpydoc !!

   .. py:method:: compute_internal_dc_current(inp)

      
      Compute the internal DC current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: create_time_series(t)

      
      Create time series.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_dc_voltage()

      
      Measure converter DC-bus voltage (V).
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs(t)

      
      Default empty implementation.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_external_dc_current(i_dc)
      :abstractmethod:


      
      Set external DC current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


