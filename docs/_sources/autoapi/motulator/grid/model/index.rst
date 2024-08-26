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

   motulator.grid.model.ACFilter
   motulator.grid.model.CarrierComparison
   motulator.grid.model.GridConverterSystem
   motulator.grid.model.LCLFilter
   motulator.grid.model.LFilter
   motulator.grid.model.Simulation
   motulator.grid.model.ThreePhaseVoltageSource
   motulator.grid.model.VoltageSourceConverter


Package Contents
----------------

.. py:class:: ACFilter

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Base class for AC-side filters.

   This provides a base class and wrapper for converter AC-side filters
   (`LFilter`, `LCLFilter`) and grid impedance. Calling this class returns the
   corresponding filter object depending on if a value for the filter
   capacitance `C_f` is given.

   :param par: Filter model parameters.
   :type par: ACFilterPars















   ..
       !! processed by numpydoc !!

   .. py:method:: meas_currents()

      
      Measure the converter phase currents.

      :returns: **i_c_abc** -- Converter phase currents (A).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


.. py:class:: CarrierComparison(N=2**12, return_complex=True)

   
   Carrier comparison.

   This computes the the switching states and their durations based on the
   duty ratios. Instead of searching for zero crossings, the switching
   instants are explicitly computed in the beginning of each sampling period,
   allowing faster simulations.

   :param N: Amount of the counter quantization levels. The default is 2**12.
   :type N: int, optional
   :param return_complex: Complex switching state space vectors are returned if True. Otherwise
                          phase switching states are returned. The default is True.
   :type return_complex: bool, optional

   .. rubric:: Examples

   >>> from motulator.common.model import CarrierComparison
   >>> carrier_cmp = CarrierComparison(return_complex=False)
   >>> # First call gives rising edges
   >>> t_steps, q_c_abc = carrier_cmp(1e-3, [.4, .2, .8])
   >>> # Durations of the switching states
   >>> t_steps
   array([0.00019995, 0.00040015, 0.00019995, 0.00019995])
   >>> # Switching states
   >>> q_c_abc
   array([[0, 0, 0],
          [0, 0, 1],
          [1, 0, 1],
          [1, 1, 1]])
   >>> # Second call gives falling edges
   >>> t_steps, q_c_abc = carrier_cmp(.001, [.4, .2, .8])
   >>> t_steps
   array([0.00019995, 0.00019995, 0.00040015, 0.00019995])
   >>> q_c_abc
   array([[1, 1, 1],
          [1, 0, 1],
          [0, 0, 1],
          [0, 0, 0]])
   >>> # Sum of the step times equals T_s
   >>> np.sum(t_steps)
   0.001
   >>> # 50% duty ratios in all phases
   >>> t_steps, q_c_abc = carrier_cmp(1e-3, [.5, .5, .5])
   >>> t_steps
   array([0.0005, 0.    , 0.    , 0.0005])
   >>> q_c_abc
   array([[0, 0, 0],
          [0, 0, 0],
          [0, 0, 0],
          [1, 1, 1]])















   ..
       !! processed by numpydoc !!

.. py:class:: GridConverterSystem(converter=None, ac_filter=None, ac_source=None)

   Bases: :py:obj:`motulator.common.model.Model`


   
   Continuous-time model for a grid converter system.

   :param converter: Converter model.
   :type converter: VoltageSourceConverter
   :param ac_filter: Dynamic model for converter output filter and grid impedance.
   :type ac_filter: LFilter | LCLFilter
   :param ac_source: Three-phase grid voltage source model.
   :type ac_source: ThreePhaseVoltageSource















   ..
       !! processed by numpydoc !!

   .. py:method:: interconnect(_)

      
      Interconnect the subsystems.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process()

      
      Post-process the solution.
















      ..
          !! processed by numpydoc !!


.. py:class:: LCLFilter(par)

   Bases: :py:obj:`ACFilter`


   
   Model of an LCL filter and an inductive-resistive grid.

   An LCL filter and an inductive-resistive grid impedance, between the
   converter and grid voltage sources, are modeled. The point-of-common-
   coupling (PCC) voltage between the LCL filter and the grid impedance is
   also calculated.

   :param par: Filter model parameters.
   :type par: ACFilterPars















   ..
       !! processed by numpydoc !!

   .. py:method:: meas_capacitor_voltages()

      
      Measure the capacitor phase voltages.

      :returns: **u_f_abc** -- Phase voltages of the filter capacitor (V).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_grid_currents()

      
      Measure the grid phase currents.

      :returns: **i_g_abc** -- Grid phase currents (A).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_pcc_voltages()

      
      Measure the phase voltages at the point of common coupling (PCC).

      :returns: **u_g_abc** -- PCC phase voltages (V).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process data with inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: LFilter(par)

   Bases: :py:obj:`ACFilter`


   
   Model of an L filter and an inductive-resistive grid.

   An L filter and an inductive-resistive grid, between the converter and grid
   voltage sources, are modeled combining their inductances and series
   resistances. The point-of-common-coupling (PCC) voltage between the L
   filter and the grid impedance is calculated.

   :param par:
               Filter model parameters. The following parameters are needed:

                   L_fc : float
                       Filter inductance (H).
                   R_fc : float, optional
                       Series resistance (Ω).
                   L_g : float
                       Grid inductance (H).
                   R_g : float, optional
                       Series resistance (Ω).
   :type par: ACFilterPars















   ..
       !! processed by numpydoc !!

   .. py:method:: meas_pcc_voltages()

      
      Measure the phase voltages at the point of common coupling (PCC).

      :returns: **u_g_abc** -- PCC phase voltages (V).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process data.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process data with inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: Simulation(mdl=None, ctrl=None)

   
   Simulation environment.

   Each simulation object has a system model object and a control system
   object.

   :param mdl: Continuous-time system model.
   :type mdl: Model
   :param ctrl: Discrete-time control system.
   :type ctrl: ControlSystem















   ..
       !! processed by numpydoc !!

   .. py:method:: save_mat(name='sim')

      
      Save the simulation data into MATLAB .mat files.

      :param name: Name for the simulation instance. The default is `sim`.
      :type name: str, optional















      ..
          !! processed by numpydoc !!


   .. py:method:: simulate(t_stop=1, max_step=np.inf)

      
      Solve the continuous-time system model and call the control system.

      :param t_stop: Simulation stop time. The default is 1.
      :type t_stop: float, optional
      :param max_step: Max step size of the solver. The default is inf.
      :type max_step: float, optional

      .. rubric:: Notes

      Other options of `solve_ivp` could be easily used if needed, but, for
      simplicity, only `max_step` is included as an option of this method.















      ..
          !! processed by numpydoc !!


.. py:class:: ThreePhaseVoltageSource(w_g, abs_e_g, phi=0, abs_e_g_neg=0, phi_neg=0)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Three-phase voltage source model.

   The frequency, phase shift, and magnitude can be given either as constants
   or functions of time. An unbalanced source can be modeled by specifying a
   negative-sequence component. Notice that the zero-sequence component is not
   included in this model.

   :param w_g: Angular frequency (rad/s).
   :type w_g: float | callable
   :param abs_e_g: Magnitude of the positive-sequence component (peak value).
   :type abs_e_g: float | callable
   :param phi: Phase shift (rad) of the positive-sequence component. The default is 0.
   :type phi: float | callable, optional
   :param abs_e_g_neg: Magnitude of the negative-sequence component (peak value). The default
                       is 0.
   :type abs_e_g_neg: float | callable, optional
   :param phi_neg: Phase shift (rad) of the negative-sequence component. The default is 0.
   :type phi_neg: float | callable, optional

   .. rubric:: Notes

   This model is typically used to represent a voltage source, but it can be
   configured to represent, e.g., a current source as well.















   ..
       !! processed by numpydoc !!

   .. py:method:: generate_space_vector(t, exp_j_theta_g)

      
      Generate the space vector in stationary coordinates.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process the solution.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivative.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_inputs(t)

      
      Set input variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


.. py:class:: VoltageSourceConverter(u_dc, C_dc=None, i_dc=lambda t: None)

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Lossless three-phase voltage-source converter.

   :param u_dc: DC-bus voltage (V). If the DC-bus capacitor is modeled, this value is
                used as the initial condition.
   :type u_dc: float
   :param C_dc: DC-bus capacitance (F). The default is None.
   :type C_dc: float, optional
   :param i_dc: External current (A) fed to the DC bus. Needed if `C_dc` is not None.
   :type i_dc: callable, optional















   ..
       !! processed by numpydoc !!

   .. py:property:: i_dc_int
      
      Converter-side DC current (A).
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_dc_voltage()

      
      Measure the converter DC-bus voltage (V).
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process data.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process data with inputs.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_inputs(t)

      
      Set input variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:property:: u_cs
      
      AC-side voltage (V).
















      ..
          !! processed by numpydoc !!


   .. py:property:: u_dc
      
      DC-bus voltage (V).
















      ..
          !! processed by numpydoc !!


