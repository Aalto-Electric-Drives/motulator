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

   motulator.grid.model.GridConverterSystem
   motulator.grid.model.ACFilter
   motulator.grid.model.LCLFilter
   motulator.grid.model.LFilter
   motulator.grid.model.ThreePhaseVoltageSource


Package Contents
----------------

.. py:class:: GridConverterSystem(converter=None, ac_filter=None, grid_model=None)

   Bases: :py:obj:`motulator.common.model.Model`


   
   Continuous-time model for a grid converter system.

   :param converter: Converter model.
   :type converter: VoltageSourceConverter
   :param ac_filter: Dynamic model for converter output filter and grid impedance.
   :type ac_filter: LFilter | LCLFilter
   :param grid_model: Three-phase grid voltage source model.
   :type grid_model: ThreePhaseVoltageSource















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


.. py:class:: ACFilter

   Bases: :py:obj:`motulator.common.model.Subsystem`


   
   Base class for AC-side filters.

   This provides a base class and wrapper for converter AC-side filters
   (`LFilter`, `LCLFilter`) and grid impedance. Calling this class returns the
   corresponding filter object depending on if a value for the filter
   capacitance `C_f` is given.

   :param filter_par: Filter model parameters.
   :type filter_par: FilterPars
   :param grid_par: Grid model parameters.
   :type grid_par: GridPars















   ..
       !! processed by numpydoc !!

   .. py:method:: meas_currents()

      
      Measure the converter phase currents.

      :returns: **i_c_abc** -- Converter phase currents (A).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_pcc_voltages()

      
      Measure the phase voltages at the point of common coupling (PCC).

      :returns: **u_g_abc** -- Phase voltages at the PCC (V).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


.. py:class:: LCLFilter(filter_par, grid_par)

   Bases: :py:obj:`ACFilter`


   
   Model of an LCL filter and an inductive-resistive grid.

   An LCL filter and an inductive-resistive grid impedance, between the
   converter and grid voltage sources, are modeled. The point-of-common-
   coupling (PCC) voltage between the LCL filter and the grid impedance is
   also calculated.

   :param grid_par: Grid model parameters.
   :type grid_par: GridPars
   :param filter_par: Filter model parameters.
   :type filter_par: FilterPars















   ..
       !! processed by numpydoc !!

   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivatives.
















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_grid_currents()

      
      Measure the grid phase currents.

      :returns: **i_g_abc** -- Grid phase currents (A).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


   .. py:method:: meas_capacitor_voltages()

      
      Measure the capacitor phase voltages.

      :returns: **u_f_abc** -- Phase voltages of the filter capacitor (V).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_with_inputs()

      
      Post-process data with inputs.
















      ..
          !! processed by numpydoc !!


.. py:class:: LFilter(filter_par, grid_par)

   Bases: :py:obj:`ACFilter`


   
   Model of an L filter and an inductive-resistive grid.

   An L filter and an inductive-resistive grid, between the converter and grid
   voltage sources, are modeled combining their inductances and series
   resistances. The point-of-common-coupling (PCC) voltage between the L
   filter and the grid impedance is separately calculated.

   :param grid_par:
                    Grid model parameters. The following parameters are needed:

                        L_g : float
                            Grid inductance (H).
                        R_g : float, optional
                            Series resistance (Ω). The default is 0.
   :type grid_par: GridPars
   :param filter_par:
                      Filter model parameters. The following parameters are needed:

                          L_fc : float
                              Filter inductance (H).
                          R_fc : float, optional
                              Series resistance (Ω). The default is 0.
   :type filter_par: FilterPars















   ..
       !! processed by numpydoc !!

   .. py:method:: set_outputs(_)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivatives.
















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


   .. py:method:: set_outputs(t)

      
      Set output variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: set_inputs(t)

      
      Set input variables.
















      ..
          !! processed by numpydoc !!


   .. py:method:: rhs()

      
      Compute the state derivative.
















      ..
          !! processed by numpydoc !!


   .. py:method:: post_process_states()

      
      Post-process the solution.
















      ..
          !! processed by numpydoc !!


