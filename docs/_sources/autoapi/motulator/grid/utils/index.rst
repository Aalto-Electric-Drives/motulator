motulator.grid.utils
====================

.. py:module:: motulator.grid.utils

.. autoapi-nested-parse::

   
   Utility functions for grid converters.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.grid.utils.BaseValues
   motulator.grid.utils.NominalValues
   motulator.grid.utils.SequenceGenerator
   motulator.grid.utils.Step


Functions
---------

.. autoapisummary::

   motulator.grid.utils.plot
   motulator.grid.utils.plot_voltage_vector


Package Contents
----------------

.. py:class:: BaseValues

   
   Base values.

   :param u: Voltage (V, peak, line-neutral).
   :type u: float
   :param i: Current (A, peak).
   :type i: float
   :param w: Angular frequency (rad/s).
   :type w: float
   :param psi: Flux linkage (Vs).
   :type psi: float
   :param p: Power (W).
   :type p: float
   :param Z: Impedance (Ω).
   :type Z: float
   :param L: Inductance (H).
   :type L: float
   :param C: Capacitance (F).
   :type C: float
   :param tau: Torque (Nm), defaults to 0.
   :type tau: float, optional
   :param n_p: Number of pole pairs, defaults to 0.
   :type n_p: int, optional
   :param w_M: Mechanical angular frequency (rad/s), defaults to 0.
   :type w_M: float, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: from_nominal(nom, n_p = None)
      :classmethod:


      
      Compute base values from nominal values.

      :param nom:
                  Nominal values containing the following fields:
                      U : float
                          Voltage (V, rms, line-line).
                      I : float
                          Current (A, rms).
                      f : float
                          Frequency (Hz).
      :type nom: NominalValues
      :param n_p: Number of pole pairs, defaults to None.
      :type n_p: int | None, optional

      :returns: Base values.
      :rtype: BaseValues

      .. rubric:: Notes

      Notice that the nominal torque is larger than the base torque due to the power
      factor and efficiency being less than unity.















      ..
          !! processed by numpydoc !!


   .. py:method:: unity()
      :classmethod:


      
      Create base values with all values set to 1.
















      ..
          !! processed by numpydoc !!


.. py:class:: NominalValues

   
   Nominal values.

   :param U: Voltage (V, rms, line-line).
   :type U: float
   :param I: Current (A, rms).
   :type I: float
   :param f: Frequency (Hz).
   :type f: float
   :param P: Power (W).
   :type P: float
   :param tau: Torque (Nm), defaults to 0.
   :type tau: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: SequenceGenerator(times, values, periodic = False)

   
   Sequence generator.

   The time array must be increasing. The output values are interpolated between the
   data points.

   :param times: Time values.
   :type times: ndarray
   :param values: Output values.
   :type values: ndarray
   :param periodic: Enables periodicity, defaults to False.
   :type periodic: bool, optional















   ..
       !! processed by numpydoc !!

.. py:class:: Step(step_time, step_value, initial_value = 0.0)

   
   Step function.

   :param step_time: Time of the step.
   :type step_time: float
   :param step_value: Value of the step.
   :type step_value: float
   :param initial_value: Initial value, defaults to 0.
   :type initial_value: float, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot(res, base, t_span = None, latex = False, plot_pcc_voltage = True)

   
   Plot example figures.

   :param res: Should contain the simulated data.
   :type res: SimulationResults
   :param base: Base values for scaling the waveforms. If not given, the waveforms are plotted
                in SI units.
   :type base: BaseValues, optional
   :param t_span: Time span. If not given, the whole simulation time is plotted.
   :type t_span: 2-tuple, optional
   :param plot_pcc_voltage: If True, plot the phase voltage waveforms at the point of common coupling (PCC).
                            Otherwise, plot the grid voltage waveforms, defaults to True.
   :type plot_pcc_voltage: bool, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot_voltage_vector(res, base)

   
   Plot locus of the grid voltage vector.

   :param res: Simulation results.
   :type res: SimulationResults
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional















   ..
       !! processed by numpydoc !!

