motulator.drive.utils
=====================

.. py:module:: motulator.drive.utils

.. autoapi-nested-parse::

   
   This module contains utility functions for machine drives.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.drive.utils.BaseValues
   motulator.drive.utils.NominalValues
   motulator.drive.utils.Sequence
   motulator.drive.utils.Step


Functions
---------

.. autoapisummary::

   motulator.drive.utils.plot
   motulator.drive.utils.plot_extra
   motulator.drive.utils.import_syre_data
   motulator.drive.utils.plot_flux_map
   motulator.drive.utils.plot_flux_vs_current
   motulator.drive.utils.plot_torque_map


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
   :param tau: Torque (Nm).
   :type tau: float
   :param n_p: Number of pole pairs.
   :type n_p: int















   ..
       !! processed by numpydoc !!

   .. py:method:: from_nominal(nom, n_p)
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
      :param n_p: Number of pole pairs.
      :type n_p: int

      :returns: Base values.
      :rtype: BaseValues

      .. rubric:: Notes

      Notice that the nominal torque is larger than the base torque due to
      the power factor and efficiency being less than unity.















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
   :param tau: Torque (Nm).
   :type tau: float















   ..
       !! processed by numpydoc !!

.. py:function:: plot(sim, base=None, t_span=None)

   
   Plot example figures.

   Plots figures in per-unit values, if the base values are given. Otherwise
   SI units are used.

   :param sim: Should contain the simulated data.
   :type sim: Simulation
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional
   :param t_span: Time span. The default is (0, sim.ctrl.t[-1]).
   :type t_span: 2-tuple, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot_extra(sim, base=None, t_span=None)

   
   Plot extra waveforms for a motor drive with a diode bridge.

   :param sim: Should contain the simulated data.
   :type sim: Simulation
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional
   :param t_span: Time span. The default is (0, sim.ctrl.t[-1]).
   :type t_span: 2-tuple, optional















   ..
       !! processed by numpydoc !!

.. py:function:: import_syre_data(fname, add_negative_q_axis=True)

   
   Import a flux map from the MATLAB data file in the SyR-e format.

   For more information on the SyR-e project and the MATLAB file format,
   please visit:

       https://github.com/SyR-e/syre_public

   The imported data is converted to the PMSM coordinate convention, in which
   the PM flux is along the d axis.

   :param fname: MATLAB file name.
   :type fname: str
   :param add_negative_q_axis: Adds the negative q-axis data based on the symmetry.
   :type add_negative_q_axis: bool, optional

   :returns: * *SimpleNamespace object with the following fields defined*
             * **i_s** (*complex ndarray*) -- Stator current data (A).
             * **psi_s** (*complex ndarray*) -- Stator flux linkage data (Vs).
             * **tau_M** (*ndarray*) -- Torque data (Nm).

   .. rubric:: Notes

   Some example data files (including THOR.mat) are available in the SyR-e
   repository, licensed under the Apache License, Version 2.0.















   ..
       !! processed by numpydoc !!

.. py:function:: plot_flux_map(data)

   
   Plot the flux linkage as function of the current.

   :param data: Flux map data.
   :type data: SimpleNamespace















   ..
       !! processed by numpydoc !!

.. py:function:: plot_flux_vs_current(data)

   
   Plot the flux vs. current characteristics.

   :param data: Flux map data.
   :type data: SimpleNamespace















   ..
       !! processed by numpydoc !!

.. py:function:: plot_torque_map(data)

   
   Plot the torque as function of the current.

   :param data: Flux map data.
   :type data: SimpleNamespace















   ..
       !! processed by numpydoc !!

.. py:class:: Sequence(times, values, periodic=False)

   
   Sequence generator.

   The time array must be increasing. The output values are interpolated
   between the data points.

   :param times: Time values.
   :type times: ndarray
   :param values: Output values.
   :type values: ndarray
   :param periodic: Enables periodicity. The default is False.
   :type periodic: bool, optional















   ..
       !! processed by numpydoc !!

.. py:class:: Step(step_time, step_value, initial_value=0)

   
   Step function.
















   ..
       !! processed by numpydoc !!

