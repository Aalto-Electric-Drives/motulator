motulator.grid.utils
====================

.. py:module:: motulator.grid.utils

.. autoapi-nested-parse::

   
   This module contains utility functions for grid converters.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.grid.utils.BaseValues
   motulator.grid.utils.FilterPars
   motulator.grid.utils.GridPars
   motulator.grid.utils.NominalValues
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
   :param tau: Torque (Nm). Default is None.
   :type tau: float, optional
   :param n_p: Number of pole pairs. Default is None.
   :type n_p: int, optional















   ..
       !! processed by numpydoc !!

   .. py:method:: from_nominal(nom, n_p=None)
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
      :param n_p: Number of pole pairs. If not given it is assumed that base values
                  for a grid converter are calculated. Default is None.
      :type n_p: int, optional

      :returns: Base values.
      :rtype: BaseValues

      .. rubric:: Notes

      Notice that the nominal torque is larger than the base torque due to
      the power factor and efficiency being less than unity.















      ..
          !! processed by numpydoc !!


.. py:class:: FilterPars

   Bases: :py:obj:`abc.ABC`


   
   Filter parameters

   :param L_fc: Converter-side inductance of the filter (H).
   :type L_fc: float
   :param L_fg: Grid-side inductance of the filter (H). The default is 0.
   :type L_fg: float, optional
   :param C_f: Filter capacitance (F). The default is 0.
   :type C_f: float, optional
   :param R_fc: Converter-side series resistance (Ω). The default is 0.
   :type R_fc: float, optional
   :param R_fg: Grid-side series resistance (Ω). The default is 0.
   :type R_fg: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: GridPars

   Bases: :py:obj:`abc.ABC`


   
   Class for grid parameters

   :param u_gN: Nominal grid voltage, phase-to-ground peak value (V).
   :type u_gN: float
   :param w_gN: Nominal grid angular frequency (rad/s).
   :type w_gN: float
   :param L_g: Grid inductance (H). The default is 0.
   :type L_g: float, optional
   :param R_g: Grid resistance (Ω). The default is 0.
   :type R_g: float, optional















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
   :param tau: Torque (Nm). Default value is None.
   :type tau: float, optional















   ..
       !! processed by numpydoc !!

.. py:class:: Step(step_time, step_value, initial_value=0)

   
   Step function.
















   ..
       !! processed by numpydoc !!

.. py:function:: plot(sim, base=None, plot_pcc_voltage=True, plot_w=False, t_span=None)

   
   Plot example figures of grid converter simulations.

   :param sim: Should contain the simulated data.
   :type sim: Simulation
   :param base: Base values for scaling the waveforms. If not given, plots the figures
                in SI units.
   :type base: BaseValues, optional
   :param plot_pcc_voltage: If True, the phase voltage waveforms are plotted at the point of common
                            coupling (PCC). Otherwise, the grid voltage waveforms are plotted. The
                            default is True.
   :type plot_pcc_voltage: bool, optional
   :param plot_w: If True, plot the grid frequency. Otherwise, plot the phase angle. The
                  default is False.
   :type plot_w: bool, optional
   :param t_span: Time span. The default is (0, sim.ctrl.ref.t[-1]).
   :type t_span: 2-tuple, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot_voltage_vector(sim, base=None)

   
   Plot locus of the grid voltage vector.

   :param sim: Should contain the simulated data.
   :type sim: Simulation
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional















   ..
       !! processed by numpydoc !!

