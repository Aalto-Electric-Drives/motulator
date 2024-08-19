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

   motulator.grid.utils.FilterPars
   motulator.grid.utils.GridPars


Functions
---------

.. autoapisummary::

   motulator.grid.utils.plot_grid
   motulator.grid.utils.plot_voltage_vector


Package Contents
----------------

.. py:function:: plot_grid(sim, base=None, plot_pcc_voltage=False, plot_w=False, t_span=None)

   
   Plot example figures of grid converter simulations.

   :param sim: Should contain the simulated data.
   :type sim: Simulation
   :param base: Base values for scaling the waveforms. If not given, plots the figures
                in SI units.
   :type base: BaseValues, optional
   :param plot_pcc_voltage: If True, plot the phase voltage waveforms at the point of common
                            coupling (PCC). Otherwise, plot the grid voltage waveforms. The default
                            is False.
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

