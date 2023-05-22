:orphan:

:py:mod:`motulator._plots`
==========================

.. py:module:: motulator._plots

.. autoapi-nested-parse::

   Example plotting scripts.

   ..
       !! processed by numpydoc !!


Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   motulator._plots.plot
   motulator._plots.plot_extra
   motulator._plots.save_plot



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

.. py:function:: save_plot(name)

   
   Save figures.

   This saves figures in a folder "figures" in the current directory. If the
   folder does not exist, it is created.

   :param name: Name for the figure
   :type name: string















   ..
       !! processed by numpydoc !!

