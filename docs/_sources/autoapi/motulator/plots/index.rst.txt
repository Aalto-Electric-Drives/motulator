:py:mod:`motulator.plots`
=========================

.. py:module:: motulator.plots

.. autoapi-nested-parse::

   Example plotting scripts.

   ..
       !! processed by numpydoc !!


Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   motulator.plots.plot
   motulator.plots.plot_extra
   motulator.plots.save_plot



.. py:function:: plot(sim, t_span=None, base=None)

   
   Plot example figures.

   Plots figures in per-unit values, if the base values are given. Otherwise
   SI units are used.

   :param sim: Should contain the simulated data.
   :type sim: Simulation object
   :param t_span: Time span. The default is (0, sim.ctrl.t[-1]).
   :type t_span: 2-tuple, optional
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional















   ..
       !! processed by numpydoc !!

.. py:function:: plot_extra(sim, t_span=(1.1, 1.125), base=None)

   
   Plot extra waveforms for a motor drive with a diode bridge.

   :param sim: Should contain the simulated data.
   :type sim: Simulation object
   :param t_span: Time span. The default is (1.1, 1.125).
   :type t_span: 2-tuple, optional
   :param base: Base values for scaling the waveforms.
   :type base: BaseValues, optional















   ..
       !! processed by numpydoc !!

.. py:function:: save_plot(name)

   
   Save figures.

   This saves figures in a folder "figures" in the current directory. If the
   folder doesn't exist, it is created.

   :param name: Name for the figure
   :type name: string
   :param plt: Handle for the figure to be saved
   :type plt: object















   ..
       !! processed by numpydoc !!

