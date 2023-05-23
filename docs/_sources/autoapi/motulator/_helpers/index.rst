:orphan:

:py:mod:`motulator._helpers`
============================

.. py:module:: motulator._helpers

.. autoapi-nested-parse::

   Helper functions and classes.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator._helpers.BaseValues
   motulator._helpers.Sequence
   motulator._helpers.Step



Functions
~~~~~~~~~

.. autoapisummary::

   motulator._helpers.abc2complex
   motulator._helpers.complex2abc



.. py:function:: abc2complex(u)

   
   Transform three-phase quantities to a complex space vector.

   :param u: Phase quantities.
   :type u: array_like, shape (3,)

   :returns: Complex space vector (peak-value scaling).
   :rtype: complex

   .. rubric:: Examples

   >>> from motulator import abc2complex
   >>> y = abc2complex([1, 2, 3])
   >>> y
   (-1-0.5773502691896258j)















   ..
       !! processed by numpydoc !!

.. py:function:: complex2abc(u)

   
   Transform a complex space vector to three-phase quantities.

   :param u: Complex space vector (peak-value scaling).
   :type u: complex

   :returns: Phase quantities.
   :rtype: ndarray, shape (3,)

   .. rubric:: Examples

   >>> from motulator import complex2abc
   >>> y = complex2abc(1-.5j)
   >>> y
   array([ 1.       , -0.9330127, -0.0669873])















   ..
       !! processed by numpydoc !!

.. py:class:: BaseValues

   
   Base values.

   Base values are computed from the nominal values and the number of pole
   pairs. They can be used, e.g., for scaling the plotted waveforms.

   :param U_nom: Voltage (V, rms, line-line).
   :type U_nom: float
   :param I_nom: Current (A, rms).
   :type I_nom: float
   :param f_nom: Frequency (Hz).
   :type f_nom: float
   :param tau_nom: Torque (Nm).
   :type tau_nom: float
   :param P_nom: Power (W).
   :type P_nom: float
   :param n_p: Number of pole pairs.
   :type n_p: int

   .. attribute:: u

      Base voltage (V, peak, line-neutral).

      :type: float

   .. attribute:: i

      Base current (A, peak).

      :type: float

   .. attribute:: w

      Base angular frequency (rad/s).

      :type: float

   .. attribute:: psi

      Base flux linkage (Vs).

      :type: float

   .. attribute:: p

      Base power (W).

      :type: float

   .. attribute:: Z

      Base impedance (Ω).

      :type: float

   .. attribute:: L

      Base inductance (H).

      :type: float

   .. attribute:: tau

      Base torque (Nm).

      :type: float















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

