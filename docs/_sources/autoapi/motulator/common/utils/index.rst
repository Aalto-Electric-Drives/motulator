motulator.common.utils
======================

.. py:module:: motulator.common.utils

.. autoapi-nested-parse::

   
   Common utilities.
















   ..
       !! processed by numpydoc !!


Classes
-------

.. autoapisummary::

   motulator.common.utils.SequenceGenerator
   motulator.common.utils.Step


Functions
---------

.. autoapisummary::

   motulator.common.utils.abc2complex
   motulator.common.utils.clip
   motulator.common.utils.complex2abc
   motulator.common.utils.empty_array
   motulator.common.utils.get_value
   motulator.common.utils.sign
   motulator.common.utils.wrap


Package Contents
----------------

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

.. py:function:: abc2complex(u)

   
   Transform three-phase quantities to a complex space vector.

   :param u: Phase quantities.
   :type u: array_like, shape (3,)

   :returns: Complex space vector (peak-value scaling).
   :rtype: complex

   .. rubric:: Examples

   >>> from motulator.common.utils import abc2complex
   >>> y = abc2complex([1, 2, 3])
   >>> y
   (-1-0.5773502691896258j)















   ..
       !! processed by numpydoc !!

.. py:function:: clip(value, min_value, max_value)

   
   Clip a value between minimum and maximum.
















   ..
       !! processed by numpydoc !!

.. py:function:: complex2abc(u)

   
   Transform a complex space vector to three-phase quantities.

   :param u: Complex space vector (peak-value scaling).
   :type u: complex

   :returns: Phase quantities.
   :rtype: ndarray, shape (3,)

   .. rubric:: Examples

   >>> from motulator.common.utils import complex2abc
   >>> y = complex2abc(1-.5j)
   >>> y
   array([ 1.       , -0.9330127, -0.0669873])















   ..
       !! processed by numpydoc !!

.. py:function:: empty_array()

   
   Return an empty array.
















   ..
       !! processed by numpydoc !!

.. py:function:: get_value(u, x)

   
   Helper to get the value of an object that is either callable or float.

   :param u: Input object.
   :type u: Any | Callable[[Any], Any] | None
   :param x: Argument to the callable object.
   :type x: Any

   :returns: Values of `u(x)` if callable, otherwise `u`.
   :rtype: Any















   ..
       !! processed by numpydoc !!

.. py:function:: sign(x)

   
   Return the sign of x: -1 for negative, 0 for zero, 1 for positive.
















   ..
       !! processed by numpydoc !!

.. py:function:: wrap(theta)

   
   Limit the angle into the range [-pi, pi).

   :param theta: Angle (rad).
   :type theta: float

   :returns: Limited angle.
   :rtype: float















   ..
       !! processed by numpydoc !!

