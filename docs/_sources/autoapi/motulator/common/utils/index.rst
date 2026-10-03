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
   motulator.common.utils.complex2abc
   motulator.common.utils.complex2line
   motulator.common.utils.dead_time_error
   motulator.common.utils.line2complex


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

.. py:function:: complex2line(u)

   
   Transform a complex space vector to two line-to-line quantities.

   :param u: Complex space vector (peak-value scaling).
   :type u: complex

   :returns: Line-to-line quantities `u_ab` and `u_bc`.
   :rtype: ndarray, shape (2,)

   .. rubric:: Examples

   >>> from motulator.common.utils import complex2line
   >>> y = complex2line(1-.5j)
   >>> y
   array([ 1.9330127, -0.8660254])















   ..
       !! processed by numpydoc !!

.. py:function:: dead_time_error(i_abc, d_abc, t_d, T_s, sign = np.sign)

   
   Compute the switching-cycle averaged duty-ratio error due to dead time.

   The realized duty ratios are `d_abc - dead_time_error(...)`. Away from the duty
   limits, the error is `t_d/(2*T_s)*sign(i_abc)`. Short pulses are suppressed if
   their duration is less than the dead time. Clamped legs have no error.

   :param i_abc: Phase currents (A).
   :type i_abc: ndarray, shape (3,)
   :param d_abc: Duty ratios in the range [0, 1].
   :type d_abc: ndarray, shape (3,)
   :param t_d: Dead time (s).
   :type t_d: float
   :param T_s: Sampling period (s), equal to the half carrier period. In the control system,
               this must equal the sampling period of the controller.
   :type T_s: float
   :param sign: Current-direction function, defaults to `np.sign`. A smooth approximation
                may be used, matching the converter model.
   :type sign: Callable[[np.ndarray], np.ndarray], optional

   :returns: Duty-ratio errors, including the effect of short pulses.
   :rtype: ndarray, shape (3,)

   .. rubric:: Notes

   This averaged model assumes constant duty ratios and currents over a carrier
   period. It does not include transients due to changes in the switching commands.















   ..
       !! processed by numpydoc !!

.. py:function:: line2complex(u)

   
   Transform two line-to-line quantities to a complex space vector.

   :param u: Line-to-line quantities `u_ab` and `u_bc`, e.g., measured voltages.
   :type u: array_like, shape (2,)

   :returns: Complex space vector (peak-value scaling), without the zero sequence.
   :rtype: complex

   .. rubric:: Examples

   >>> from motulator.common.utils import line2complex
   >>> y = line2complex([2, -1])
   >>> y
   (1-0.5773502691896258j)















   ..
       !! processed by numpydoc !!

