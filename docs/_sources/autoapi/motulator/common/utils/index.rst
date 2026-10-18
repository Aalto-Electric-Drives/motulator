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

   motulator.common.utils.BaseValues
   motulator.common.utils.NominalValues
   motulator.common.utils.SequenceGenerator
   motulator.common.utils.Step


Functions
---------

.. autoapisummary::

   motulator.common.utils.abc2complex
   motulator.common.utils.complex2abc
   motulator.common.utils.complex2line
   motulator.common.utils.line2complex
   motulator.common.utils.set_style


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

.. py:function:: set_style(latex = False)

   
   Configure matplotlib for screen viewing or for LaTeX documents.

   The plotting functions of motulator use this style, and it can be set before
   drawing other figures in the same style (e.g., in related packages).

   :param latex: Use the style for LaTeX documents instead of the style for screen viewing,
                 defaults to False.
   :type latex: bool, optional















   ..
       !! processed by numpydoc !!

