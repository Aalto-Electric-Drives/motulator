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
   motulator.common.utils.Sequence
   motulator.common.utils.Step


Functions
---------

.. autoapisummary::

   motulator.common.utils.abc2complex
   motulator.common.utils.complex2abc
   motulator.common.utils.wrap


Package Contents
----------------

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

.. py:function:: wrap(theta)

   
   Limit the angle into the range [-pi, pi).

   :param theta: Angle (rad).
   :type theta: float

   :returns: Limited angle.
   :rtype: float















   ..
       !! processed by numpydoc !!

