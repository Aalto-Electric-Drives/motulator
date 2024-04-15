:orphan:

:py:mod:`motulator.model._lc_filter`
====================================

.. py:module:: motulator.model._lc_filter

.. autoapi-nested-parse::

   Continuous-time model for an output LC filter.

   The space vector model is implemented in stator coordinates.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model._lc_filter.LCFilter




.. py:class:: LCFilter(L, C, R_L=0, G_C=0)


   
   LC-filter model.

   :param L: Inductance (H).
   :type L: float
   :param C: Capacitance (F).
   :type C: float
   :param R_L: Series resistance (Ω) of the inductor. The default is 0.
   :type R_L: float, optional
   :param G_C: Parallel conductance (S) of the capacitor. The default is 0.
   :type G_C: float, optional















   ..
       !! processed by numpydoc !!
   .. py:method:: f(i_cs, u_ss, u_cs, i_ss)

      
      Compute the state derivative.

      :param i_cs: Converter output current (A).
      :type i_cs: complex
      :param u_ss: Capacitor (stator) voltage (V).
      :type u_ss: complex
      :param u_cs: Converter output voltage (V).
      :type u_cs: complex
      :param i_ss: Stator current (A).
      :type i_ss: complex

      :returns: Time derivative of the state vector, [di_cs, du_ss]
      :rtype: complex list, length 2















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_currents()

      
      Returns the converter phase currents at the end of the sampling period.

      :returns: **i_c_abc** -- Phase currents (A).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_voltages()

      
      Returns the capacitor (stator) phase voltages at the end of the
      sampling period.

      :returns: **u_s_abc** -- Phase voltages (V).
      :rtype: 3-tuple of floats















      ..
          !! processed by numpydoc !!


