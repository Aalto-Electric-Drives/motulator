:orphan:

:py:mod:`motulator.model._converter`
====================================

.. py:module:: motulator.model._converter

.. autoapi-nested-parse::

   Power converter models.

   An inverter with constant DC-bus voltage and a frequency converter with a diode
   front-end rectifier are modeled. Complex space vectors are used also for duty
   ratios and switching states, wherever applicable. In this module, all space
   vectors are in stationary coordinates.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model._converter.Inverter
   motulator.model._converter.FrequencyConverter




.. py:class:: Inverter(u_dc)


   
   Inverter with constant DC-bus voltage and switching-cycle averaging.

   :param u_dc: DC-bus voltage (V).
   :type u_dc: float















   ..
       !! processed by numpydoc !!
   .. py:method:: ac_voltage(q, u_dc)
      :staticmethod:

      
      Compute the AC-side voltage of a lossless inverter.

      :param q: Switching state vector.
      :type q: complex
      :param u_dc: DC-bus voltage (V).
      :type u_dc: float

      :returns: **u_ac** -- AC-side voltage (V).
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: dc_current(q, i_ac)
      :staticmethod:

      
      Compute the DC-side current of a lossless inverter.

      :param q: Switching state vector.
      :type q: complex
      :param i_ac: AC-side current (A).
      :type i_ac: complex

      :returns: **i_dc** -- DC-side current (A).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_dc_voltage()

      
      Measure the DC-bus voltage.

      :returns: DC-bus voltage (V).
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: FrequencyConverter(L, C, U_g, f_g)


   Bases: :py:obj:`Inverter`

   
   Frequency converter.

   This extends the Inverter class with models for a strong grid, a
   three-phase diode-bridge rectifier, an LC filter, and a three-phase
   inverter.

   :param L: DC-bus inductance (H).
   :type L: float
   :param C: DC-bus capacitance (F).
   :type C: float
   :param U_g: Grid voltage (V, line-line, rms).
   :type U_g: float
   :param f_g: Grid frequency (Hz).
   :type f_g: float















   ..
       !! processed by numpydoc !!
   .. py:method:: grid_voltages(t)

      
      Compute three-phase grid voltages.

      :param t: Time (s).
      :type t: float

      :returns: **u_g_abc** -- Phase voltages (V).
      :rtype: ndarray of floats, shape (3,)















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, u_dc, i_L, i_dc)

      
      Compute the state derivatives.

      :param t: Time (s).
      :type t: float
      :param u_dc: DC-bus voltage (V) over the capacitor.
      :type u_dc: float
      :param i_L: DC-bus inductor current (A).
      :type i_L: float
      :param i_dc: Current to the inverter (A).
      :type i_dc: float

      :returns: Time derivative of the state vector, [du_dc, di_L]
      :rtype: list, length 2















      ..
          !! processed by numpydoc !!


