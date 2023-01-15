:py:mod:`motulator.model.converter`
===================================

.. py:module:: motulator.model.converter

.. autoapi-nested-parse::

   Power converter models.

   An inverter with constant DC-bus voltage and a frequency converter with a diode
   front-end rectifier are modeled. Complex space vectors are used also for duty
   ratios and switching states, wherever applicable. In this module, all space
   vectors are in stationary coordinates. The default values correspond to a
   2.2-kW 400-V motor drive.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.model.converter.Inverter
   motulator.model.converter.FrequencyConverter




.. py:class:: Inverter(u_dc=540)

   
   Inverter with constant DC-bus voltage and switching-cycle averaging.

   :param u_dc: DC-bus voltage.
   :type u_dc: float















   ..
       !! processed by numpydoc !!
   .. py:method:: ac_voltage(q, u_dc)
      :staticmethod:

      
      Compute the AC-side voltage of a lossless inverter.

      :param q: Switching state vector.
      :type q: complex
      :param u_dc: DC-bus voltage.
      :type u_dc: float

      :returns: **u_ac** -- AC-side voltage.
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: dc_current(q, i_ac)
      :staticmethod:

      
      Compute the DC-side current of a lossless inverter.

      :param q: Switching state vector.
      :type q: complex
      :param i_ac: AC-side current.
      :type i_ac: complex

      :returns: **i_dc** -- DC-side current.
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: meas_dc_voltage()

      
      Measure the DC-bus voltage.

      :returns: DC-bus voltage.
      :rtype: float















      ..
          !! processed by numpydoc !!


.. py:class:: FrequencyConverter(L=0.002, C=0.000235, U_g=400, f_g=50)

   Bases: :py:obj:`Inverter`

   
   Frequency converter.

   This extends the Inverter class with models for a strong grid, a
   three-phase diode-bridge rectifier, an LC filter, and a three-phase
   inverter.

   :param L: DC-bus inductance.
   :type L: float
   :param C: DC-bus capacitance.
   :type C: float
   :param U_g: Grid voltage (line-line, rms).
   :type U_g: float
   :param f_g: Grid frequency.
   :type f_g: float















   ..
       !! processed by numpydoc !!
   .. py:method:: grid_voltages(t)

      
      Compute three-phase grid voltages.

      :param t: Time.
      :type t: float

      :returns: **u_g_abc** -- The phase voltages.
      :rtype: ndarray of floats, shape (3,)















      ..
          !! processed by numpydoc !!

   .. py:method:: f(t, u_dc, i_L, i_dc)

      
      Compute the state derivatives.

      :param t: Time.
      :type t: float
      :param u_dc: DC-bus voltage over the capacitor.
      :type u_dc: float
      :param i_L: DC-bus inductor current.
      :type i_L: float
      :param i_dc: Current to the inverter.
      :type i_dc: float

      :returns: Time derivative of the state vector, [du_dc, d_iL]
      :rtype: list, length 2















      ..
          !! processed by numpydoc !!


