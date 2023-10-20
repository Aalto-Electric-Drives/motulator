:orphan:

:py:mod:`motulator.control.sm._observers`
=========================================

.. py:module:: motulator.control.sm._observers

.. autoapi-nested-parse::

   State observers for synchronous machines.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.sm._observers.Observer
   motulator.control.sm._observers.FluxObserver




.. py:class:: Observer(par, alpha_o=2 * np.pi * 40, k=None, sensorless=True)


   
   Observer for synchronous machines in estimated rotor coordinates.

   This observer estimates the rotor angle, the rotor speed, and the stator
   flux linkage. The design is based on [#Hin2018]_. The observer gain
   decouples the electrical and mechanical dynamics and allows placing the
   poles of the corresponding linearized estimation error dynamics. This
   implementation operates in estimated rotor coordinates. The observer can
   also be used in the sensored mode by providing the measured rotor speed as
   an input.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param alpha_o: Observer bandwidth (electrical rad/s). The default is 2*pi*40.
   :type alpha_o: float, optional
   :param k: Observer gain as a function of the rotor angular speed. The default is
             ``lambda w_m: 0.25*(R_s*(L_d + L_q)/(L_d*L_q) + 0.2*abs(w_m))`` if
             `sensorless` else ``lambda w_m: 2*pi*15``.
   :type k: callable, optional

   .. attribute:: theta_m

      Rotor angle estimate (electrical rad).

      :type: float

   .. attribute:: w_m

      Rotor speed estimate (electrical rad/s).

      :type: float

   .. attribute:: psi_s

      Stator flux estimate (Vs).

      :type: complex

   .. rubric:: References

   .. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
      sensorless synchronous motor drives: Framework for design and analysis,"
      IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753















   ..
       !! processed by numpydoc !!
   .. py:method:: update(T_s, u_s, i_s, w_m=None)

      
      Update the states for the next sampling period.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_s: Stator voltage (V) in estimated rotor coordinates.
      :type u_s: complex
      :param i_s: Stator current (A) in estimated rotor coordinates.
      :type i_s: complex
      :param w_m: Rotor angular speed (electrical rad/s). Needed only in the sensored
                  mode. The default is None.
      :type w_m: float, optional















      ..
          !! processed by numpydoc !!


.. py:class:: FluxObserver(par, alpha_o=2 * np.pi * 20, zeta_inf=0.2)


   
   Sensorless stator flux observer in external coordinates.

   This observer estimates the stator flux linkage and the angle of the coordinate
   system with respect to the d-axis of the rotor. Speed-estimation is omitted.
   The observer gain decouples the electrical and mechanical dynamics and allows placing
   the poles of the corresponding linearized estimation error dynamics. This implementation operates
   in external coordinates (typically synchronous coordinates defined by reference signals
   of a control system).

   :param par: Machine model parameters.
   :type par: ModelPars
   :param alpha_o: Observer gain (rad/s). The default is 2*pi*20.
   :type alpha_o: float, optional
   :param zeta_inf: Damping ratio at infinite speed. The default is 0.2.
   :type zeta_inf: float, optional

   .. attribute:: delta

      Angle estimate of the coordinate system with respect
      to the d-axis of the rotor (electrical rad).

      :type: float

   .. attribute:: psi_s

      Stator flux estimate (Vs).

      :type: complex

   .. rubric:: References

   .. [#Tii2022] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors,
      "Stable and passive observer-based V/Hz control for synchronous Motors,"
      Proc. IEEE ECCE, Detroit, MI, Oct. 2022,
      https://doi.org/10.1109/ECCE50734.2022.9947858















   ..
       !! processed by numpydoc !!
   .. py:method:: update(T_s, u_s, i_s, w_s)

      
      Update the states for the next sampling period.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_s: Stator voltage (V).
      :type u_s: complex
      :param i_s: Stator current (A).
      :type i_s: complex
      :param w_s: Stator angular frequency (rad/s).
      :type w_s: float















      ..
          !! processed by numpydoc !!


