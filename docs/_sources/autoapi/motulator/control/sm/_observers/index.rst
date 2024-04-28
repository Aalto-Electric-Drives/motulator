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




.. py:class:: Observer(par, alpha_o=2 * np.pi * 40, k=None, k_f=None, sensorless=True)


   
   Observer for synchronous machines in estimated rotor coordinates.

   This observer estimates the stator flux linkage, the rotor angle, the rotor
   speed, and (optionally) the PM-flux linkage. The design is based on
   [#Hin2018]_ and [#Tuo2018]. The observer gain decouples the electrical and
   mechanical dynamics and allows placing the poles of the corresponding
   linearized estimation error dynamics. The PM-flux linkage can also be
   estimated [#Tuo2018]_. This implementation operates in estimated rotor
   coordinates. The observer can  also be used in the sensored mode by
   providing the measured rotor speed as an input.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param alpha_o: Observer bandwidth (electrical rad/s). The default is 2*pi*40.
   :type alpha_o: float, optional
   :param k: Observer gain as a function of the rotor angular speed. The default is
             ``lambda w_m: 0.25*(R_s*(L_d + L_q)/(L_d*L_q) + 0.2*abs(w_m))`` if
             `sensorless` else ``lambda w_m: 2*pi*15``.
   :type k: callable, optional
   :param k_f: PM-flux estimation gain (V) as a function of the rotor angular speed.
               The default is zero, ``lambda w_m: 0``. A typical nonzero gain is of
               the form ``lambda w_m: max(k*(abs(w_m) - w_min), 0)``, i.e., zero below
               the speed `w_min` (rad/s) and linearly increasing above that with the
               slope `k` (Vs).
   :type k_f: callable, optional
   :param sensorless: If True, sensorless control is used. The default is True.
   :type sensorless: bool, optional

   .. attribute:: theta_m

      Rotor angle estimate (electrical rad).

      :type: float

   .. attribute:: w_m

      Rotor speed estimate (electrical rad/s).

      :type: float

   .. attribute:: psi_s

      Stator flux estimate (Vs).

      :type: complex

   .. attribute:: psi_f

      PM-flux estimate (Vs). The PM-flux estimate lumps various disturbances.
      Hence, it can become slightly negative in the case of SyRMs.

      :type: float

   .. rubric:: Notes

   The sensored mode assumes that the control system operates in the measured
   rotor coordinates, i.e., the rotor-angle tracking is disabled.

   .. rubric:: References

   .. [#Hin2018] Hinkkanen, Saarakkala, Awan, Mölsä, Tuovinen, "Observers for
      sensorless synchronous motor drives: Framework for design and analysis,"
      IEEE Trans. Ind. Appl., 2018, https://doi.org/10.1109/TIA.2018.2858753

   .. [#Tuo2018] Tuovinen, Awan, Kukkola, Saarakkala, Hinkkanen, "Permanent-
      magnet flux adaptation for sensorless synchronous motor drives," Proc.
      IEEE SLED, 2018, https://doi.org/10.1109/SLED.2018.8485899















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

   This observer estimates the stator flux linkage and the angle of the
   coordinate system with respect to the d-axis of the rotor. Speed-estimation
   is omitted. The observer gain decouples the electrical and mechanical
   dynamics and allows placing the poles of the corresponding linearized
   estimation error dynamics. This implementation operates in external
   coordinates (typically synchronous coordinates defined by reference signals
   of a control system).

   :param par: Machine model parameters.
   :type par: ModelPars
   :param alpha_o: Observer gain (rad/s). The default is 2*pi*20.
   :type alpha_o: float, optional
   :param zeta_inf: Damping ratio at infinite speed. The default is 0.2.
   :type zeta_inf: float, optional

   .. attribute:: delta

      Angle estimate of the coordinate system with respect to the d-axis of
      the rotor (electrical rad).

      :type: float

   .. attribute:: psi_s

      Stator flux estimate (Vs).

      :type: complex















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


