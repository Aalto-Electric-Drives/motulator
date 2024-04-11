:orphan:

:py:mod:`motulator.control.im._observers`
=========================================

.. py:module:: motulator.control.im._observers

.. autoapi-nested-parse::

   State observers for induction machines.

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.im._observers.Observer
   motulator.control.im._observers.FullOrderObserver
   motulator.control.im._observers.FluxObserver




.. py:class:: Observer(par, k=None, alpha_o=2 * np.pi * 40, sensorless=True)


   
   Reduced-order flux observer for induction machines in estimated
   rotor flux coordinates.

   This class implements a reduced-order flux observer for induction machines.
   Both sensored and sensorless operation are supported. The observer
   structure is similar to [#Hin2010]_. The observer operates in estimated
   rotor flux coordinates.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param k: Observer gain as a function of the rotor angular speed. The default is
             ``lambda w_m: (0.5*R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)`` if
             `sensorless` else ``lambda w_m: 1 + 0.2*abs(w_m)/(R_R/L_M - 1j*w_m)``.
   :type k: callable, optional
   :param alpha_o: Observer bandwidth (rad/s). The default is 2*pi*40.
   :type alpha_o: float, optional
   :param sensorless: If True, sensorless mode is used. The default is True.
   :type sensorless: bool, optional

   .. attribute:: psi_R

      Rotor flux magnitude estimate (Vs).

      :type: float

   .. attribute:: theta_s

      Rotor flux angle estimate (rad).

      :type: float

   .. attribute:: w_m

      Rotor angular speed estimate (electrical rad/s). In sensored mode, this
      is the measured low-pass-filtered speed.

      :type: float

   .. rubric:: Notes

   The pure voltage model corresponds to ``k = lambda w_m: 0``, resulting in
   the marginally stable estimation-error dynamics. The current model is
   obtained by setting ``k = lambda w_m: 1``.

   .. rubric:: References

   .. [#Hin2010] Hinkkanen, Harnefors, Luomi, "Reduced-order flux observers
      with stator-resistance adaptation for speed-sensorless induction motor
      drives," IEEE Trans. Power Electron., 2010,
      https://doi.org/10.1109/TPEL.2009.2039650















   ..
       !! processed by numpydoc !!

.. py:class:: FullOrderObserver(par, k=None, alpha_o=2 * np.pi * 40, alpha_i=2 * np.pi * 400, sensorless=True)


   
   Full-order flux observer for induction machines in estimated
   rotor flux coordinates.

   This class implements a full-order flux observer for induction machines.
   The observer structure is similar to [#Tii2023]_. The observer operates in
   estimated rotor flux coordinates.

   :param par: Machine model parameters.
   :type par: ModelPars
   :param k: Observer gain as a function of the rotor angular speed. The default is
             ``lambda w_m: (R_R/L_M + 0.2*abs(w_m))/(R_R/L_M - 1j*w_m)``
   :type k: callable, optional
   :param alpha_o: Observer bandwidth (rad/s). The default is 2*pi*40.
   :type alpha_o: float, optional
   :param alpha_i: Current estimation bandwidth (rad/s). The default is 2*pi*400.
   :type alpha_i: float, optional
   :param sensorless: If True, sensorless mode is used. The default is True.
   :type sensorless: bool, optional

   .. attribute:: psi_R

      Rotor flux magnitude estimate (Vs).

      :type: float

   .. attribute:: i_s

      Stator current estimate (A).

      :type: float

   .. attribute:: theta_s

      Rotor flux angle estimate (rad).

      :type: float

   .. attribute:: w_m

      Integral state of the rotor angular speed estimate (electrical rad/s).

      :type: float

   .. rubric:: References

   .. [#Tii2023] Tiitinen, Hinkkanen, Harnefors, "Speed-adaptive full-order
      observer revisited: Closed-form design for induction motor drives,"
      Proc. IEEE SLED, 2023, https://doi.org/10.1109/SLED57582.2023.10261359















   ..
       !! processed by numpydoc !!

.. py:class:: FluxObserver(par, alpha_o, b=None)


   
   Sensorless reduced-order flux observer in external coordinates.

   This is a sensorless reduced-order flux observer in synchronous coordinates
   for an induction machine. The observer gain decouples the electrical and
   mechanical dynamics and allows placing the poles of the linearized
   estimation error dynamics. This implementation operates in external
   coordinates (typically synchronous coordinates defined by reference signals
   of a control system).

   :param par: Machine model parameters.
   :type par: ModelPars
   :param alpha_o: Speed-estimation bandwidth (rad/s).
   :type alpha_o: float
   :param b: Coefficient (rad/s) of the characteristic polynomial as a function of
             the rotor angular speed estimate. The default is
             ``lambda w_m: R_R/L_M + .4*abs(w_m)``.
   :type b: callable, optional

   .. attribute:: psi_R

      Rotor flux estimate (Vs).

      :type: complex

   .. attribute:: w_m

      Rotor angular speed estimate (electrical rad/s).

      :type: float

   .. rubric:: Notes

   The characteristic polynomial of the observer in synchronous coordinates is
   ``s**2 + b*s + w_s**2``.















   ..
       !! processed by numpydoc !!
   .. py:method:: update(T_s, u_s, i_s, w_s)

      
      Update the states.

      :param T_s: Sampling period (s).
      :type T_s: float
      :param u_s: Stator voltage (V) in synchronous coordinates.
      :type u_s: complex
      :param i_s: Stator current (A) in synchronous coordinates.
      :type i_s: complex
      :param w_s: Angular frequency (rad/s) of the coordinate system.
      :type w_s: float















      ..
          !! processed by numpydoc !!


