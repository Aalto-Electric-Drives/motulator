:py:mod:`motulator.control.sm_flux_vector`
==========================================

.. py:module:: motulator.control.sm_flux_vector

.. autoapi-nested-parse::

   Flux-vector control for synchronous motor drives.

   This implements a version of stator-flux-vector control [Ra0108916492f-1]_. Rotor coordinates
   as well as decoupling between the stator flux and torque channels are used [Ra0108916492f-2]_.
   Here, the stator flux magnitude and the electromagnetic torque are selected as
   controllable variables. Proportional controllers are used for simplicity. The
   magnetic saturation is not considered in this implementation.

   .. rubric:: References

   .. [Ra0108916492f-1] Pellegrino, Armando, Guglielmi, “Direct flux field-oriented control of
      IPM drives with variable DC link in the field-weakening region,” IEEE Trans.
      Ind. Appl., 2009, https://doi.org/10.1109/TIA.2009.2027167

   .. [Ra0108916492f-2] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of
      synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl.,
      2019, https://doi.org/10.1109/TIA.2019.2927316

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.sm_flux_vector.SynchronousMotorFluxVectorCtrlPars
   motulator.control.sm_flux_vector.SynchronousMotorFluxVectorCtrl
   motulator.control.sm_flux_vector.Observer




.. py:class:: SynchronousMotorFluxVectorCtrlPars

   
   Control parameters: flux-vector control for synchronous motor drives.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: w_m_ref
      :type: Callable[[float], float]

      

   .. py:attribute:: sensorless
      :type: bool
      :value: True

      

   .. py:attribute:: T_s
      :type: float
      :value: 0.00025

      

   .. py:attribute:: psi_s_min
      :type: float

      

   .. py:attribute:: psi_s_max
      :type: float

      

   .. py:attribute:: k_u
      :type: float
      :value: 0.9

      

   .. py:attribute:: alpha_psi
      :type: float

      

   .. py:attribute:: alpha_tau
      :type: float

      

   .. py:attribute:: alpha_s
      :type: float

      

   .. py:attribute:: tau_M_max
      :type: float

      

   .. py:attribute:: i_s_max
      :type: float

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.6

      

   .. py:attribute:: L_d
      :type: float
      :value: 0.036

      

   .. py:attribute:: L_q
      :type: float
      :value: 0.051

      

   .. py:attribute:: psi_f
      :type: float
      :value: 0.545

      

   .. py:attribute:: n_p
      :type: int
      :value: 3

      

   .. py:attribute:: J
      :type: float
      :value: 0.015

      

   .. py:attribute:: w_o
      :type: float

      

   .. py:attribute:: zeta_inf
      :type: float
      :value: 0.2

      

   .. py:attribute:: g
      :type: float

      


.. py:class:: SynchronousMotorFluxVectorCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   Flux-vector control for synchronous motor drives.

   This class interconnects the subsystems of the control system and
   provides the interface to the solver.

   :param pars: Control parameters.
   :type pars: SynchronousMotoroFluxVectorCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of a synchronous motor drive for getting the
                  feedback signals.
      :type mdl: SynchronousMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!


.. py:class:: Observer(pars)

   
   Sensored observer.

   :param pars: Control parameters.
   :type pars: SynchronousMotoroFluxVectorCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: update(u_s, i_s, w_m)

      
      Update the states for the next sampling period.

      :param u_s: Stator voltage in estimated rotor coordinates.
      :type u_s: complex
      :param i_s: Stator current in estimated rotor coordinates.
      :type i_s: complex
      :param w_m: Rotor speed (in electrical rad/s).
      :type w_m: float















      ..
          !! processed by numpydoc !!


