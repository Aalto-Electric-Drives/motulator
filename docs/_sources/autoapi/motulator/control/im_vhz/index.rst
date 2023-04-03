:py:mod:`motulator.control.im_vhz`
==================================

.. py:module:: motulator.control.im_vhz

.. autoapi-nested-parse::

   V/Hz control for induction motor drives.

   The method is similar to [R1c4f0ace2b68-1]_. Open-loop V/Hz control can be obtained as a
   special case by choosing::

       R_s, R_R = 0, 0
       k_u, k_w = 0, 0

   .. rubric:: Notes

   The low-pass-filtered values are marked with ref at the end of the variable
   name. These slowly varying quasi-steady-state quantities can be seen to
   represent the operating point (marked with the subscript 0 in [R1c4f0ace2b68-1]_).

   .. rubric:: References

   .. [R1c4f0ace2b68-1] Hinkkanen, Tiitinen, Mölsä, Harnefors, "On the stability of
      volts-per-hertz control for induction motors," IEEE J. Emerg. Sel. Topics
      Power Electron., 2022, https://doi.org/10.1109/JESTPE.2021.3060583

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.im_vhz.InductionMotorVHzCtrlPars
   motulator.control.im_vhz.InductionMotorVHzCtrl




.. py:class:: InductionMotorVHzCtrlPars

   
   V/Hz control parameters.
















   ..
       !! processed by numpydoc !!
   .. py:attribute:: w_m_ref
      :type: Callable[[float], float]

      

   .. py:attribute:: T_s
      :type: float
      :value: 0.00025

      

   .. py:attribute:: six_step
      :type: bool
      :value: False

      

   .. py:attribute:: psi_s_nom
      :type: float
      :value: 1.04

      

   .. py:attribute:: rate_limit
      :type: float

      

   .. py:attribute:: R_s
      :type: float
      :value: 3.7

      

   .. py:attribute:: R_R
      :type: float
      :value: 2.1

      

   .. py:attribute:: L_sgm
      :type: float
      :value: 0.021

      

   .. py:attribute:: L_M
      :type: float
      :value: 0.224

      

   .. py:attribute:: k_u
      :type: float
      :value: 1

      

   .. py:attribute:: k_w
      :type: float
      :value: 4

      


.. py:class:: InductionMotorVHzCtrl(pars)

   Bases: :py:obj:`motulator.control.common.Ctrl`

   
   V/Hz control with the stator current feedback.

   :param pars: Control parameters.
   :type pars: InductionMotorVHzCtrlPars















   ..
       !! processed by numpydoc !!
   .. py:method:: __call__(mdl)

      
      Run the main control loop.

      :param mdl: Continuous-time model of an induction motor drive for getting the
                  feedback signals.
      :type mdl: InductionMotorDrive

      :returns: * **T_s** (*float*) -- Sampling period.
                * **d_abc_ref** (*ndarray, shape (3,)*) -- Duty ratio references.















      ..
          !! processed by numpydoc !!

   .. py:method:: stator_freq(w_s_ref, i_s)

      
      Compute the dynamic stator frequency.

      This computes the dynamic stator frequency reference used in the
      coordinate transformations.















      ..
          !! processed by numpydoc !!

   .. py:method:: voltage_reference(w_s, i_s)

      
      Compute the stator voltage reference.
















      ..
          !! processed by numpydoc !!


