:orphan:

:py:mod:`motulator.control.im._vhz`
===================================

.. py:module:: motulator.control.im._vhz

.. autoapi-nested-parse::

   V/Hz control for induction motor drives.

   The method is similar to [#Hin2022]_. Open-loop V/Hz control can be obtained as
   a special case by choosing::

       R_s, R_R = 0, 0
       k_u, k_w = 0, 0

   .. rubric:: References

   .. [#Hin2022] Hinkkanen, Tiitinen, Mölsä, Harnefors, "On the stability of
      volts-per-hertz control for induction motors," IEEE J. Emerg. Sel. Topics
      Power Electron., 2022, https://doi.org/10.1109/JESTPE.2021.3060583

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.im._vhz.VHzCtrl




.. py:class:: VHzCtrl(T_s, par, psi_s_nom, k_u=1.0, k_w=4.0, six_step=False)

   Bases: :py:obj:`motulator.control._common.Ctrl`

   
   V/Hz control with the stator current feedback.

   :param par: Control parameters.
   :type par: ModelPars















   ..
       !! processed by numpydoc !!

