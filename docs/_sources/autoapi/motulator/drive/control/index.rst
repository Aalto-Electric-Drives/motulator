motulator.drive.control
=======================

.. py:module:: motulator.drive.control

.. autoapi-nested-parse::

   
   Controllers for machine drives.
















   ..
       !! processed by numpydoc !!


Submodules
----------

.. toctree::
   :maxdepth: 1

   /autoapi/motulator/drive/control/im/index
   /autoapi/motulator/drive/control/sm/index


Classes
-------

.. autoapisummary::

   motulator.drive.control.SpeedController


Package Contents
----------------

.. py:class:: SpeedController(J, alpha_s, alpha_i = None, tau_M_max = inf)

   Bases: :py:obj:`motulator.common.control._controllers.PIController`


   
   2DOF PI speed controller.

   This is an interface for a speed controller. The gains are initialized based on the
   desired closed-loop bandwidth and the rotor inertia estimate.

   :param J: Total inertia of the rotor (kgm²).
   :type J: float
   :param alpha_s: Reference-tracking bandwidth (rad/s).
   :type alpha_s: float
   :param alpha_i: Integral action bandwidth (rad/s), defaults to `alpha_s`.
   :type alpha_i: float, optional
   :param tau_M_max: Maximum motor torque (Nm), defaults to `inf`.
   :type tau_M_max: float, optional















   ..
       !! processed by numpydoc !!

