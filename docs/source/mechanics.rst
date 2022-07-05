Mechanics
=========

Continuous-time mechanics models are given in the module :mod:`motulator.model.mech`. The stiff rotational mechanics are governed by

.. math::
    J\frac{\mathrm{d}\omega_\mathrm{M}}{\mathrm{d} t} = \tau_\mathrm{M} - \tau_\mathrm{L}

where :math:`\omega_\mathrm{M}` is the mechanical angular speed of the rotor, :math:`\tau_\mathrm{M}` is the electromagnetic torque, and :math:`J` is the total moment of inertia. The total load torque is

.. math::
    \tau_\mathrm{L} = \tau_\ell + B \omega_\mathrm{M} 

where :math:`\tau_\ell` is the external load torque as a function of time and :math:`B` is the viscous friction coefficient.

.. figure:: figs/mech_block.svg
   :width: 35%
   :align: center
   :alt: Block diagram of mechanics
   :target: .

   Block diagram of mechanics.
