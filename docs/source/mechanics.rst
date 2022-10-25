Mechanics
=========

Continuous-time mechanics models are given in the module :mod:`motulator.model.mech`. The stiff rotational mechanics are governed by

.. math::
    J\frac{\mathrm{d}\omega_\mathrm{M}}{\mathrm{d} t} = \tau_\mathrm{M} - \tau_\mathrm{L}

where :math:`\omega_\mathrm{M}` is the mechanical angular speed of the rotor, :math:`\tau_\mathrm{M}` is the electromagnetic torque, and :math:`J` is the total moment of inertia. The total load torque is

.. math::
    \tau_\mathrm{L} = \tau_{\mathrm{L},\omega} + \tau_{\mathrm{L},t}

where :math:`\tau_{\mathrm{L},\omega}` is the speed-dependent load torque and :math:`\tau_{\mathrm{L},t}` is the external load torque as a function of time. The most typical speed-dependent load torque terms are viscous friction :math:`\tau_{\mathrm{L},\omega} = b\omega_\mathrm{M}` (e.g. laminar fluid flow in bearings) and quadratic load torque :math:`\tau_{\mathrm{L},\omega} = k\omega_\mathrm{M}^2\mathrm{sign}(\omega_\mathrm{M})` (e.g. pumps, fans, air resistance in vehicles).

.. figure:: figs/mech_block.svg
   :width: 100%
   :align: center
   :alt: Block diagram of mechanics
   :target: .

   Block diagram of mechanics.
