Grid Voltage Source
===================

This document describes continuous-time models for a three-phase AC voltage source.
This section will be extended in the future.

Stiff Source
------------

A model for a stiff grid voltage is implemented in the class :class:`motulator.grid.model.StiffSource`.
The model is an ideal three-phase voltage source, where the phase voltages are

.. math::
    e_\mathrm{ga} &= \hat{e}_\mathrm{ga}\cos(\vartheta_\mathrm{p} - \phi) \\
    e_\mathrm{gb} &= \hat{e}_\mathrm{gb}\cos(\vartheta_\mathrm{p} - 2\pi/3 - \phi) \\
    e_\mathrm{gc} &= \hat{e}_\mathrm{gc}\cos(\vartheta_\mathrm{p} - 4\pi/3 - \phi)
    :label: grid_voltages

The peak values of the phase voltages are :math:`\hat{e}_\mathrm{ga}`, :math:`\hat{e}_\mathrm{gb}`
and :math:`\hat{e}_\mathrm{gc}`, which can be given separately as time-dependent functions to
simulate nonsymmetric grid faults. Furthermore, :math:`\vartheta_\mathrm{p}` is the
grid phase voltage angle and :math:`\phi` an additional phase shift. If the phase voltages are symmetric,
:math:`\vartheta_\mathrm{g}=\vartheta_\mathrm{p}` holds, where :math:`\vartheta_\mathrm{g}` is the
angle of the grid voltage space vector :math:`\boldsymbol{e}_\mathrm{g}^\mathrm{s}`.

Nonsymmetry in the grid voltage vector can also be modeled with a negative sequence component which is defined as

.. math::
    \boldsymbol{e}_\mathrm{g-}^\mathrm{s} = \hat{e}_\mathrm{g-}\mathrm{e}^{-\mathrm{j}(\vartheta_\mathrm{p} + \phi_\mathrm{u-} + \phi)}
    :label: neg_sequence

where :math:`\hat{e}_\mathrm{g-}` is the magnitude of the negative sequence component and :math:`\phi_\mathrm{u-}` is the
phase shift of the negative sequence.