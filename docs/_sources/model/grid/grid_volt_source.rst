Grid Voltage Source
===================

This document describes continuous-time models for three-phase AC voltage sources.

Ideal three-phase voltage source
--------------------------------

A model for an ideal three-phase voltage source is implemented in the class :class:`motulator.grid.model.ThreePhaseVoltageSource`. The grid voltage space vector is calculated as a combination of a positive sequence and optional negative sequence component as

.. math::
    \boldsymbol{e}_\mathrm{g}^\mathrm{s} = e_\mathrm{g+}\mathrm{e}^{\mathrm{j}(\vartheta_\mathrm{g} + \phi_\mathrm{+})} + e_\mathrm{g-}\mathrm{e}^{-\mathrm{j}(\vartheta_\mathrm{g} + \phi_\mathrm{-})}
    :label: grid_voltage_vector

where :math:`e_\mathrm{g+}` and :math:`e_\mathrm{g-}` are the magnitudes of the positive and negative sequence components, :math:`\phi_\mathrm{+}` and :math:`\phi_\mathrm{-}` the respective phase shifts, and :math:`\vartheta_\mathrm{g}` the angle of the grid voltage. The grid voltage angle is obtained by integrating the angular frequency :math:`\omega_\mathrm{g}`. All parameters can be given as time-varying functions to simulate various fault conditions.