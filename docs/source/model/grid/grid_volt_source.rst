Three-Phase Source
==================

A model for an ideal three-phase source is implemented in the class :class:`motulator.grid.model.ThreePhaseSource`. Typically, this model is used to represent the grid voltage source. The voltage space vector is calculated as a combination of a positive-sequence and optional negative-sequence components as

.. math::
    \frac{\mathrm{d}\vartheta_\mathrm{g}}{\mathrm{d} t} &= \omega_\mathrm{g} \\
    \boldsymbol{e}_\mathrm{g}^\mathrm{s} &= e_\mathrm{g+}\mathrm{e}^{\mathrm{j}(\vartheta_\mathrm{g} + \phi_\mathrm{+})} + e_\mathrm{g-}\mathrm{e}^{-\mathrm{j}(\vartheta_\mathrm{g} + \phi_\mathrm{-})}
    :label: grid_voltage_vector

where :math:`e_\mathrm{g+}` and :math:`e_\mathrm{g-}` are the magnitudes of the positive-sequence and negative-sequence components, respectively, and :math:`\phi_\mathrm{+}` and :math:`\phi_\mathrm{-}` are the positive-sequence and negative-sequence phase shifts, respectively. The angle :math:`\vartheta_\mathrm{g}` is obtained by integrating the angular frequency :math:`\omega_\mathrm{g}`. All parameters can be given as time-varying functions to simulate various fault conditions.
