AC Filter and Grid Impedance
============================

This document describes continuous-time models of an AC filter and grid impedance. The dynamics of the AC filters and grid impedance are modeled in the class :class:`motulator.grid.model.ACFilter`. Based on the filter parameters the ACFilter class initializes either an L- or LCL filter. The parameters of the AC filter and grid impedance are defined in the data class :class:`motulator.grid.utils.ACFilterPars`.

L Filter
--------

A dynamic model of an L filter and inductive-resistive grid impedance is implemented in the class :class:`motulator.grid.model.LFilter`. The model in stationary coordinates is

.. math::
   L_\mathrm{t}\frac{\mathrm{d}\boldsymbol{i}_\mathrm{c}^\mathrm{s}}{\mathrm{d} t}
   = \boldsymbol{u}_\mathrm{c}^\mathrm{s} - \boldsymbol{e}_\mathrm{g}^\mathrm{s}
   - R_\mathrm{t}\boldsymbol{i}_\mathrm{c}^\mathrm{s}
   :label: L_filt_model

where :math:`\boldsymbol{i}_\mathrm{c}^\mathrm{s}` is the converter current, :math:`\boldsymbol{u}_\mathrm{c}^\mathrm{s}` is the converter voltage, :math:`\boldsymbol{e}_\mathrm{g}^\mathrm{s}` is the grid voltage, :math:`R_\mathrm{t} = R_\mathrm{fc} + R_\mathrm{g}` is the total resistance comprising the filter series resistance :math:`R_\mathrm{fc}` and the grid resistance :math:`R_\mathrm{g}`, and :math:`L_\mathrm{t} = L_\mathrm{fc} + L_\mathrm{g}` is the total inductance comprising the filter inductance :math:`L_\mathrm{fc}` and the grid inductance :math:`L_\mathrm{g}`. The point of common coupling (PCC) is modeled to be between the L filter and the grid impedance. The voltage at the PCC is

.. math::
   \boldsymbol{u}_\mathrm{g}^\mathrm{s}
   = \frac{L_\mathrm{g}(\boldsymbol{u}_\mathrm{c}^\mathrm{s}
   - R_\mathrm{fc}\boldsymbol{i}_\mathrm{c}^\mathrm{s})
   + L_\mathrm{fc}(\boldsymbol{e}_\mathrm{g}^\mathrm{s}
   + R_\mathrm{g}\boldsymbol{i}_\mathrm{c}^\mathrm{s})}{L_\mathrm{t}}
   :label: L_filt_PCC_voltage

.. figure:: ../figs/l_filter.svg
   :width: 100%
   :align: center
   :alt: Diagram of L filter and grid impedance
   :target: .

   L filter and inductive-resistive grid impedance.

LCL Filter
----------

A dynamic model of an LCL filter and inductive-resistive grid impedance is implemented in the class :class:`motulator.grid.model.LCLFilter`. The model in stationary coordinates is

.. math::
   L_\mathrm{fc}\frac{\mathrm{d}\boldsymbol{i}_\mathrm{c}^\mathrm{s}}{\mathrm{d} t}
   &= \boldsymbol{u}_\mathrm{c}^\mathrm{s}
   - \boldsymbol{u}_\mathrm{f}^\mathrm{s}
   - R_\mathrm{fc}\boldsymbol{i}_\mathrm{c}^\mathrm{s}\\
   C_\mathrm{f}\frac{\mathrm{d}\boldsymbol{u}_\mathrm{f}^\mathrm{s}}{\mathrm{d} t}
   &= \boldsymbol{i}_\mathrm{c}^\mathrm{s}
   - \boldsymbol{i}_\mathrm{g}^\mathrm{s}\\
   L_\mathrm{t}\frac{\mathrm{d}\boldsymbol{i}_\mathrm{g}^\mathrm{s}}{\mathrm{d} t}
   &= \boldsymbol{u}_\mathrm{f}^\mathrm{s}
   - \boldsymbol{e}_\mathrm{g}^\mathrm{s}
   - R_\mathrm{t}\boldsymbol{i}_\mathrm{g}^\mathrm{s}
   :label: LCL_filt_model

where :math:`\boldsymbol{i}_\mathrm{c}^\mathrm{s}` is the converter current, :math:`\boldsymbol{i}_\mathrm{g}^\mathrm{s}` is the grid current, and :math:`\boldsymbol{u}_\mathrm{f}^\mathrm{s}` is the capacitor voltage. The converter-side and grid-side inductances of the LCL filter are :math:`L_\mathrm{fc}` and :math:`L_\mathrm{fg}`, respectively, and their series resistances are :math:`R_\mathrm{fc}` and :math:`R_\mathrm{fg}`, respectively. The filter capacitance is :math:`C_\mathrm{f}`. The total grid-side inductance and resistance are :math:`L_\mathrm{t} = L_\mathrm{fg} + L_\mathrm{g}` and :math:`R_\mathrm{t} = R_\mathrm{fg} + R_\mathrm{g}`, respectively. The PCC is modeled to be between the LCL filter and the inductive-resistive grid impedance. The voltage at the PCC is

.. math::
   \boldsymbol{u}_\mathrm{g}^\mathrm{s}
   = \frac{L_\mathrm{g}(\boldsymbol{u}_\mathrm{f}^\mathrm{s}
   - R_\mathrm{fg}\boldsymbol{i}_\mathrm{g}^\mathrm{s})
   + L_\mathrm{fg}(\boldsymbol{e}_\mathrm{g}^\mathrm{s}
   + R_\mathrm{g}\boldsymbol{i}_\mathrm{g}^\mathrm{s})}{L_\mathrm{t}}
   :label: LCL_filt_PCC_voltage

.. figure:: ../figs/lcl_filter.svg
   :width: 100%
   :align: center
   :alt: Diagram of LCL filter and grid impedance
   :target: .

   LCL filter and inductive-resistive grid impedance.
