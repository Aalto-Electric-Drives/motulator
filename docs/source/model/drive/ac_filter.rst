Output LC Filter
================

An LC filter may be used between the voltage-source converter and the electric machine in some applications [#Sal2006]_. The subscripts c and s refer to the converter-side and stator-side quantities, respectively. The superscript s refers to the stationary coordinates.

.. figure:: ../figs/lc_filter.svg
   :figclass: only-light
   :width: 100%
   :align: center
   :alt: LC filter
   :target: .

   LC filter.

.. figure:: ../figs/lc_filter.svg
   :figclass: invert-colors-dark only-dark
   :width: 100%
   :align: center
   :alt: LC filter
   :target: .

   LC filter.

A dynamic model of the filter is

.. math::
   L_\mathrm{f} \frac{\mathrm{d}\boldsymbol{i}_\mathrm{c}^\mathrm{s}}{\mathrm{d} t}
   &= \boldsymbol{u}_\mathrm{c}^\mathrm{s} - \boldsymbol{u}_\mathrm{s}^\mathrm{s}
   - R_\mathrm{f} \boldsymbol{i}_\mathrm{c}^\mathrm{s} \\
   C_\mathrm{f} \frac{\mathrm{d}\boldsymbol{u}_\mathrm{s}^\mathrm{s}}{\mathrm{d} t}
   &= \boldsymbol{i}_\mathrm{c}^\mathrm{s} - \boldsymbol{i}_\mathrm{s}^\mathrm{s}
   :label: LC_filter_model

where :math:`L_\mathrm{f}` is the filter inductance, :math:`R_\mathrm{f}` is the series resistance of the filter inductor, and :math:`C_\mathrm{f}` the filter capacitance. Furthermore, :math:`\boldsymbol{i}_\mathrm{c}^\mathrm{s}` is the converter current, :math:`\boldsymbol{i}_\mathrm{s}^\mathrm{s}` is the stator current, :math:`\boldsymbol{u}_\mathrm{c}^\mathrm{s}` is the converter voltage, and :math:`\boldsymbol{u}_\mathrm{s}^\mathrm{s}` is the capacitor voltage (corresponding to the stator voltage).

The filter model is implemented in the class :class:`motulator.drive.model.LCFilter`. For its usage, see the example :doc:`/drive_examples/vhz/plot_vhz_im_2kw_lc`.

.. rubric:: References

.. [#Sal2006] Salomäki, Hinkkanen, Luomi, "Sensorless control of induction motor drives equipped with inverter output filter," IEEE Trans. Ind. Electron., 2006, https://doi.org/10.1109/TIE.2006.878314
