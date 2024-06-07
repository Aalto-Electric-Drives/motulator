Output LC Filter
================

This document describes a continuous-time model of an LC filter, which may be used between the converter and the electric machine in some applications [#Sal2006]_. Space vectors are used to represent the three-phase quantities. The subscript c and s refer to the converter-side and the stator-side quantities, respectively. The superscript s refers to the stationary coordinates. 

.. figure:: figs/lc_filter.svg
   :width: 100%
   :align: center
   :alt: LC filter
   :target: .
   
   LC filter.

A dynamic model of the filter is

.. math::
   L \frac{\mathrm{d}\boldsymbol{i}_\mathrm{c}^\mathrm{s}}{\mathrm{d} t} 
   &= \boldsymbol{u}_\mathrm{c}^\mathrm{s} - \boldsymbol{u}_\mathrm{s}^\mathrm{s} 
   - R \boldsymbol{i}_\mathrm{c}^\mathrm{s} \\
   C \frac{\mathrm{d}\boldsymbol{u}_\mathrm{s}^\mathrm{s}}{\mathrm{d} t} 
   &= \boldsymbol{i}_\mathrm{c}^\mathrm{s} 
   - \boldsymbol{i}_\mathrm{s}^\mathrm{s}     
   :label: LC_filter_model

where :math:`L` and :math:`R` are the inductance and the series resistance of the inductor, respectively, and :math:`C` is the capacitance. Furthermore, :math:`\boldsymbol{i}_\mathrm{c}^\mathrm{s}` is the converter current, :math:`\boldsymbol{i}_\mathrm{s}^\mathrm{s}` is the stator current, :math:`\boldsymbol{u}_\mathrm{c}^\mathrm{s}` is the converter voltage, and :math:`\boldsymbol{u}_\mathrm{s}^\mathrm{s}` is the capacitor voltage (corresponding to the stator voltage). 

The filter model is implemented in the class :class:`motulator.drive.model.LCFilter`. For its usage, see the example :doc:`/auto_examples/vhz/plot_vhz_ctrl_im_2kw_lc`. 

.. rubric:: References

.. [#Sal2006] Salomäki, Hinkkanen, Luomi, "Sensorless control of induction motor drives equipped with inverter output filter," IEEE Trans. Ind. Electron., 2006, https://doi.org/10.1109/TIE.2006.878314
