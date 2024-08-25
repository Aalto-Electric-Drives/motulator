Six-Pulse Diode Bridge
======================

The figure below shows a six-pulse diode bridge rectifier, where the inductor :math:`L_\mathrm{dc}` is placed in the DC link. For simplicity, a three-phase supply voltage is assumed to be stiff. The DC current dynamics are  

.. math:: 
   L_\mathrm{dc} \frac{\mathrm{d}i_\mathrm{L}}{\mathrm{d}t}
   = u_\mathrm{di} - u_\mathrm{dc} 
   :label: diode_bridge

where :math:`i_\mathrm{L}` is the DC-bus current, :math:`u_\mathrm{di}` is the voltage over the diode bridge, :math:`u_\mathrm{dc}` is the DC-bus voltage, and :math:`L_{\mathrm{dc}}` is the DC-bus inductance. 

.. figure:: ../figs/diode_bridge.svg
   :width: 100%
   :align: center
   :alt: Six-pulse diode bridge rectifier an three-phase supply voltage
   :target: .

   Six-pulse diode bridge rectifier.

The voltage-source converter described in :doc:`/model/common` is extended with this diode bridge model in the class :class:`motulator.drive.model.FrequencyConverter`. Examples using the six-pulse diode bridge can be found in :doc:`/drive_examples/vhz/plot_vhz_ctrl_im_2kw` and :doc:`/drive_examples/vector/plot_vector_ctrl_pmsm_2kw_diode`.


