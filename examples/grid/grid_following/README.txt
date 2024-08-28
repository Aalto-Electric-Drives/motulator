Grid-Following Control
----------------------

These examples demonstrate grid-following control.

The current controller in the grid following converters uses 2DOF synchronous-frame complex-vector PI controller, with an additional feedforward term. The structure of the controller is shown in the figure below:

.. figure:: /control/figs/ComplexFFPI.svg
   :width: 60%
   :align: center
   :alt: Two-degree-of-freedom complex-vector PI controller, with feedforward.

   The complex-vector gain selection is based on[#Bri2000]_. More details about the gain selection can be found in the :doc:`/control/drive/current_ctrl` section.

.. rubric:: References

.. [#Bri2000] Briz, Degner, Lorenz, "Analysis and design of current regulators using complex vectors," IEEE Trans. Ind. Appl., 2000,https://doi.org/10.1109/28.845057