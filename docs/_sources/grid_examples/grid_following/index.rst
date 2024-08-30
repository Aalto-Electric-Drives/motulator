

.. _sphx_glr_grid_examples_grid_following:

Grid-Following Control
----------------------

These examples demonstrate grid-following control.

The current controller in the grid following converters uses 2DOF synchronous-frame complex-vector PI controller, with an additional feedforward term. The structure of the controller is shown in the figure below:

.. figure:: /control/figs/complexffpi.svg
   :width: 60%
   :align: center
   :alt: Two-degree-of-freedom complex-vector PI controller, with feedforward.

   The complex-vector gain selection is based on [#Bri2000]_. More details about the gain selection can be found in the :doc:`/control/drive/current_ctrl` section.

.. rubric:: References

.. [#Bri2000] Briz, Degner, Lorenz, "Analysis and design of current regulators using complex vectors," IEEE Trans. Ind. Appl., 2000,https://doi.org/10.1109/28.845057


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="10-kVA converter, LCL filter">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_gfl_lcl_10kva_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_following_plot_gfl_lcl_10kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA converter, LCL filter</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="10-kVA converter, DC-bus voltage">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_gfl_dc_bus_10kva_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_following_plot_gfl_dc_bus_10kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA converter, DC-bus voltage</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="10-kVA converter">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_gfl_10kva_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_following_plot_gfl_10kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA converter</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /grid_examples/grid_following/plot_gfl_lcl_10kva
   /grid_examples/grid_following/plot_gfl_dc_bus_10kva
   /grid_examples/grid_following/plot_gfl_10kva

