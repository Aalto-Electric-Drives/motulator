

.. _sphx_glr_grid_examples_grid_following:

Grid-Following Control
----------------------

These examples demonstrate grid-following (GFL) control. The current controller uses 2DOF synchronous-frame complex-vector PI controller, with an additional feedforward term from the low-pass-filtered grid voltage.


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example modifies /grid_examples/grid_following/plot_10kva_gfl by connecting the converter to the grid through an LCL filter. The LCL-filter dynamics are not taken into account in the control system.">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_10kva_lcl_gfl_thumb.png
    :alt:

  :doc:`/grid_examples/grid_following/plot_10kva_lcl_gfl`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA, LCL filter, GFL</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example extends /grid_examples/grid_following/plot_10kva_gfl by modeling the DC-bus dynamics. The control system additionally includes a DC-bus voltage controller.">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_10kva_dc_bus_gfl_thumb.png
    :alt:

  :doc:`/grid_examples/grid_following/plot_10kva_dc_bus_gfl`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA, DC bus, GFL</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 10-kVA grid-following (GFL) converter connected to an L filter and a strong grid. The control system includes a phase-locked loop (PLL) to synchronize with the grid, a current reference generator, and a PI-based current controller.">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_10kva_gfl_thumb.png
    :alt:

  :doc:`/grid_examples/grid_following/plot_10kva_gfl`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA, GFL</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates the start-up of a grid-following converter with DC-bus voltage control. The converter gates are blocked at first. The DC bus is charged from its initial voltage of 500 V through the antiparallel diodes toward the peak line-to-line voltage of the grid, while the phase-locked loop synchronizes with the grid and the current controller tracks the measured grid voltage. The gates are released at 40 ms, after which the DC-bus voltage controller raises the voltage to 650 V, with the power limited to the nominal power. A reactive power reference step follows.">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_10kva_startup_gfl_thumb.png
    :alt:

  :doc:`/grid_examples/grid_following/plot_10kva_startup_gfl`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA, start-up, GFL</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /grid_examples/grid_following/plot_10kva_lcl_gfl
   /grid_examples/grid_following/plot_10kva_dc_bus_gfl
   /grid_examples/grid_following/plot_10kva_gfl
   /grid_examples/grid_following/plot_10kva_startup_gfl

