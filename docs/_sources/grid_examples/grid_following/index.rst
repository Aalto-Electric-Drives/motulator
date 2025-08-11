

.. _sphx_glr_grid_examples_grid_following:

Grid-Following Control
----------------------

These examples demonstrate grid-following (GFL) control. The current controller uses 2DOF synchronous-frame complex-vector PI controller, with an additional feedforward term from the low-pass-filtered grid voltage.



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 10-kVA grid-following (GFL) converter connected to a strong grid through an LCL filter. The control system includes a phase-locked loop (PLL) to synchronize with the grid, a current reference generator, and a PI-type current controller. The LCL-filter dynamics are not taken into account in the control system.">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_10kva_lcl_gfl_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_following_plot_10kva_lcl_gfl.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA, LCL filter, GFL</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 10-kVA grid-following (GFL) converter connected to an L filter and a strong grid. The DC-bus dynamics are modeled. The control system includes a DC-bus voltage controller, a phase-locked loop (PLL) to synchronize with the grid, a current reference generator, and a PI-type current controller.">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_10kva_dc_bus_gfl_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_following_plot_10kva_dc_bus_gfl.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA, DC bus, GFL</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 10-kVA grid-following (GFL) converter connected to an L filter and a strong grid. The control system includes a phase-locked loop (PLL) to synchronize with the grid, a current reference generator, and a PI-based current controller.">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_10kva_gfl_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_following_plot_10kva_gfl.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA, GFL</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /grid_examples/grid_following/plot_10kva_lcl_gfl
   /grid_examples/grid_following/plot_10kva_dc_bus_gfl
   /grid_examples/grid_following/plot_10kva_gfl

