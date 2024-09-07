

.. _sphx_glr_grid_examples_grid_following:

Grid-Following Control
----------------------

These examples demonstrate grid-following control. The current controller uses 2DOF synchronous-frame complex-vector PI controller, with an additional feedforward term from the low-pass filtered grid voltage.



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a grid-following-controlled converter connected to a strong grid through an LCL filter. The control system includes a phase-locked loop (PLL) to synchronize with the grid, a current reference generator, and a PI-type current controller. The dynamics of the LCL filter are not taken into account in the control system.">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_gfl_lcl_10kva_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_following_plot_gfl_lcl_10kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA converter, LCL filter</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a grid-following-controlled converter connected to a strong grid and regulating the DC-bus voltage. The control system includes a DC-bus voltage controller, a phase-locked loop (PLL) to synchronize with the grid, a current reference generator, and a PI-type current controller.">

.. only:: html

  .. image:: /grid_examples/grid_following/images/thumb/sphx_glr_plot_gfl_dc_bus_10kva_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_following_plot_gfl_dc_bus_10kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">10-kVA converter, DC-bus voltage</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a grid-following-controlled converter connected to an L filter and a strong grid. The control system includes a phase-locked loop (PLL) to synchronize with the grid, a current reference generator, and a PI-based current controller.">

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

