

.. _sphx_glr_grid_examples_grid_forming:

Grid-Forming Control
--------------------

These examples demonstrate grid-forming control. The example :doc:`/grid_examples/grid_forming/plot_13kva_rfpsc_gfm` uses a power-synchronization loop for synchronizing with the grid [#Har2020]_. In :doc:`/grid_examples/grid_forming/plot_13kva_do_gfm`, disturbance-observer-based control is used [#Nur2024]_.

.. rubric:: References

.. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward power-synchronization control," IEEE Trans. Power Electron., 2020, https://doi.org/10.1109/TPEL.2020.2970991

.. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming converter control based on a disturbance observer, "IEEE Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates grid-forming (GFM) converter using reference-feedforward power-synchronization control (RFPSC). The converter is connected to a weak grid.">

.. only:: html

  .. image:: /grid_examples/grid_forming/images/thumb/sphx_glr_plot_13kva_rfpsc_gfm_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_forming_plot_13kva_rfpsc_gfm.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA, RFPSC-GFM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 12.5-kVA disturbance-observer-based grid-forming (DO-GFM) converter, connected to a weak grid. The converter output voltage and the active power are directly controlled. Grid synchronization is provided by the disturbance observer. A transparent current controller is included for current limitation.">

.. only:: html

  .. image:: /grid_examples/grid_forming/images/thumb/sphx_glr_plot_13kva_do_gfm_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_forming_plot_13kva_do_gfm.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA, DO-GFM</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /grid_examples/grid_forming/plot_13kva_rfpsc_gfm
   /grid_examples/grid_forming/plot_13kva_do_gfm

