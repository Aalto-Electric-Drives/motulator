

.. _sphx_glr_grid_examples_grid_forming:

Grid-Forming Control
--------------------

These examples demonstrate grid-forming control. The example :doc:`/grid_examples/grid_forming/plot_13kva_rfpsc_gfm` uses a power-synchronization loop for synchronizing with the grid [#Har2020]_. In :doc:`/grid_examples/grid_forming/plot_13kva_do_gfm`, disturbance-observer-based control is used [#Nur2024]_. The example :doc:`/grid_examples/grid_forming/plot_13kva_do_gfm_negseq` extends the disturbance-observer-based control method with the addition of negative sequence [#Maa2026b]_.

.. rubric:: References

.. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward power-synchronization control," IEEE Trans. Power Electron., 2020, https://doi.org/10.1109/TPEL.2020.2970991

.. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming converter control based on a disturbance observer, "IEEE Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503

.. [#Maa2026b] Määttä, Hinkkanen, Nurminen, Karaca, Mourouvin, Kukkola, Harnefors, "Disturbance-observer-based grid-forming control for unbalanced grids," 2026, https://arxiv.org/abs/2608.11857


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 12.5-kVA grid-forming (GFM) converter using reference-feedforward power-synchronization control (RFPSC). The converter is connected to a weak grid. The converter and grid models are the same as in /grid_examples/grid_forming/plot_13kva_do_gfm.">

.. only:: html

  .. image:: /grid_examples/grid_forming/images/thumb/sphx_glr_plot_13kva_rfpsc_gfm_thumb.png
    :alt:

  :doc:`/grid_examples/grid_forming/plot_13kva_rfpsc_gfm`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA, RFPSC-GFM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 12.5-kVA disturbance-observer-based grid-forming (DO-GFM) converter, connected to a weak grid. The converter output voltage and the active power are directly controlled. Grid synchronization is provided by the disturbance observer. A transparent current controller is included for current limitation. A grid-voltage sag in this weak grid is simulated in /grid_examples/grid_forming/plot_13kva_do_gfm_sag.">

.. only:: html

  .. image:: /grid_examples/grid_forming/images/thumb/sphx_glr_plot_13kva_do_gfm_thumb.png
    :alt:

  :doc:`/grid_examples/grid_forming/plot_13kva_do_gfm`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA, DO-GFM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 12.5-kVA disturbance-observer-based grid-forming (DO-GFM) converter, connected to a weak grid. The control method is designed to take into account negative sequence grid voltage. The positive-sequence converter output voltage and the positive-sequence active power are directly controlled. Grid synchronization is provided by the disturbance observer. A transparent current controller is included for current limitation.">

.. only:: html

  .. image:: /grid_examples/grid_forming/images/thumb/sphx_glr_plot_13kva_do_gfm_negseq_thumb.png
    :alt:

  :doc:`/grid_examples/grid_forming/plot_13kva_do_gfm_negseq`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA, DO-GFM with sequence separation</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 12.5-kVA disturbance-observer-based grid-forming (DO-GFM) converter during a grid-voltage sag in a weak grid. Apart from the sag, the converter and grid models are the same as in /grid_examples/grid_forming/plot_13kva_do_gfm. The grid voltage drops to 0.5 p.u. while the active-power reference is kept at 1 p.u. The active-power reference is limited to a realizable level, prioritizing the reactive current [#Maa2026]_.">

.. only:: html

  .. image:: /grid_examples/grid_forming/images/thumb/sphx_glr_plot_13kva_do_gfm_sag_thumb.png
    :alt:

  :doc:`/grid_examples/grid_forming/plot_13kva_do_gfm_sag`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA, DO-GFM, grid-voltage sag</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /grid_examples/grid_forming/plot_13kva_rfpsc_gfm
   /grid_examples/grid_forming/plot_13kva_do_gfm
   /grid_examples/grid_forming/plot_13kva_do_gfm_negseq
   /grid_examples/grid_forming/plot_13kva_do_gfm_sag

