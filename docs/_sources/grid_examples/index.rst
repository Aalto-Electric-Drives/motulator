:orphan:

Grid Converters
===============

A collection of example scripts for grid converters.


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. thumbnail-parent-div-close

.. raw:: html

    </div>

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

Grid-Forming Control
--------------------

These examples demonstrate grid-forming control. The example :doc:`/grid_examples/grid_forming/plot_13kva_rfpsc_gfm` uses a power-synchronization loop for synchronizing with the grid [#Har2020]_. In :doc:`/grid_examples/grid_forming/plot_13kva_do_gfm`, disturbance-observer-based control is used [#Nur2024]_. The example :doc:`/grid_examples/grid_forming/plot_13kva_do_gfm_negseq` extends the disturbance-observer-based control method with the addition of negative sequence [#Maa2026b]_.

.. rubric:: References

.. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward power-synchronization control," IEEE Trans. Power Electron., 2020, https://doi.org/10.1109/TPEL.2020.2970991

.. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming converter control based on a disturbance observer," IEEE Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503

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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 12.5-kVA disturbance-observer-based grid-forming (DO-GFM) converter, connected to a weak grid. The control method is designed to take into account negative-sequence grid voltage. The positive-sequence converter output voltage and the positive-sequence active power are directly controlled. Grid synchronization is provided by the disturbance observer. A transparent current controller is included for current limitation.">

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
   :includehidden:


   /grid_examples/grid_following/index.rst
   /grid_examples/grid_forming/index.rst


.. only:: html

  .. container:: sphx-glr-footer sphx-glr-footer-gallery

    .. container:: sphx-glr-download sphx-glr-download-python

      :download:`Download all examples in Python source code: grid_examples_python.zip </grid_examples/grid_examples_python.zip>`

    .. container:: sphx-glr-download sphx-glr-download-jupyter

      :download:`Download all examples in Jupyter notebooks: grid_examples_jupyter.zip </grid_examples/grid_examples_jupyter.zip>`


.. only:: html

 .. rst-class:: sphx-glr-signature

    `Gallery generated by Sphinx-Gallery <https://sphinx-gallery.github.io>`_
