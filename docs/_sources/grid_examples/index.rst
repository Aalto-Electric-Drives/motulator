:orphan:

Grid Converters
===============

A collection of example scripts for grid converters.



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
