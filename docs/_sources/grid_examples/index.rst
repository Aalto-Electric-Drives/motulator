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

These examples demonstrate grid-following control. The current controller uses 2DOF synchronous-frame complex-vector PI controller, with an additional feedforward term from the low-pass filtered grid voltage.



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a grid-following-controlled converter connected to a strong grid through an LCL filter. The control system includes a phase-locked loop (PLL) to synchronize with the grid, a current reference generator, and a PI-type current controller. The LCL-filter dynamics are not taken into account in the control system.">

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

Grid-Forming Control
--------------------

These examples demonstrate grid-forming control. The example :doc:`/grid_examples/grid_forming/plot_gfm_rfpsc_13kva` uses a power-synchronization loop for synchronizing with the grid [#Har2020]_. In :doc:`/grid_examples/grid_forming/plot_gfm_obs_13kva`, disturbance-observer-based control is used [#Nur2024]_.

.. rubric:: References

.. [#Har2020] Harnefors, Rahman, Hinkkanen, Routimo, "Reference-feedforward power-synchronization control," IEEE Trans. Power Electron., 2020, https://doi.org/10.1109/TPEL.2020.2970991

.. [#Nur2024] Nurminen, Mourouvin, Hinkkanen, Kukkola, "Multifunctional grid-forming converter control based on a disturbance observer, "IEEE Trans. Power Electron., 2024, https://doi.org/10.1109/TPEL.2024.3433503



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates reference-feedforward power-synchronization control (RFPSC) of a converter connected to a weak grid.">

.. only:: html

  .. image:: /grid_examples/grid_forming/images/thumb/sphx_glr_plot_gfm_rfpsc_13kva_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_forming_plot_gfm_rfpsc_13kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA converter, RFPSC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a converter using disturbance-observer-based control in grid- forming mode. The converter output voltage and the active power are directly controlled, and grid synchronization is provided by the disturbance observer. A transparent current controller is included for current limitation.">

.. only:: html

  .. image:: /grid_examples/grid_forming/images/thumb/sphx_glr_plot_gfm_obs_13kva_thumb.png
    :alt:

  :ref:`sphx_glr_grid_examples_grid_forming_plot_gfm_obs_13kva.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">12.5-kVA converter, disturbance observer</div>
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
