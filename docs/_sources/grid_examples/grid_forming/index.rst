

.. _sphx_glr_grid_examples_grid_forming:

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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a converter using disturbance-observer-based control in grid-forming mode. The converter output voltage and the active power are directly controlled, and grid synchronization is provided by the disturbance observer. A transparent current controller is included for current limitation.">

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

   /grid_examples/grid_forming/plot_gfm_rfpsc_13kva
   /grid_examples/grid_forming/plot_gfm_obs_13kva

