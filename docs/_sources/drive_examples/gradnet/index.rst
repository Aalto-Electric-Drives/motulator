

.. _sphx_glr_drive_examples_gradnet:

GradNet Models
--------------

These examples use our physics-constrained neural-network framework for dynamic modeling of saturable synchronous machines, including spatial harmonics [#Li2026]_. The gradient network (GradNet) architecture is based on [#Cha2025]_. This folder contains measured datasets for two PM synchronous reluctance machines: a four-pole 5.6-kW machine (Baldor ECS101M0H7EF4) and a 10-pole 28-kW machine (Brusa HSM1.10.18.04). Additionally, a FEM dataset with spatial harmonics is provided for the 5.6-kW machine.

.. rubric:: References

.. [#Li2026] Li, Foissner, Martin, Piippo, Hinkkanen, "Gradient networks for universal magnetic modeling of synchronous machines," 2026, https://arxiv.org/abs/2602.14947

.. [#Cha2025] Chaudhari, Pranav, Moura, "Gradient networks," IEEE Trans. Signal Process., 2025, https://doi.org/10.1109/TSP.2024.3496692


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example trains a GradNet flux-linkage map for a four-pole 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) from a FEM dataset without spatial harmonics.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_train_6kw_pmsyrm_flux_map_fem_no_harm_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/train_6kw_pmsyrm_flux_map_fem_no_harm`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, train flux map, FEM data, no spatial harmonics</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example trains a GradNet current map for a four-pole 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) from a FEM dataset with spatial harmonics.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_train_6kw_pmsyrm_curr_map_fem_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/train_6kw_pmsyrm_curr_map_fem`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, train current map, FEM data with spatial harmonics</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates flux-vector control (FVC) of a 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) drive. GradNet models trained on FEM data are used for both the machine model and the control system. The machine model uses a GradNet current map with spatial harmonics, while the control system uses a GradNet flux map without spatial harmonics.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_plot_6kw_pmsyrm_gn_fvc_fem_harm_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/plot_6kw_pmsyrm_gn_fvc_fem_harm`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, GradNet from FEM data, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates flux-vector control (FVC) of a 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) drive. GradNet models trained on measured data without spatial harmonics are used for both the machine model and the control system.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_plot_6kw_pmsyrm_gn_fvc_meas_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/plot_6kw_pmsyrm_gn_fvc_meas`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, GradNet from measured data, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example trains a GradNet flux-linkage map for a 10-pole 28-kW PM synchronous reluctance machine (Brusa HSM1.10.18.04) from a measured dataset.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_plot_28kw_pmsyrm_flux_map_meas_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/plot_28kw_pmsyrm_flux_map_meas`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">28-kW PM-SyRM, train flux map, measured data</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example trains a GradNet flux-linkage map for a four-pole 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) from a measured dataset.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_plot_6kw_pmsyrm_flux_map_meas_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/plot_6kw_pmsyrm_flux_map_meas`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, train flux map, measured data</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example trains a GradNet current map for a four-pole 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) from a measured dataset.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_plot_6kw_pmsyrm_curr_map_meas_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/plot_6kw_pmsyrm_curr_map_meas`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, train current map, measured data</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example trains a GradNet current map for a 10-pole 28-kW PM synchronous reluctance machine (Brusa HSM1.10.18.04) from a measured dataset.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_plot_28kw_pmsyrm_curr_map_meas_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/plot_28kw_pmsyrm_curr_map_meas`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">28-kW PM-SyRM, train current map, measured data</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example trains a GradNet flux-linkage map for a four-pole 5.6-kW PM synchronous reluctance machine (ABB Baldor ECS101M0H7EF4) from a FEM dataset with spatial harmonics.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_plot_6kw_pmsyrm_flux_map_fem_harm_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/plot_6kw_pmsyrm_flux_map_fem_harm`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, train flux map, FEM data with spatial harmonics</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/gradnet/train_6kw_pmsyrm_flux_map_fem_no_harm
   /drive_examples/gradnet/train_6kw_pmsyrm_curr_map_fem
   /drive_examples/gradnet/plot_6kw_pmsyrm_gn_fvc_fem_harm
   /drive_examples/gradnet/plot_6kw_pmsyrm_gn_fvc_meas
   /drive_examples/gradnet/plot_28kw_pmsyrm_flux_map_meas
   /drive_examples/gradnet/plot_6kw_pmsyrm_flux_map_meas
   /drive_examples/gradnet/plot_6kw_pmsyrm_curr_map_meas
   /drive_examples/gradnet/plot_28kw_pmsyrm_curr_map_meas
   /drive_examples/gradnet/plot_6kw_pmsyrm_flux_map_fem_harm

