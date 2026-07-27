

.. _sphx_glr_drive_examples_gradnet:

GradNet Models
--------------

These examples demonstrate a physics-constrained neural network framework for dynamic modeling of saturable synchronous machines, including spatial harmonics. The gradient networks are based on [#Cha2025]_. A 5.6-kW PM-SyRM provides the dataset. Both the control and plant models are parameterized by neural networks.

.. rubric:: References

.. [#Cha2025] Chaudhari, Pranav, Moura, "Gradient Networks," IEEE Trans. Signal Process., vol. 73, pp. 324-339, 2025, doi: 10.1109/TSP.2024.3496692.


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

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates flux-vector control (FVC) of a 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) drive. GradNet saturation models, trained on the FEM dataset with spatial harmonics, are used.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_plot_6kw_pmsyrm_gn_fvc_fem_harm_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/plot_6kw_pmsyrm_gn_fvc_fem_harm`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, GradNet from FEM data, FVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates flux-vector control (FVC) of a 5.6-kW PM synchronous reluctance machine (Baldor ECS101M0H7EF4) drive. GradNet saturation models, trained on the measured dataset without spatial harmonics, are used.">

.. only:: html

  .. image:: /drive_examples/gradnet/images/thumb/sphx_glr_plot_6kw_pmsyrm_gn_fvc_meas_thumb.png
    :alt:

  :doc:`/drive_examples/gradnet/plot_6kw_pmsyrm_gn_fvc_meas`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5.6-kW PM-SyRM, GradNet from measured data, FVC</div>
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
   /drive_examples/gradnet/plot_6kw_pmsyrm_flux_map_meas
   /drive_examples/gradnet/plot_6kw_pmsyrm_curr_map_meas
   /drive_examples/gradnet/plot_6kw_pmsyrm_flux_map_fem_harm

