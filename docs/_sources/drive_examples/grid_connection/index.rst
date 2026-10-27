

.. _sphx_glr_drive_examples_grid_connection:

Grid Connection
---------------

These examples demonstrate machines and drives connected to the grid. The example :doc:`/drive_examples/grid_connection/plot_2kw_im_dol` simulates the direct-on-line start of an induction machine, which has no converter. In :doc:`/drive_examples/grid_connection/plot_2kw_im_diode_vhz` and :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_diode_cvc`, the drive is fed from a diode bridge, which has no control system. The examples :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_afe_cvc` and :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_afe_dvf_cvc` include an active front end with grid-following and dual-voltage-forming control, respectively, each with two independent control systems.


.. raw:: html

  <div id='sg-tag-list' class='sphx-glr-tag-list'></div>


.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates the direct-on-line start of a 2.2-kW induction machine (IM), whose stator is connected directly to a stiff grid. The machine drives a fan, whose load torque is proportional to the square of the speed. The model has no converter, so it is simulated without a control system.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_im_dol_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_im_dol`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, direct-on-line start</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates sensorless current-vector control (CVC) of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive, fed from a six-pulse diode bridge. The diode bridge is supplied from a stiff grid through an AC choke, and it has no control system.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_ipmsm_diode_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_diode_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, diode bridge, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates pure open-loop V/Hz control of a 2.2-kW induction machine (IM) drive, fed from a six-pulse diode bridge. The diode bridge is supplied from a stiff grid through an AC choke, and it has no control system.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_im_diode_vhz_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_im_diode_vhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, diode bridge, V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive whose active front end (AFE) uses dual-voltage-forming control [#Mou2025]_ and is connected to a weak grid. The grid-side converter regulates both the DC-bus voltage and its output voltage magnitude without a cascaded DC-bus voltage controller: the external power fed to the DC bus by the drive is estimated by a disturbance observer and fed forward. The machine-side converter uses sensorless current-vector control (CVC). The two control systems are independent: they have different sampling periods, and their carriers are not synchronized. The system model is otherwise the same as in /drive_examples/grid_connection/plot_2kw_ipmsm_afe_cvc, except for the weak grid.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_ipmsm_afe_dvf_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_afe_dvf_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, dual-voltage-forming AFE, CVC</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive equipped with an active front end (AFE), i.e., a grid-connected converter that controls the DC-bus voltage. The machine-side converter uses sensorless current-vector control (CVC), and the grid-side converter uses grid-following current-vector control with a DC-bus voltage controller. The two control systems are independent: they have different sampling periods, and their carriers are not synchronized.">

.. only:: html

  .. image:: /drive_examples/grid_connection/images/thumb/sphx_glr_plot_2kw_ipmsm_afe_cvc_thumb.png
    :alt:

  :doc:`/drive_examples/grid_connection/plot_2kw_ipmsm_afe_cvc`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, active front end, CVC</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/grid_connection/plot_2kw_im_dol
   /drive_examples/grid_connection/plot_2kw_ipmsm_diode_cvc
   /drive_examples/grid_connection/plot_2kw_im_diode_vhz
   /drive_examples/grid_connection/plot_2kw_ipmsm_afe_dvf_cvc
   /drive_examples/grid_connection/plot_2kw_ipmsm_afe_cvc

