

.. _sphx_glr_drive_examples_obs_vhz:

Observer-Based V/Hz Control
---------------------------

These examples demonstrate observer-based V/Hz control for induction machines [#Tii2022a]_ and synchronous machines [#Tii2022b]_. The examples :doc:`/drive_examples/obs_vhz/plot_obs_vhz_ctrl_syrm_7kw` and :doc:`/drive_examples/obs_vhz/plot_obs_vhz_ctrl_pmsyrm_thor` also present the use of saturation models. The example :doc:`/drive_examples/obs_vhz/plot_obs_vhz_ctrl_pmsm_2kw_two_mass` demonstrates the use of a two-mass mechanics model. 

.. rubric:: References

.. [#Tii2022a] Tiitinen, Hinkkanen, Harnefors, "Stable and passive observer-based V/Hz control for induction motors," Proc. IEEE ECCE, Detroit, MI, Oct. 2022, https://doi.org/10.1109/ECCE50734.2022.9948057

.. [#Tii2022b] Tiitinen, Hinkkanen, Kukkola, Routimo, Pellegrino, Harnefors, "Stable and passive observer-based V/Hz control for synchronous Motors," Proc. IEEE ECCE, Detroit, MI, Oct. 2022, https://doi.org/10.1109/ECCE50734.2022.9947858



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive.">

.. only:: html

  .. image:: /drive_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_obs_vhz_plot_obs_vhz_ctrl_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW induction motor drive.">

.. only:: html

  .. image:: /drive_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_obs_vhz_plot_obs_vhz_ctrl_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a saturated 6.7-kW synchronous reluctance motor drive. The saturation is not taken into account in  the control method (only in the system model).">

.. only:: html

  .. image:: /drive_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_syrm_7kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_obs_vhz_plot_obs_vhz_ctrl_syrm_7kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, saturated</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive. The mechanical subsystem is modeled as a two-mass system. The resonance frequency of the mechanics is around 85 Hz. The mechanical parameters correspond to  [#Saa2015]_, except that the torsional damping is set to a smaller value in  this example.">

.. only:: html

  .. image:: /drive_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_pmsm_2kw_two_mass_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_obs_vhz_plot_obs_vhz_ctrl_pmsm_2kw_two_mass.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM, 2-mass mechanics</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a saturated 5-kW permanent-magnet synchronous reluctance motor. The flux maps of this example motor, known as THOR, are from the SyR-e project:">

.. only:: html

  .. image:: /drive_examples/obs_vhz/images/thumb/sphx_glr_plot_obs_vhz_ctrl_pmsyrm_thor_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_obs_vhz_plot_obs_vhz_ctrl_pmsyrm_thor.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">5-kW PM-SyRM, flux maps from SyR-e</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/obs_vhz/plot_obs_vhz_ctrl_pmsm_2kw
   /drive_examples/obs_vhz/plot_obs_vhz_ctrl_im_2kw
   /drive_examples/obs_vhz/plot_obs_vhz_ctrl_syrm_7kw
   /drive_examples/obs_vhz/plot_obs_vhz_ctrl_pmsm_2kw_two_mass
   /drive_examples/obs_vhz/plot_obs_vhz_ctrl_pmsyrm_thor

