

.. _sphx_glr_drive_examples_vhz:

V/Hz Control
------------

These examples demonstrate observer-based V/Hz control for synchronous machines [#Tii2025aa]_ and induction machines [#Tii2025bb]_. The example :doc:`/drive_examples/vhz/plot_obs_vhz_pmsm_2kw_two_mass` demonstrates the use of a two-mass mechanics model. Furthermore, the examples :doc:`/drive_examples/vhz/plot_vhz_im_2kw` and :doc:`/drive_examples/vhz/plot_vhz_im_2kw_lc` show operation of an induction machine under pure open-loop V/Hz control with a diode front-end rectifier and with an LC filter, respectively.

.. rubric:: References

.. [#Tii2025aa] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025bb] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, in press



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW induction motor.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_obs_vhz_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_obs_vhz_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_obs_vhz_pmsm_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_obs_vhz_pmsm_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="A diode bridge, stiff three-phase grid, and a DC link is modeled. The control system is configured as pure open-loop V/Hz control.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_vhz_im_2kw_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_vhz_im_2kw.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, diode bridge</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a saturated 6.7-kW synchronous reluctance motor drive. The control method uses constant inductances.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_obs_vhz_syrm_7kw_sat_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_obs_vhz_syrm_7kw_sat.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW SyRM, saturated</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates open-loop V/Hz control of a 2.2-kW induction machine drive equipped with an output LC filter.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_vhz_im_2kw_lc_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_vhz_im_2kw_lc.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW induction motor, LC filter</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz control of a 2.2-kW PMSM drive. The mechanical subsystem is modeled as a two-mass system. The resonance frequency of the mechanics is around 85 Hz. The mechanical parameters correspond to [#Saa2015]_, except that the torsional damping is set to a smaller value in this example.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_obs_vhz_pmsm_2kw_two_mass_thumb.png
    :alt:

  :ref:`sphx_glr_drive_examples_vhz_plot_obs_vhz_pmsm_2kw_two_mass.py`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW PMSM, 2-mass mechanics</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/vhz/plot_obs_vhz_im_2kw
   /drive_examples/vhz/plot_obs_vhz_pmsm_2kw
   /drive_examples/vhz/plot_vhz_im_2kw
   /drive_examples/vhz/plot_obs_vhz_syrm_7kw_sat
   /drive_examples/vhz/plot_vhz_im_2kw_lc
   /drive_examples/vhz/plot_obs_vhz_pmsm_2kw_two_mass

