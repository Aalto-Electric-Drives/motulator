

.. _sphx_glr_drive_examples_vhz:

V/Hz Control
------------

These examples demonstrate observer-based V/Hz (O-V/Hz) control for synchronous machines [#Tii2025aa]_ and induction machines [#Tii2025bb]_. The example :doc:`/drive_examples/vhz/plot_2kw_ipmsm_2mass_ovhz` demonstrates the use of a two-mass mechanics model. Furthermore, the examples :doc:`/drive_examples/vhz/plot_2kw_im_diode_vhz` and :doc:`/drive_examples/vhz/plot_2kw_im_lc_vhz` show operation of an induction machine under pure open-loop V/Hz control with a diode front-end rectifier and with an LC filter, respectively.

.. rubric:: References

.. [#Tii2025aa] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025bb] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2025.3559958



.. raw:: html

    <div class="sphx-glr-thumbnails">

.. thumbnail-parent-div-open

.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz (O-V/Hz) control of a 2.2-kW induction machine (IM).">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_im_ovhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_im_ovhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, O-V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz (O-V/Hz) control of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_ipmsm_ovhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_ipmsm_ovhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, O-V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates pure open-loop V/Hz control of a 2.2-kW induction machine (IM) drive. A diode bridge, stiff three-phase grid, and a DC link is modeled.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_im_diode_vhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_im_diode_vhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, diode bridge, V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz (O-V/Hz) control of a 6.7-kW synchronous reluctance machine (SyRM) drive. The magnetic saturation is included in the machine model, while the control system uses constant parameters.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_7kw_syrm_sat_ovhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_7kw_syrm_sat_ovhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">6.7-kW saturated SyRM, O-V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates open-loop V/Hz control of a 2.2-kW induction machine (IM) drive equipped with an output LC filter.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_im_lc_vhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_im_lc_vhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IM, LC filter, V/Hz control</div>
    </div>


.. raw:: html

    <div class="sphx-glr-thumbcontainer" tooltip="This example simulates observer-based V/Hz (O-V/Hz) control of a 2.2-kW interior permanent-magnet synchronous machine (IPMSM) drive. The mechanical subsystem is modeled as a two-mass system. The resonance frequency of the mechanics is around 85 Hz. The mechanical parameters correspond to [#Saa2015]_, except that the torsional damping is set to a smaller value in this example.">

.. only:: html

  .. image:: /drive_examples/vhz/images/thumb/sphx_glr_plot_2kw_ipmsm_2mass_ovhz_thumb.png
    :alt:

  :doc:`/drive_examples/vhz/plot_2kw_ipmsm_2mass_ovhz`

.. raw:: html

      <div class="sphx-glr-thumbnail-title">2.2-kW IPMSM, 2-mass mechanics, O-V/Hz control</div>
    </div>


.. thumbnail-parent-div-close

.. raw:: html

    </div>


.. toctree::
   :hidden:

   /drive_examples/vhz/plot_2kw_im_ovhz
   /drive_examples/vhz/plot_2kw_ipmsm_ovhz
   /drive_examples/vhz/plot_2kw_im_diode_vhz
   /drive_examples/vhz/plot_7kw_syrm_sat_ovhz
   /drive_examples/vhz/plot_2kw_im_lc_vhz
   /drive_examples/vhz/plot_2kw_ipmsm_2mass_ovhz

