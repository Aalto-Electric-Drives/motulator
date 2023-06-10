:orphan:

:py:mod:`motulator.model.sm._flux_maps`
=======================================

.. py:module:: motulator.model.sm._flux_maps

.. autoapi-nested-parse::

   Import and plot flux maps from the SyR-e project.

   ..
       !! processed by numpydoc !!


Module Contents
---------------


Functions
~~~~~~~~~

.. autoapisummary::

   motulator.model.sm._flux_maps.import_syre_data
   motulator.model.sm._flux_maps.plot_flux_map
   motulator.model.sm._flux_maps.plot_torque_map
   motulator.model.sm._flux_maps.plot_flux_vs_current
   motulator.model.sm._flux_maps.downsample_flux_map
   motulator.model.sm._flux_maps.invert_flux_map



.. py:function:: import_syre_data(fname, add_negative_q_axis=True)

   
   Import a flux map from the MATLAB data file in the SyR-e format.

   For more information on the SyR-e project and the MATLAB file format,
   please visit:

       https://github.com/SyR-e/syre_public

   The imported data is converted to the PMSM coordinate convention, in which
   the PM flux is along the d axis.

   :param fname: MATLAB file name.
   :type fname: str
   :param add_negative_q_axis: Adds the negative q-axis data based on the symmetry.
   :type add_negative_q_axis: bool, optional

   :returns: * *Bunch object with the following fields defined*
             * **i_s** (*complex ndarray*) -- Stator current data (A).
             * **psi_s** (*complex ndarray*) -- Stator flux linkage data (Vs).
             * **tau_M** (*ndarray*) -- Torque data (Nm).

   .. rubric:: Notes

   Some example data files (including THOR.mat) are available in the SyR-e
   repository, licensed under the Apache License, Version 2.0.















   ..
       !! processed by numpydoc !!

.. py:function:: plot_flux_map(data)

   
   Plot the flux linkage as function of the current.

   :param data: Flux map data.
   :type data: Bunch















   ..
       !! processed by numpydoc !!

.. py:function:: plot_torque_map(data)

   
   Plot the torque as function of the current.

   :param data: Flux map data.
   :type data: Bunch















   ..
       !! processed by numpydoc !!

.. py:function:: plot_flux_vs_current(data)

   
   Plot the flux vs. current characteristics.

   :param data: Flux map data.
   :type data: Bunch















   ..
       !! processed by numpydoc !!

.. py:function:: downsample_flux_map(data, N_d=32, N_q=32)

   
   Downsample the flux map by means of linear interpolation.

   :param data: Flux map data.
   :type data: Bunch
   :param N_d: Number of interpolated samples in the d axis. The default is 32.
   :type N_d: int, optional
   :param N_q: Number of interpolated samples in the q axis. The default is 32.
   :type N_q: int, optional

   :returns: * *Bunch object with the following fields defined*
             * **i_s** (*complex ndarray, shape (N_d, N_q)*) -- Stator current data (A).
             * **psi_s** (*complex ndarray, shape (N_d, N_q)*) -- Stator flux linkage data (Vs).
             * **tau_M** (*ndarray, shape (N_d, N_q)*) -- Torque data (Nm).















   ..
       !! processed by numpydoc !!

.. py:function:: invert_flux_map(data, N_d=32, N_q=32)

   
   Compute the inverse flux map by means of linear interpolation.

   :param data: Flux map data.
   :type data: Bunch
   :param N_d: Number of interpolated samples in the d axis. The default is 32.
   :type N_d: int, optional
   :param N_q: Number of interpolated samples in the q axis. The default is 32.
   :type N_q: int, optional

   :returns: * *Bunch object with the following fields defined*
             * **psi_s** (*complex ndarray, shape (N_d, N_q)*) -- Stator flux linkage data (Vs).
             * **i_s** (*complex ndarray, shape (N_d, N_q)*) -- Stator current data (A).
             * **tau_M** (*ndarray, shape (N_d, N_q)*) -- Torque data (Nm).















   ..
       !! processed by numpydoc !!

