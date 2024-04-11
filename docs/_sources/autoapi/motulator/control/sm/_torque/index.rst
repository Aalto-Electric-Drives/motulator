:orphan:

:py:mod:`motulator.control.sm._torque`
======================================

.. py:module:: motulator.control.sm._torque

.. autoapi-nested-parse::

   Torque characteristics for synchronous machines.

   This contains computation and plotting of torque characteristics for
   synchronous machines, including the MTPA and MTPV loci [#Mor1990]_. The methods
   can be used to define look-up tables for control and to analyze the
   characteristics. This version omits the magnetic saturation.

   .. rubric:: References

   .. [#Mor1990] Morimoto, Takeda, Hirasa, Taniguchi, "Expansion of operating
      limits for permanent magnet motor by current vector control considering
      inverter capacity," IEEE Trans. Ind. Appl., 1990,
      https://doi.org/10.1109/28.60058

   ..
       !! processed by numpydoc !!


Module Contents
---------------

Classes
~~~~~~~

.. autoapisummary::

   motulator.control.sm._torque.TorqueCharacteristics




.. py:class:: TorqueCharacteristics(par)


   
   Compute MTPA and MTPV loci based on the machine parameters.

   The magnetic saturation is omitted.

   :param par: Machine model parameters.
   :type par: ModelPars















   ..
       !! processed by numpydoc !!
   .. py:method:: torque(psi_s)

      
      Compute the torque as a function of the stator flux linkage.

      :param psi_s: Stator flux (Vs).
      :type psi_s: complex

      :returns: **tau_M** -- Electromagnetic torque (Nm).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: current(psi_s)

      
      Compute the stator current as a function of the stator flux linkage.

      :param psi_s: Stator flux linkage (Vs).
      :type psi_s: complex

      :returns: **i_s** -- Stator current (A).
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: flux(i_s)

      
      Compute the stator flux linkage as a function of the current.

      :param i_s: Stator current (A).
      :type i_s: complex

      :returns: **psi_s** -- Stator flux linkage (Vs).
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpa(abs_i_s)

      
      Compute the MTPA stator current angle.

      :param abs_i_s: Stator current magnitude (A).
      :type abs_i_s: float

      :returns: **beta** -- MTPA angle of the stator current vector (electrical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpv(abs_psi_s)

      
      Compute the MTPV stator flux angle.

      :param abs_psi_s: Stator flux magnitude (Vs).
      :type abs_psi_s: float

      :returns: **delta** -- MTPV angle of the stator flux vector (electrical rad).
      :rtype: float















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpv_current(abs_i_s)

      
      Compute the MTPV based on the current magnitude.

      This computes the MTPV based on the current magnitude, i.e., the
      intersection of the MTPV current locus and the current limit circle.
      This method is not necessary for computing the control look-up tables.
      It is used here to "cut" the MTPV characteristics at the desired
      current. Alternatively just a large enough maximum flux magnitude could
      be used.

      :param abs_i_s: Stator current magnitude (A).
      :type abs_i_s: float

      :returns: **i_s** -- MTPV stator current (A).
      :rtype: complex















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpa_locus(i_s_max, psi_s_min=None, N=20)

      
      Compute the MTPA locus.

      :param i_s_max: Maximum stator current magnitude (A) at which the locus is
                      computed.
      :type i_s_max: float
      :param psi_s_min: Minimum stator flux magnitude (Vs) at which the locus is computed.
      :type psi_s_min: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **psi_s** (*complex*) -- Stator flux (Vs).
                * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).
                * **abs_psi_s_vs_tau_M** (*callable*) -- Stator flux magnitude (Vs) as a function of the torque (Nm).
                * **i_sd_vs_tau_M** (*callable*) -- d-axis current (A) as a function of the torque (Nm).















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpv_locus(psi_s_max=None, i_s_max=None, N=20)

      
      Compute the MTPV locus.

      :param psi_s_max: Maximum stator flux magnitude (Vs) at which the locus is computed.
                        Either `psi_s_max` or `i_s_max` must be given.
      :type psi_s_max: float, optional
      :param i_s_max: Maximum stator current magnitude (A) at which the locus is
                      computed.
      :type i_s_max: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **psi_s** (*complex*) -- Stator flux (Vs).
                * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).
                * **tau_M_vs_abs_psi_s** (*interp1d object*) -- Torque (Nm) as a function of the flux magnitude (Vs).















      ..
          !! processed by numpydoc !!

   .. py:method:: current_limit(i_s_max, gamma1=np.pi, gamma2=0, N=20)

      
      Compute the current limit.

      :param i_s_max: Current limit (A).
      :type i_s_max: float
      :param gamma1: Starting angle (electrical rad). The default is 0.
      :type gamma1: float, optional
      :param gamma2: End angle (electrical rad). The defauls in pi.
      :type gamma2: float, optional
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **psi_s** (*complex*) -- Stator flux (Vs).
                * **i_s** (*complex*) -- Stator current (A).
                * **tau_M** (*float*) -- Electromagnetic torque (Nm).
                * **tau_M_vs_abs_psi_s** (*interp1d object*) -- Torque (Nm) as a function of the flux magnitude (Vs).















      ..
          !! processed by numpydoc !!

   .. py:method:: mtpv_and_current_limits(i_s_max, N=20)

      
      Merge the MTPV and current limits into a single interpolant.

      :param i_s_max: Current limit (A).
      :type i_s_max: float
      :param N: Amount of points. The default is 20.
      :type N: int, optional

      :returns: * *Bunch object with the following fields defined*
                * **tau_M_vs_abs_psi_s** (*interp1d object*) -- Torque (Nm) as a function of the flux magnitude (Vs).
                * **i_sd_vs_tau_M** (*interp1d object*) -- d-axis current (A) as a function of the torque (Nm).















      ..
          !! processed by numpydoc !!

   .. py:method:: plot_flux_loci(i_s_max, base, N=20)

      
      Plot the stator flux linkage loci.

      Per-unit quantities are used.

      :param i_s_max: Maximum current (A) at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!

   .. py:method:: plot_current_loci(i_s_max, base, N=20)

      
      Plot the current loci.

      Per-unit quantities are used.

      :param i_s_max: Maximum current (A) at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!

   .. py:method:: plot_torque_current(i_s_max, base, N=20)

      
      Plot torque vs. current characteristics.

      Per-unit quantities are used.

      :param i_s_max: Maximum current (A) at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!

   .. py:method:: plot_torque_flux(i_s_max, base, N=20)

      
      Plot torque vs. flux magnitude characteristics.

      Per-unit quantities are used.

      :param i_s_max: Maximum current (A) at which the loci are evaluated.
      :type i_s_max: float
      :param base: Base values.
      :type base: BaseValues
      :param N: Amount of points to be evaluated. The default is 20.
      :type N: int, optional















      ..
          !! processed by numpydoc !!


