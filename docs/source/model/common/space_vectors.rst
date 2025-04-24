Space Vectors
=============

The system models in *motulator* apply peak-valued complex space vectors, marked with boldface in the following equations [#Hin2024]_. As an example, consider the space vector of the stator current,

.. math::
	\boldsymbol{i}^\mathrm{s}_\mathrm{s} &= \frac{2}{3}\left(i_\mathrm{sa} + i_\mathrm{sb}\mathrm{e}^{\mathrm{j}2\pi/3} + i_\mathrm{sc}\mathrm{e}^{\mathrm{j} 4\pi/3}\right) \\
   &= i_{\mathrm{s}\alpha} + \mathrm{j} i_{\mathrm{s}\beta}
   :label: space_vector

where :math:`i_\mathrm{sa}`, :math:`i_\mathrm{sb}`, and :math:`i_\mathrm{sc}` are the phase currents, which may vary freely in time. In this notation, the subscript s refers to the stator quantities and the superscript s refers to the stationary coordinates. In code, this space vector is denoted by :code:`i_s_ab`.

The space vector does not include the zero-sequence component, which is defined as

.. math::
	i_0 = \frac{1}{3}\left(i_\mathrm{a} + i_\mathrm{b} + i_\mathrm{c}\right)
   :label: zero_sequence

Even though the zero-sequence voltage exists at the output of typical converters, there is no path for the zero-sequence current to flow (i.e., :math:`i_0 = 0`), if the three-phase system is delta-connected or its star point is not connected. Consequently, the zero-sequence voltage cannot produce power or torque.

The space vector can be transformed to other coordinates as

.. math::
	\boldsymbol{i}_\mathrm{s} = \mathrm{e}^{-\mathrm{j}\vartheta_\mathrm{k}}\boldsymbol{i}^\mathrm{s}_\mathrm{s}
   :label: coordinate_transformation

where :math:`\vartheta_\mathrm{k}` is the angle of the new coordinate system with respect to stationary coordinates. The coordinate system can be fixed to a physical rotor or it can defined by a control system (typically fixed to some estimated quantity). The components in rotor coordinates are traditionally denoted according to :math:`\boldsymbol{i}_\mathrm{s} = i_\mathrm{sd} + \mathrm{j} i_\mathrm{sq}`. Hence, in code, the current vector in rotor coordinates is denoted by :code:`i_s_dq`. The vector in controller coordinates is simply denoted by :code:`i_s` in code. The notation for other space vectors is similar.

The space vector transformation in :eq:`space_vector` is implemented in the function :func:`motulator.common.utils.abc2complex` and its inverse transformation in the function :func:`motulator.common.utils.complex2abc`.

.. rubric:: References

.. [#Hin2024] Hinkkanen,  Harnefors, Kukkola, "Fundamentals of Electric Machine Drives," lecture notes, 2024, https://doi.org/10.5281/zenodo.10609166
