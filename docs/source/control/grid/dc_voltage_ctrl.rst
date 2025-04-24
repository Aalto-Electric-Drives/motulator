DC-Bus Voltage Control
======================

A PI controller for the DC-bus voltage is implemented in the :class:`motulator.grid.control.DCBusVoltageController` class, whose base class is :class:`motulator.common.control.PIController`. In the following, the tuning of the DC-bus voltage controller is considered in the continuous-time domain.

Controller Structure
--------------------

In order to have a linear closed-loop system, it is convenient to use the DC-bus energy as the controlled variable, [#Hur2001]_

.. math::
	\hat W_\mathrm{dc} = \frac{1}{2}\hat{C}_\mathrm{dc} u_\mathrm{dc}^2 \qquad
    W_\mathrm{dc,ref} = \frac{1}{2}\hat{C}_\mathrm{dc} u_\mathrm{dc,ref}^2
    :label: dc_cap_energy

where :math:`\hat{C}_\mathrm{dc}` is the DC-bus capacitance estimate, :math:`u_\mathrm{dc}` is the measured DC-bus voltage, and :math:`u_\mathrm{dc,ref}` is the DC-bus voltage reference. Using these variables, the output power reference :math:`p_\mathrm{c,ref}` for the converter is obtained from a PI controller,

.. math::
    p_\mathrm{c,ref} = -k_\mathrm{p} (W_\mathrm{dc,ref} - \hat{W}_\mathrm{dc}) - \int k_\mathrm{i} (W_\mathrm{dc,ref} - \hat{W}_\mathrm{dc}) \mathrm{d} t
    :label: dc_voltage_ctrl

where :math:`k_\mathrm{p}` is the proportional gain and :math:`k_\mathrm{i}` is the integral gain. The negative signs appear since the positive converter output power decreases the DC-bus capacitor energy, see :eq:`capacitor`.

Closed-Loop System
------------------

For simplicity, the capacitance estimate being accurate, :math:`\hat{C}_\mathrm{dc} = C_\mathrm{dc}`. Furthermore, the converter is assumed to be lossless and its output power control ideal. The DC-bus energy balance is

.. math::
    \frac{\mathrm{d} W_\mathrm{dc}}{\mathrm{d} t} = p_\mathrm{dc} - p_\mathrm{c}
    :label: capacitor

where :math:`p_\mathrm{dc}` is the external power fed to the DC bus, :math:`p_\mathrm{c}` is the converter output power, and :math:`W_\mathrm{dc} = (C_\mathrm{dc}/2) u_\mathrm{dc}^2` is the energy stored in the DC-bus capacitor. The power :math:`p_\mathrm{dc}` is considered as an unknown disturbance.

In the Laplace domain, the closed-loop system resulting from :eq:`dc_voltage_ctrl` and :eq:`capacitor` is given by

.. math::
    W_\mathrm{dc}(s) = \frac{k_\mathrm{p} s + k_\mathrm{i}}{s^2 + k_\mathrm{p} s + k_\mathrm{i}} W_\mathrm{dc,ref}(s) + \frac{s}{s^2 + k_\mathrm{p} s + k_\mathrm{i}} p_\mathrm{dc}(s)
    :label: dc_voltage_ctrl_closed_loop

where it can be seen that :math:`W_\mathrm{dc} = W_\mathrm{dc,ref}` holds in the steady state. Furthermore, it can be shown that also :math:`u_\mathrm{dc} = u_\mathrm{dc,ref}` holds in the steady state (independently of the errors in the capacitance estimate :math:`\hat{C}`, since both reference and measured values in :eq:`dc_cap_energy` are scaled by the same estimate).

Gain Selection
--------------

Based on :eq:`dc_voltage_ctrl_closed_loop`, the gain selection

.. math::
    k_\mathrm{p} = 2\alpha_\mathrm{dc} \qquad
    k_\mathrm{i} = \alpha_\mathrm{dc}^2
    :label: dc_voltage_ctrl_gain_selection

results in the double real pole at :math:`s = -\alpha_\mathrm{dc}`. The closed-loop bandwidth is approximately :math:`\alpha_\mathrm{dc}`.

.. rubric:: References

.. [#Hur2001] Hur, Jung, Nam, "A fast dynamic DC-link power-balancing scheme for a PWM converter-inverter system," IEEE Trans. Ind. Electron., 2001, https://doi.org/10.1109/41.937412
