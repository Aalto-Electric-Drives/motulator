# V/Hz Control

In V/Hz control, the machine is driven at the frequency given by the speed reference, without a speed controller. Two methods are available: open-loop V/Hz control for induction machines and observer-based V/Hz control for synchronous machines {cite}`Tii2025a` and induction machines {cite}`Tii2025b`. Both are used with the {class}`~motulator.drive.control.sm.VHzControlSystem` class, which limits the rate of change of the speed reference.

## Open-Loop V/Hz Control

In open-loop V/Hz control, the stator voltage reference is given by a V/Hz curve as a function of the stator angular frequency $\omegas = \np\omegaMref$, where $\omegaMref$ is the rate-limited speed reference. The reference is given in coordinates rotating at $\omegas$. The linear V/Hz curve is

```{math}
---
label: vhz_linear
---
    \usref = \jj\omegas\psi_\mathrm{s}^\mathrm{nom}
```

where $\psi_\mathrm{s}^\mathrm{nom}$ is the nominal stator flux linkage. In the steady state, the stator voltage equation in {eq}`im_voltages` is $\us = \Rs\is + \jj\omegas\psis$ in these coordinates. Hence, the curve {eq}`vhz_linear` gives the nominal stator flux linkage, if the resistive voltage drop is negligible. At low frequencies, the resistive voltage drop reduces the flux linkage. At no load, the stator current is $\is = \psis/\Ls$, where $\Ls$ is the stator inductance, and the curve

```{math}
---
label: vhz_no_load
---
    \usref = \left(\frac{\Rs}{\Ls} + \jj\omegas\right)\psi_\mathrm{s}^\mathrm{nom}
```

gives the nominal stator flux linkage at all frequencies. The slip is not compensated for, i.e., the rotor speed deviates from its reference depending on the load.

The {class}`motulator.drive.control.im.OpenLoopVHzController` class takes the V/Hz curve as a function. See the examples {doc}`/drive_examples/vhz/plot_2kw_im_lc_vhz` and {doc}`/drive_examples/grid_connection/plot_2kw_im_diode_vhz`.

## Observer-Based V/Hz Control

[Figure 1](fig:vhz_ctrl) shows the structure of observer-based V/Hz control, which is available in the {class}`motulator.drive.control.sm.ObserverBasedVHzController` class for synchronous machines and in the {class}`motulator.drive.control.im.ObserverBasedVHzController` class for induction machines. The structure is that of flux-vector control, see {doc}`/control/drive/flux_vector_ctrl`, with the following differences:

- The observer does not estimate the speed. The rate-limited speed reference is used in its place, $\hatomegam = \omegamref$, see {doc}`/control/drive/observers`.

- The torque reference is not given by a speed controller, but it is the low-pass-filtered torque estimate

  ```{math}
  ---
  label: vhz_tauMref
  ---
      \tauMref = \frac{\alpha_\mathrm{f}}{s + \alpha_\mathrm{f}}\hattauM
  ```

  where $\alpha_\mathrm{f}$ is the bandwidth of the filter. The reference generation limits this reference, see {doc}`/control/drive/reference_gen`.

The torque reference thus equals the torque estimate in the steady state, and the torque control error is the high-pass-filtered torque estimate with the opposite sign, $\tauMref - \hattauM = -[s/(s + \alpha_\mathrm{f})]\hattauM$. The stator flux magnitude is controlled as in flux-vector control.

```{figure} ../figs/vhz_ctrl.svg
---
name: fig:vhz_ctrl
class: only-light
width: 100%
align: center
alt: Observer-based V/Hz control
---
*Figure 1:* Observer-based V/Hz control. For induction machines, the reference generation uses the estimates $\hatomegas$ and $\hatabspsiR$ from the observer instead of $\omegamref$. The flux and torque controller uses the stator flux linkage and torque estimates shown in the figure as well as other estimates from the observer, such as the stator current and the angular speed of the coordinate system. The PWM realizes the limited voltage reference $\ussreflim$ in stationary coordinates, which the observer gets with the computational delay of one sampling period.
```

```{figure} ../figs/vhz_ctrl.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Observer-based V/Hz control
---
*Figure 1:* Observer-based V/Hz control. For induction machines, the reference generation uses the estimates $\hatomegas$ and $\hatabspsiR$ from the observer instead of $\omegamref$. The flux and torque controller uses the stator flux linkage and torque estimates shown in the figure as well as other estimates from the observer, such as the stator current and the angular speed of the coordinate system. The PWM realizes the limited voltage reference $\ussreflim$ in stationary coordinates, which the observer gets with the computational delay of one sampling period.
```

See the examples in {doc}`/drive_examples/vhz/index`.
