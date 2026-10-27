# Three-Phase Source

A model for an ideal three-phase source is implemented in the {class}`motulator.common.model.ThreePhaseSource` class. Typically, this model is used to represent the grid voltage source, which feeds a grid converter or a diode bridge through an AC filter, or a machine connected directly to the grid, see {class}`motulator.drive.model.GridConnectedMachine`. The voltage space vector is calculated as a combination of a positive-sequence component and (optionally) a negative-sequence components as

```{math}
---
label: grid_voltage_vector
---
\frac{\D\thetag}{\D t} &= \omegag \\
\egs &= e_\mathrm{g+}\e^{\jj(\thetag + \phi_+)} + e_\mathrm{g-}\e^{-\jj(\thetag + \phi_-)}
```

where $e_\mathrm{g+}$ and $e_\mathrm{g-}$ are the magnitudes of the positive-sequence and negative-sequence components, respectively, and $\phi_+$ and $\phi_-$ are the positive-sequence and negative-sequence phase shifts, respectively. The angle $\thetag$ is obtained by integrating the angular frequency $\omegag$. All parameters can be given as time-varying functions to simulate various fault conditions.

For measuring the output admittance of the converter, the {class}`motulator.grid.model.ThreePhaseSourceWithSignalInjection` class adds the excitation $\mathbf{u}_\mathrm{e}\cos(2\pi f_\mathrm{e}t)$ to the positive-sequence component in the synchronous coordinates of the source, see the package [motulator-impedance](https://github.com/Aalto-Electric-Drives/motulator-impedance).
