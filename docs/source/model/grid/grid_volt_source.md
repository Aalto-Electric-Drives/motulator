# Three-Phase Source

A model for an ideal three-phase source is implemented in the {class}`motulator.grid.model.ThreePhaseSource` class. Typically, this model is used to represent the grid voltage source. The voltage space vector is calculated as a combination of a positive-sequence component and (optionally) a negative-sequence components as

```{math}
---
label: grid_voltage_vector
---
\frac{\D\thetag}{\D t} &= \omegag \\
\egs &= e_\mathrm{g+}\e^{\jj(\thetag + \phi_+)} + e_\mathrm{g-}\e^{-\jj(\thetag + \phi_-)}
```

where $e_\mathrm{g+}$ and $e_\mathrm{g-}$ are the magnitudes of the positive-sequence and negative-sequence components, respectively, and $\phi_+$ and $\phi_-$ are the positive-sequence and negative-sequence phase shifts, respectively. The angle $\thetag$ is obtained by integrating the angular frequency $\omegag$. All parameters can be given as time-varying functions to simulate various fault conditions.

For identifying the output admittance of the converter, the {class}`motulator.grid.model.ThreePhaseSourceWithSignalInjection` class adds the excitation $\mathbf{u}_\mathrm{e}\cos(2\pi f_\mathrm{e}t)$ to the positive-sequence component in the synchronous coordinates of the source. The identification is run by {func}`motulator.grid.utils.run_identification`: the system is first simulated to the operating point, after which the PCC voltage is imposed by a stiff source and small-signal excitations are injected in the d- and q-axes. See the examples {doc}`/grid_examples/identification/plot_10kva_gfl_identification` and {doc}`/grid_examples/identification/plot_13kva_do_gfm_identification`.
