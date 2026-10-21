# Reference Generation

## For Induction Machines

For induction machines, the {class}`motulator.drive.control.im.ReferenceGenerator` class implements a simple reference generation that can be used with flux-vector control. [Figure 1](fig:im_ref_gen) illustrates this method that consists of field weakening, current limitation (CL), and breakdown torque limitation.

For current limitation, the priority is given to the flux. The stator flux limit and the torque limit, respectively, are

```{math}
---
label: im_cl
---
    \psi_\mathrm{s}^\mathrm{cl} &= \hatLsgm \ismax + \hatabspsiR \\
    \tau_\mathrm{M}^\mathrm{cl} &= \frac{3\np}{2} \frac{\hatabspsiR}{\hatLsgm} \sqrt{(\abspsisref)^2 - (\psi_\mathrm{sd}^\mathrm{cl})^2}
```

where $\hatabspsiR$ is the rotor flux magnitude estimate and $\ismax$ is the maximum current. The torque limit is the torque at which the current magnitude reaches $\ismax$ for the stator flux reference $\abspsisref$, whose d-component in rotor flux coordinates is then

```{math}
---
label: im_cl_d
---
    \psi_\mathrm{sd}^\mathrm{cl} = \frac{(\abspsisref)^2 + \hatabspsiR^2 - (\hatLsgm \ismax)^2}{2\hatabspsiR}
```

The breakdown torque limit is

```{math}
---
label: im_bd_lim
---
    \tau_\mathrm{M}^\mathrm{b} = \frac{3\np}{2} \frac{(\abspsisref)^2}{2\hat{L}_\ell}
```

where the leakage inductance estimate can be expressed by means of the inverse-Γ parameters as $\hat L_\ell = [(\hatLM + \hatLsgm)/\hatLM]\hatLsgm$.

This reference generation is used in the {class}`motulator.drive.control.im.FluxVectorController` class, which implements flux-vector control for induction machines. See also the {doc}`/drive_examples/flux_vector/plot_2kw_im_sat_fvc` example.

```{figure} ../figs/im_ref_gen.svg
---
name: fig:im_ref_gen
class: only-light
width: 100%
align: center
alt: Reference generation for induction machines
---
*Figure 1:* Reference generation for induction machines, including field weakening, current limitation (CL), and breakdown torque limitation. The limited torque reference is denoted by $\overline{\tau}_\mathrm{M}^\mathrm{ref}$ and the rated stator flux by $\psi_\mathrm{s}^\mathrm{nom}$. The parameter $k_\mathrm{u}$ is the voltage utilization factor.
```

```{figure} ../figs/im_ref_gen.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Reference generation for induction machines
---
*Figure 1:* Reference generation for induction machines, including field weakening, current limitation (CL), and breakdown torque limitation.  The limited torque reference is denoted by $\overline{\tau}_\mathrm{M}^\mathrm{ref}$ and the rated stator flux by $\psi_\mathrm{s}^\mathrm{nom}$. The parameter $k_\mathrm{u}$ is the voltage utilization factor.
```

## For Synchronous Machines

For synchronous machines, the {class}`motulator.drive.control.sm.ReferenceGenerator` class implements the optimal reference generation that can be used both with current-vector control and flux-vector control. [Figure 2](fig:sm_ref_gen) illustrates this method, including field weakening, maximum-torque-per-ampere (MTPA), maximum-torque-per-volt (MTPV), and current limitation (CL) {cite}`Mey2006, Awa2018`. [Figure 3](fig:flux_vs_torque) shows an example of the optimal reference characteristics for a 5.6-kW permanent-magnet synchronous reluctance machine (PM-SyRM). See also the {ref}`mtpa_mtpv` document and the {doc}`/drive_examples/flux_vector/plot_6kw_pmsyrm_sat_fvc` example for more details.

Note that the reference generation of flux-vector control is simpler than that of current-vector control, since the two-dimensional transformation to the current reference is avoided. Here, the current reference is driven toward the flux and torque references by a dynamic tracking law with the bandwidth $\alpha_\mathrm{ref}$ {cite}`Sar2026`, while other approaches such as a two-dimensional lookup table could be used.

Instead of the precomputed lookup tables, the optimal references can also be generated online {cite}`Sar2026`. The {class}`motulator.drive.control.sm.ReferenceGeneratorOnline` class tracks the current vectors that correspond to the torque reference on the MTPA locus and to the flux reference at the MTPV limit and on the current-limit circle by similar dynamic tracking laws, using the flux map and the incremental inductances of the machine model. The flux and torque references are obtained from these operating points in the same way as from the lookup tables. The online reference generation is selected by setting `online_ref=True` in the configuration of the current-vector or flux-vector controller, see the example {doc}`/drive_examples/current_vector/plot_6kw_pmsyrm_sat_cvc`.

```{figure} ../figs/sm_ref_gen.svg
---
name: fig:sm_ref_gen
class: only-light
width: 100%
align: center
alt: Reference generation for synchronous machines
---
*Figure 2:* Optimal reference generation for synchronous machines, including field weakening, maximum-torque-per-ampere (MTPA), maximum-torque-per-volt (MTPV), and current limitation (CL). These are single-dimensional lookup tables, see the example in [Figure 3](fig:flux_vs_torque). Two-dimensional transformation to the current reference is needed only in the case of current-vector control.
```

```{figure} ../figs/sm_ref_gen.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Reference generation for synchronous machines
---
*Figure 2:* Optimal reference generation for synchronous machines, including field weakening, maximum-torque-per-ampere (MTPA), maximum-torque-per-volt (MTPV), and current limitation (CL). These are single-dimensional lookup tables, see the example in [Figure 3](fig:flux_vs_torque). Two-dimensional transformation to the current reference is needed only in the case of current-vector control.
```

```{figure} ../figs/flux_vs_torque.svg
---
name: fig:flux_vs_torque
class: only-light
width: 100%
align: center
alt: Example of optimal reference characteristics
---
*Figure 3:* Example of optimal reference characteristics for a 5.6-kW PM-SyRM.
```

```{figure} ../figs/flux_vs_torque.svg
---
class: invert-colors-dark only-dark
width: 100%
align: center
alt: Example of optimal reference characteristics
---
*Figure 3:* Example of optimal reference characteristics for a 5.6-kW PM-SyRM.
```
