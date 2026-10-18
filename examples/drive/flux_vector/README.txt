Flux-Vector Control
-------------------

These examples demonstrate flux-vector control (FVC) of electric machine drives [#Pel2009]_. In the implemented control system, decoupling between the stator flux and torque channels are used according to [#Awa2019]_. Furthermore, the stator flux magnitude and the electromagnetic torque are selected as controllable variables. The implementations correspond to [#Tii2025a]_ for synchronous machines and [#Tii2025b]_ for induction machines. The magnetic saturation is modeled and taken into account in control. The examples with GradNet magnetic models [#Li2026]_, based on gradient networks [#Cha2025]_, which model the current or flux linkage map as the gradient of the magnetic energy or co-energy, use models trained from measured and FEM data with the related package `motulator-gradnet <https://aalto-electric-drives.github.io/motulator-gradnet/>`_.

.. rubric:: References

.. [#Pel2009] Pellegrino, Armando, Guglielmi, "Direct flux field-oriented control of IPM drives with variable DC link in the field-weakening region," IEEE Trans. Ind. Appl., 2009, https://doi.org/10.1109/TIA.2009.2027167

.. [#Awa2019] Awan, Hinkkanen, Bojoi, Pellegrino, "Stator-flux-oriented control of synchronous motors: A systematic design procedure," IEEE Trans. Ind. Appl., 2019, https://doi.org/10.1109/TIA.2019.2927316

.. [#Tii2025a] Tiitinen, Hinkkanen, Harnefors, "Design framework for sensorless control of synchronous machine drives," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2024.3429650

.. [#Tii2025b] Tiitinen, Hinkkanen, Harnefors, "Sensorless flux-vector control framework: An extension for induction machines," IEEE Trans. Ind. Electron., 2025, https://doi.org/10.1109/TIE.2025.3559958

.. [#Li2026] Li, Foissner, Martin, Piippo, Hinkkanen, "Gradient networks for universal magnetic modeling of synchronous machines," 2026, https://arxiv.org/abs/2602.14947

.. [#Cha2025] Chaudhari, Pranav, Moura, "Gradient networks," IEEE Trans. Signal Process., 2025, https://doi.org/10.1109/TSP.2024.3496692
