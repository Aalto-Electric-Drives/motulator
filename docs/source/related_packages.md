# Related Packages

The packages listed on this page extend *motulator* with additional models, control methods, and tools. Each package is maintained in its own repository, separately from *motulator*, and its issues are handled there.

## Packages

<!-- One row per package: name linked to the repository, one-sentence description,
     and the latest motulator release the package has been tested with. -->

| Package                                                                               | Description                                                                                                    | Tested with |
| ------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------------------------------- | ----------- |
| [motulator-multiphase](https://github.com/Aalto-Electric-Drives/motulator-multiphase) | Multi-three-phase machine drives based on the decoupled multi-stator transformation.                           | 0.15.0      |
| [motulator-impedance](https://github.com/Aalto-Electric-Drives/motulator-impedance)   | Mechanical impedance of machine drives and output admittance of grid converters, measured by frequency sweeps. | 0.15.0      |
| [motulator-export](https://github.com/Aalto-Electric-Drives/motulator-export)         | Export of drive and grid converter systems to PLECS and Simulink, with the control algorithms ported to C.     | 0.15.2      |

## Listing a Package

A package can be listed if it is open source, installable with `pip`, and has a README with at least one runnable example. To list it, open an [issue](https://github.com/Aalto-Electric-Drives/motulator/issues) or a pull request that adds a row to the table. The maintainers of a package keep the tested release up to date; packages that no longer work or are no longer maintained may be removed from the list.

## Developing a Package

The interfaces of *motulator* may change in any release, so a package should use only the public interfaces, including the hooks documented for subclasses, and run its tests against the latest release. The recommended name is `motulator-<name>` (imported as `motulator_<name>`), with the code in `motulator_<name>/`, tests in `tests/`, and examples in `examples/`. If a method cannot be implemented through the public interfaces, propose the missing hook (e.g., an overridable method or a configuration field) in a small pull request to *motulator*, separate from the package. Such a hook must be generic and must not carry the method itself.
