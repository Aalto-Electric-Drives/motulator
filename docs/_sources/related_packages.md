# Related Packages

The packages listed on this page extend *motulator* with additional models, control methods, and tools. Their authors maintain them and answer the issues in their own repositories; the maintainers of *motulator* do not review, test, or support them.

## Packages

<!-- One row per package: name linked to the repository, one-sentence description,
     and the latest compatible motulator release. -->

| Package | Description | Compatible with |
| ------- | ----------- | --------------- |
|         |             |                 |

## Listing a Package

A package is listed if it meets the following conditions:

- It is published on PyPI under a name starting with `motulator-` (e.g., `motulator-flyingstart`, imported as `motulator_flyingstart`) and under an open-source license.
- It uses only the public interfaces of *motulator*, including the hooks documented for subclasses, and no private modules, and it declares the compatible releases in its dependencies, e.g., `motulator>=0.13,<0.14`.
- It has automated tests that run against the declared releases and a README or documentation with at least one runnable example.

To list a package, open an [issue](https://github.com/Aalto-Electric-Drives/motulator/issues) titled "Related package: `<name>`" with a link to the repository and a one-sentence description. A package that does not work with the latest release is marked as such in the table, and it is removed if it has not been updated within the two following minor releases or at the request of its authors.

## Developing a Package

A package has the usual layout: `pyproject.toml`, the code in `src/motulator_<name>/`, tests in `tests/`, and examples in `examples/`, with a continuous-integration workflow that runs the tests against the latest release of *motulator*. If a method cannot be implemented through the public interfaces, propose the missing hook (e.g., an overridable method or a configuration field) in a small pull request to *motulator*, separate from the package. Such a hook must be generic and must not carry the method itself.
