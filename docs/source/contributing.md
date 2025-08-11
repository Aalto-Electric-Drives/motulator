# Contributing

## Style Guide

We follow the [Style Guide for Python Code PEP8](https://peps.python.org/pep-0008/). Furthermore, we use the [Ruff code formatter](https://github.com/astral-sh/ruff) and [Pyright static type checker](https://github.com/microsoft/pyright), both configured in the [pyproject.toml](https://github.com/Aalto-Electric-Drives/motulator/blob/main/pyproject.toml) file.
Public methods and classes should have proper [docstrings](https://peps.python.org/pep-0257/), formatted according to the [NumPy docstring guide](https://numpydoc.readthedocs.io/en/latest/format.html).

[Sphinx](https://www.sphinx-doc.org) is used to auto-generate documentation. We use [Sphinx-Gallery](https://sphinx-gallery.github.io/stable/index.html) to automatically generate the {ref}`examples` from the scripts in the [examples](https://github.com/Aalto-Electric-Drives/motulator/tree/main/examples) folder. The format of the example scripts is essential for their proper rendering in the documentation (see the existing example scripts).

## Install Optional Dependencies

We recommend to install the `dev` dependencies (see also {doc}`/installation`) as well as the VS Code extensions recommended in the [extensions.json](https://github.com/Aalto-Electric-Drives/motulator/blob/main/.vscode/extensions.json) file. These extensions ensure consistent code style, proper type checking, and an efficient development workflow when contributing to this project.

If you aim to work with the documentation, install also the `doc` dependencies. For previewing the documentation in VS Code, you can install the [Esbonio extension](https://marketplace.visualstudio.com/items?itemName=swyddfa.esbonio). Alternatively, you can build the documentation locally using the command `make html` in the `docs` directory. The documentation will be built in the `docs/build` directory.

## Figures

Figures in the documentation are created in SVG format using [Inkscape](https://inkscape.org/). For mathematical equations in figures, we use the [TexText extension](https://textext.github.io/textext/) for Inkscape. This allows writing editable equations in LaTeX syntax that are then rendered within the SVG figures. The preamble file can be found in the repository:

```
docs/source/inkscape_textext_preamble.sty
```

## Submitting a Bug Report or a Feature Request

If you have found bugs or have feature requests, please submit them to the [issue tracker](https://github.com/Aalto-Electric-Drives/motulator/issues). If you are reporting a bug, please include the following in your report:

- Name and version of your operating system.
- Any relevant details about your local setup, such as the Python interpreter version and installed libraries.
- A short reproducible code snippet, so that anyone can reproduce the bug easily.
- Try to be specific about what functions are involved and the shape of the data, especially when including a code snippet is not feasible.
- Provide the full traceback to any exceptions that were raised.

## Workflow for Contributing

When contributing code, please follow this workflow:

1. Create a new [issue](https://guides.github.com/features/issues/) to discuss your proposal before starting.
1. Create your own fork and clone the repository to your local machine.
1. Commit and push your changes to your fork.
1. Run `pre-commit run --all-files` to ensure that your code is properly formatted.
1. When your code is finished, create a pull request.

Once a pull request has been created, it will be reviewed, and changes might be suggested, which you can make by simply adding new commits to your branch. After the successful review, we will merge your changes into the main repository.

We aim to keep the codebase clean and maintainable. Therefore, we may not be able to accept all contributions (even if they are well-written and feasible). For this reason, it is advisable to discuss possible contributions beforehand in an issue.
