Contributing to motulator
=========================

Thank you for investing your time in contributing to our project!
In this guide you will get an overview of the contribution workflow.

The project is hosted on https://github.com/Aalto-Electric-Drives/motulator

- [Ways to contribute](#ways-to-contribute)
- [Submitting a bug report or a feature request](#submitting-a-bug-report-or-a-feature-request)
- [Contributing code](#contributing-code)
  - [Workflow for contributing code](#workflow-for-contributing-code)
    - [A. Before you begin](#a-before-you-begin)
    - [B. Writing your code](#b-writing-your-code)
    - [C. Merging your changes with motulator](#c-merging-your-changes-with-motulator)
  - [Coding style guidelines](#coding-style-guidelines)
  - [Dependencies and reusing code](#dependencies-and-reusing-code)
- [Contributing to documentation](#contributing-to-documentation)
  - [Building the documentation](#building-the-documentation)
  - [Workflow for contributing documentation](#workflow-for-contributing-documentation)
  - [Example python files](#example-python-files)

<a name="ways-to-contribute"></a>
## Ways to contribute

Most common ways of contributing are the implementation of code or
documentation. Other common ways of contributing are bug triaging and
fixing of typos or other nuances in the documentation. We'd also like
to hear about your suggestions, so if you wish to see a feature
implemented, feel free to open an
[issue](https://docs.github.com/en/issues/tracking-your-work-with-issues/about-issues)
on GitHub.

<a name="submitting-a-bug-report-or-a-feature-request"></a>
## Submitting a bug report or a feature request

If you have any feature requests for motulator or you have found bugs,
please submit them to the
[issue tracker](https://github.com/Aalto-Electric-Drives/motulator/issues).

Before submitting an issue, please verify that your issue is not being
currently addressed by other
[issues](https://github.com/Aalto-Electric-Drives/motulator/issues) or
[pull requests](https://github.com/Aalto-Electric-Drives/motulator/pulls).

When you are reporting a bug to Github, please include the following
in your bug report:

- Name and version of your operating system.

- Any relevant details about your local setup, such as the Python
  interpreter version and installed libraries.

- A short reproducible
  [code snippet](https://help.github.com/articles/creating-and-highlighting-code-blocks),
  so that anyone can reproduce the bug easily. For a snippet that is
  longer than 50 lines, please link to a [gist](https://gist.github.com)
  or a github repo.

- Try your best to be specific about what functions are involved and the
  shape of the data. This is especially important if including a
  reproducible code snippet is not feasible.

- If an exception is raised, please provide the full traceback.

<a name="contributing-code"></a>
## Contributing code

To avoid duplicating work, it is recommended that you search through the
[issue tracker](https://github.com/Aalto-Electric-Drives/motulator/issues)
and the
[pull request list](https://github.com/Aalto-Electric-Drives/motulator/pulls).
If in doubt about duplicated work, it is recommended to first open an issue in
the [issue tracker](https://github.com/Aalto-Electric-Drives/motulator/issues)
to get some feedbacks from core developers.

<a name="workflow-for-contributing-code"></a>
## Workflow for contributing code

Code contributions are implemented into `motulator/` folder, where
the source code (including methods and classes) is located.
When contributing code, make sure you follow this workflow:

<a name="a-before-you-begin"></a>
## A. Before you begin

1. Create an [issue](https://guides.github.com/features/issues/) where new proposals can be discussed before any coding is done.
2. Create a [branch](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/) of this repository on your own [fork](https://help.github.com/articles/fork-a-repo/), where all changes will be made.
3. Download the source code onto your local system, by [cloning](https://help.github.com/articles/cloning-a-repository/) your fork of the repository.
4. Install motulator with `$ pip install motulator`

<a name="b-writing-your-code"></a>
## B. Writing your code

5. Make sure to follow our [coding style guidelines](#coding-style-guidelines).
6. Commit your changes to your branch with useful, descriptive commit messages.
7. If you want to add a dependency on another library, or re-use code you found somewhere else, have a look at [these guidelines](#dependencies-and-reusing-code).

<a name="c-Merging-your-changes-with-motulator"></a>
## C. Merging your changes with motulator

8. Before merging, confirm that your code runs successfully and doesn't contain any bugs.
9. motulator has online documentation at https://aalto-electric-drives.github.io/motulator/. To make sure any new methods or classes you added show up there, please read the :ref:`documentation <contributing-to-documentation>` section.
10. If you added a major new feature, consider showcasing it in an [example](#example-python-files).
11. When you feel your code is finished, create a [pull request](https://help.github.com/articles/about-pull-requests/) on [motulator](https://github.com/Aalto-Electric-Drives/motulator).
12. Once a pull request has been created, it will be reviewed by any member of the community. Changes might be suggested which you can make by simply adding new commits to the branch. When everything's finished, someone with the right GitHub permissions will merge your changes into motulator main repository.

<a name="coding-style-guidelines"></a>
## Coding style guidelines

motulator follows the general Python style guide
[PEP8](https://www.python.org/dev/peps/pep-0008/)
for coding style. To ensure readability of Python code, please make sure to try
and follow this style guide as much as possible when contributing code.

<a name="dependencies-and-reusing-code"></a>
## Dependencies and reusing code

In case a contribution/issue involves changes to the API principles or changes
to dependencies, said changes should be carefully considered and discussed in
[GitHub](https://github.com/Aalto-Electric-Drives/motulator/issues). Including
code from other packages is possible, as long as their license permits it and
is compatible with ours, but again should be considered carefully and discussed
in the group.

<a name="contributing-to-documentation"></a>
## Contributing to documentation

motulator uses [Sphinx](https://www.sphinx-doc.org/en/master/index.html)
to auto-generate documentation. Every method and every class should have a
[docstring](https://www.python.org/dev/peps/pep-0257/) that describes what
it does, and what the expected input and output is. For docstrings, the
preferred style is
[Numpy's format](https://numpydoc.readthedocs.io/en/latest/format.html).
This format works well with Sphinx and is probably the most commonly used
format in Python projects.

<a name="building-the-documentation"></a>
## Building the documentation

Building the documentation requires installing the required Sphinx
extensions as well as some additional packages:

```
pip install sphinx m2r2 sphinx_rtd_theme sphinx-gallery numpydoc \
            spinx-copybutton sphinx-autoapi motulator
```

Also make sure to install the requirements described in the
`requirements.txt` file with pip install (numpy, matplotlib etc.).

To build the documentation, you first need to be in the `docs` folder:

```
cd docs
```

When you are in the docs folder, the documentation can be built by running:

```
make html
```

This documentation will be generated in the `docs/build/html/` directory,
and can be viewed from web browser by opening one of the .html files from
that directory.

<a name="workflow-for-contributing-documentation"></a>
## Workflow for contributing documentation

Contributions to documentation include reStructuredText documents,
function docstrings, [examples](#example-python-files), etc.
reStructuredText documents live in the source code repository under
the `docs/source/` directory.

Workflow for contributing documentation goes similarly to
[how code is contributed](#workflow-for-contributing-code), so in summary,
documentation is merged to motulator from branches of motulator forks.

<a name="example-python-files"></a>
## Example python files

motulator uses [sphinx-gallery](https://sphinx-gallery.github.io/stable/index.html)
extension to feature the `examples/` folder scripts into the documentation.
The examples in the `examples/` folder are categorized inside their respective
subfolder. For example `examples/im/` folder features examples on induction motor
drives and `examples/sm/` features examples on synchronous motor drives. Please
try your best to follow this convention as well.

If you have made contributions to code that could be considered a "major" feature,
then please consider including this feature in an example. Of course it is
subjective which features are "major", so please discuss on GitHub first!