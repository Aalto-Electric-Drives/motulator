Contributing to motulator
=========================

Thank you for investing your time in contributing to our project!
In this guide you will get an overview of the contribution workflow.

Most common ways of contributing are the implementation of code or
documentation. Other common ways of contributing are bug triaging and
fixing of typos or errors in the documentation. We'd also like
to hear about your suggestions, so if you wish to see a feature
implemented, feel free to open an
[issue](https://docs.github.com/en/issues/tracking-your-work-with-issues/about-issues)
on GitHub.

The project is hosted on https://github.com/Aalto-Electric-Drives/motulator

- [Submitting a bug report or a feature request](#submitting-a-bug-report-or-a-feature-request)
- [Workflow for contributing code](#workflow-for-contributing-code)
- [Contributing to documentation](#contributing-to-documentation)

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
  so that anyone can reproduce the bug easily. Alternatively you can link to a
  [gist](https://gist.github.com) or a github repo for longer code snippets.

- Try to be specific about what functions are involved and the
  shape of the data, especially when including a
  code snippet is not feasible.

- If an exception is raised, please provide the full traceback.

<a name="workflow-for-contributing-code"></a>
## Workflow for contributing code

motulator follows the general Python style guide
[PEP8](https://www.python.org/dev/peps/pep-0008/)
for coding style. To ensure readability of Python code, please make sure to try
and follow this style guide as much as possible when contributing code.

Code contributions are implemented into `motulator/` folder, where
the source code (including methods and classes) is located. Make sure to
download motulator by using the command `$ pip install motulator`. When
contributing code, make sure you follow this workflow:

1. Create an [issue](https://guides.github.com/features/issues/) where new proposals can be discussed before any coding is done.
2. Create a [branch](https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/) of this repository on your own [fork](https://help.github.com/articles/fork-a-repo/), where all changes will be made, and [clone](https://help.github.com/articles/cloning-a-repository/) the repository to your local machine.
3. [Commit](https://github.com/git-guides/git-commit) and [push](https://github.com/git-guides/git-push) your changes to your branch.
4. When your code is finished, create a [pull request](https://help.github.com/articles/about-pull-requests/) on [motulator](https://github.com/Aalto-Electric-Drives/motulator).

Once a pull request has been created, it will be reviewed, and changes might
be suggested, which you can make by simply adding new commits to your branch.
When everything's finished, someone with the right GitHub permissions will
merge your changes into motulator main repository.

If your changes include direct inclusion of code from other packages or dependencies
other than described in
[requirements](https://github.com/Aalto-Electric-Drives/motulator/blob/main/requirements.txt),
make sure to carefully consider and discuss this in your GitHub issue. If for instance, code is
copied from another GitHub repository, the license of that GitHub repository has to permit this
and the license has to be compatible with ours.

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

Contributions to documentation include reStructuredText documents,
function docstrings and examples. The reStructuredText documents live in
the source code repository under the `docs/source/` directory. motulator
uses [sphinx-gallery](https://sphinx-gallery.github.io/stable/index.html)
to automatically generate
[examples](https://aalto-electric-drives.github.io/motulator/auto_examples/index.html)
from the `examples/` folder.

Workflow for contributing documentation goes similarly to
[how code is contributed](#workflow-for-contributing-code), so in summary,
documentation is merged to motulator by merging from a forked repository.

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
