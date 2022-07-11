Contributing
============

Thank you for investing your time in contributing to our project!
In this guide you will get an overview of the contribution workflow.

The project is hosted on https://github.com/Aalto-Electric-Drives/motulator

Ways to contribute
------------------

Most common ways of contributing are the implementation of code or
documentation. Other common ways of contributing are bug fixes and fixing
of typos or other nuances in the documentation. Documentation can be found
under the `docs/` directory.

Bug triaging, bug reporting, reviewing pull requests and suggesting features
are also helpful contributions we are looking forward to.


Submitting a bug report or a feature request
--------------------------------------------

We use GitHub `issues <https://github.com/Aalto-Electric-Drives/motulator/issues>`_
to track bugs and feature requests. If you have found a bug or wish to see a feature
implemented, feel free to open an issue.

It is recommended to verify that your issue is not being currently addressed by
other `issues <https://github.com/Aalto-Electric-Drives/motulator/issues>`_ or
`pull requests <https://github.com/Aalto-Electric-Drives/motulator/pulls>`_

When reporting a bug to `Github
<https://github.com/Aalto-Electric-Drives/motulator/issues>`_, please do your best to
follow these guidelines! This will make it a lot easier to provide you with good
feedback:

- Make sure to include a short reproducible code snippet in a bug report,
  this way anyone can reproduce the bug easily. For a snippet that is
  longer than 50 lines, please link to a `gist
  <https://gist.github.com>`_ or a github repo.

- Try your best to be specific about what functions are involved and the
  shape of the data. This is especially important if including a
  reproducible code snippet is not feasible.

- If an exception is raised, please provide the full traceback.

- Include your **operating system type and version number**, as well as
  your **Python, motulator and requirements versions** (from `requirements.txt`).

- Please ensure all code snippets and error messages are formatted in
  appropriate code blocks.  See `Creating and highlighting code blocks
  <https://help.github.com/articles/creating-and-highlighting-code-blocks>`_
  for more details.

Contributing code
-----------------

.. note::

  To avoid duplicating work, it is highly advised that you search through the
  `issue tracker <https://github.com/Aalto-Electric-Drives/motulator/issues>`_ and
  the `PR list <https://github.com/Aalto-Electric-Drives/motulator/pulls>`_.
  If in doubt about duplicated work, or if you want to work on a non-trivial
  feature, it's recommended to first open an issue in
  the `issue tracker <https://github.com/scikit-learn/scikit-learn/issues>`_
  to get some feedbacks from core developers.

Coding syle guidelines
----------------------

motulator follows the general Python style guide
[PEP8](https://www.python.org/dev/peps/pep-0008/)
for coding style. To ensure readability of Python code, please make sure to try
and follow this style guide as much as possible when contributing code.

How to contribute code
----------------------

The preferred way to contribute to motulator is to fork the `main
repository <https://github.com/Aalto-Electric-Drives/motulator>`_ on GitHub,
then submit a "pull request"...

Contributing documentation
--------------------------

motulator uses [Sphinx](https://www.sphinx-doc.org/en/master/index.html)
to auto-generate documentation. For docstrings, the preferred style is
[Numpy's format](https://numpydoc.readthedocs.io/en/latest/format.html).
This format works well with Sphinx and is probably the most commonly used
format in Python projects.

We are glad to accept any sort of documentation: function docstrings,
reStructuredText documents, tutorials, etc. reStructuredText
documents live in the source code repository under the ``docs/`` directory.

You can edit the documentation using any text editor, and then generate the
HTML output by typing ``make`` from the ``docs/`` directory. Alternatively,
``make html`` may be used to generate the documentation with the example
gallery (which takes quite some time). The resulting HTML files will be
placed in ``build/html/index.html`` and are viewable in a web browser.

Building the documentation requires installing some additional packages:

.. prompt:: bash $

    pip install sphinx m2r2 sphinx_rtd_theme sphinx-gallery numpydoc \
                spinx-copybutton sphinx-autoapi motulator

Also make sure to install the requirements described in the
``requirements.txt`` file with pip install (numpy, matplotlib etc.).

To build the documentation, you need to be in the ``doc`` folder:

.. prompt:: bash $

    cd doc
