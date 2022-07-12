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
under the ``docs/`` directory.

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

  To avoid duplicating work, it is recommended that you search through the
  `issue tracker <https://github.com/Aalto-Electric-Drives/motulator/issues>`_
  and the
  `pull request list <https://github.com/Aalto-Electric-Drives/motulator/pulls>`_.
  If in doubt about duplicated work, it is recommended to first open an issue in
  the `issue tracker <https://github.com/Aalto-Electric-Drives/motulator/issues>`_
  to get some feedbacks from core developers.

Workflow for contributing code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Code contributions are implemented into ``motulator/`` folder, where
the source code (including methods and classes) is located.
When contributing code, make sure you follow the following workflow:

A. Before you begin
###################

1. Create an `issue <https://guides.github.com/features/issues/>`_ where new proposals can be discussed before any coding is done.
2. Create a `branch <https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/>`_ of this repository on your own `fork <https://help.github.com/articles/fork-a-repo/>`_, where all changes will be made.
3. Download the source code onto your local system, by `cloning <https://help.github.com/articles/cloning-a-repository/>`_ your fork of the repository.
4. Install motulator with `$ pip install motulator`

B. Writing your code
####################

5. Make sure to follow our :ref:`coding style guidelines <coding-style-guidelines>`.
6. Commit your changes to your branch with `useful, descriptive commit messages <https://chris.beams.io/posts/git-commit/>`_.
7. If you want to add a dependency on another library, or re-use code you found somewhere else, have a look at :ref:`these guidelines <dependencies-and-reusing-code>`.

C. Merging your changes with motulator
######################################

8. Before merging, confirm that your code runs successfully and doesn't contain any bugs.
9. motulator has online documentation at https://aalto-electric-drives.github.io/motulator/. To make sure any new methods or classes you added show up there, please read the :ref:`documentation <contributing-to-documentation>` section.
10. If you added a major new feature, consider showcasing it in an :ref:`example <example-python-files>`.
11. When you feel your code is finished, create a `pull request <https://help.github.com/articles/about-pull-requests/>`_ on `motulator <https://github.com/Aalto-Electric-Drives/motulator>`_.
12. Once a pull request has been created, it will be reviewed by any member of the community. Changes might be suggested which you can make by simply adding new commits to the branch. When everything's finished, someone with the right GitHub permissions will merge your changes into motulator main repository.

.. _coding-style-guidelines:

Coding style guidelines
~~~~~~~~~~~~~~~~~~~~~~

motulator follows the general Python style guide `PEP8
<https://www.python.org/dev/peps/pep-0008/>`_
for coding style. To ensure readability of Python code, please make sure to try
and follow this style guide as much as possible when contributing code.

.. _dependencies-and-reusing-code:

Dependencies and reusing code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In case a contribution/issue involves changes to the API principles or changes
to dependencies, said changes should be carefully considered and discussed in
`GitHub <https://guides.github.com/features/issues/>`_. Including code from
othe packages is possible, as long as their license permits it and is compatible
with ours, but again should be considered carefully and discussed in the group.

.. _contributing-to-documentation:

Contributing to documentation
-----------------------------

motulator uses `Sphinx <https://www.sphinx-doc.org/en/master/index.html>`_
to auto-generate documentation. For docstrings, the preferred style is
`Numpy's format <https://numpydoc.readthedocs.io/en/latest/format.html>`_.
This format works well with Sphinx and is probably the most commonly used
format in Python projects. Every method and every class should have a
docstring that describes what it does, and what the expected input and
output is.

We are glad to accept any sort of documentation: function docstrings,
reStructuredText documents, tutorials, etc. reStructuredText
documents live in the source code repository under the ``docs/`` directory.

You can edit the documentation using any text editor, and then generate the
HTML output by typing ``make`` from the ``docs/`` directory. Alternatively,
``make html`` may be used to generate the documentation with the example
gallery (which takes quite some time). The resulting HTML files will be
placed in ``build/html/index.html`` and are viewable in a web browser.

Building the documentation requires installing some additional packages:

.. code::

    pip install sphinx m2r2 sphinx_rtd_theme sphinx-gallery numpydoc \
                spinx-copybutton sphinx-autoapi motulator

Also make sure to install the requirements described in the
``requirements.txt`` file with pip install (numpy, matplotlib etc.).

To build the documentation, you need to be in the ``docs`` folder:

.. code::

    cd docs

When you are in the docs folder, the documentation can be built by running:

.. code::

    make html

Workflow for contributing documentation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Workflow for contributing documentation

.. _example-python-files:

Example python files
~~~~~~~~~~~~~~~~~~~~

motulator uses `sphinx-gallery <https://sphinx-gallery.github.io/stable/index.html>`_
extension to feature the ``examples/`` folder scripts into the documentation.
The examples in the ``examples/`` folder are categorized inside their respective
subfolder. For example ``examples/im/`` folder features examples on induction motor
drives and ``examples/sm/`` features examples on synchronous motor drives. Please
try your best to follow this convention as well.

If you have made contributions to code that could be considered a "major" feature,
then please consider including this feature in an example. Of course it is
subjective which features are "major", so please discuss on GitHub first!
