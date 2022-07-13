Contributing
============

Thank you for investing your time in contributing to our project!
In this guide you will get an overview of the contribution workflow.

The project is hosted on https://github.com/Aalto-Electric-Drives/motulator

.. contents::
   :depth: 3
   :backlinks: none

Ways to contribute
------------------

Most common ways of contributing are the implementation of code or
documentation. Other common ways of contributing are bug triaging and
fixing of typos or other nuances in the documentation. We'd also like
to hear about your suggestions, so if you wish to see a feature
implemented, feel free to open an `issue
<https://docs.github.com/en/issues/tracking-your-work-with-issues/about-issues>`__
on GitHub.

Submitting a bug report or a feature request
--------------------------------------------

If you have any feature requests for motulator or you have found bugs,
please submit them to the `issue tracker
<https://github.com/Aalto-Electric-Drives/motulator/issues>`__.

Before submitting an issue, please verify that your issue is not being
currently addressed by other
`issues <https://github.com/Aalto-Electric-Drives/motulator/issues>`__ or
`pull requests <https://github.com/Aalto-Electric-Drives/motulator/pulls>`__

When you are reporting a bug to Github, please include the following
in your bug report:

- Name and version of your operating system.

- Any relevant details about your local setup, such as the Python
  interpreter version and installed libraries.

- A short reproducible `code snippet
  <https://help.github.com/articles/creating-and-highlighting-code-blocks>`__,
  so that anyone can reproduce the bug easily. For a snippet that is
  longer than 50 lines, please link to a `gist
  <https://gist.github.com>`_ or a github repo.

- Try your best to be specific about what functions are involved and the
  shape of the data. This is especially important if including a
  reproducible code snippet is not feasible.

- If an exception is raised, please provide the full traceback.

Contributing code
-----------------

.. note::

  To avoid duplicating work, it is recommended that you search through the
  `issue tracker <https://github.com/Aalto-Electric-Drives/motulator/issues>`__
  and the
  `pull request list <https://github.com/Aalto-Electric-Drives/motulator/pulls>`__.
  If in doubt about duplicated work, it is recommended to first open an issue in
  the `issue tracker <https://github.com/Aalto-Electric-Drives/motulator/issues>`__
  to get some feedbacks from core developers.

.. _workflow-for-contributing-code:

Workflow for contributing code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Code contributions are implemented into ``motulator/`` folder, where
the source code (including methods and classes) is located.
When contributing code, make sure you follow this workflow:

A. Before you begin
###################

1. Create an `issue <https://guides.github.com/features/issues/>`__ where new proposals can be discussed before any coding is done.
2. Create a `branch <https://help.github.com/articles/creating-and-deleting-branches-within-your-repository/>`__ of this repository on your own `fork <https://help.github.com/articles/fork-a-repo/>`__, where all changes will be made.
3. Download the source code onto your local system, by `cloning <https://help.github.com/articles/cloning-a-repository/>`__ your fork of the repository.
4. Install motulator with `$ pip install motulator`

B. Writing your code
####################

5. Make sure to follow our :ref:`coding style guidelines <coding-style-guidelines>`.
6. Commit your changes to your branch with useful, descriptive commit messages.
7. If you want to add a dependency on another library, or re-use code you found somewhere else, have a look at :ref:`these guidelines <dependencies-and-reusing-code>`.

C. Merging your changes with motulator
######################################

8. Before merging, confirm that your code runs successfully and doesn't contain any bugs.
9. motulator has online documentation at https://aalto-electric-drives.github.io/motulator/. To make sure any new methods or classes you added show up there, please read the :ref:`documentation <contributing-to-documentation>` section.
10. If you added a major new feature, consider showcasing it in an :ref:`example <example-python-files>`.
11. When you feel your code is finished, create a `pull request <https://help.github.com/articles/about-pull-requests/>`__ on `motulator <https://github.com/Aalto-Electric-Drives/motulator>`__.
12. Once a pull request has been created, it will be reviewed by any member of the community. Changes might be suggested which you can make by simply adding new commits to the branch. When everything's finished, someone with the right GitHub permissions will merge your changes into motulator main repository.

.. _coding-style-guidelines:

Coding style guidelines
~~~~~~~~~~~~~~~~~~~~~~~

motulator follows the general Python style guide `PEP8
<https://www.python.org/dev/peps/pep-0008/>`__
for coding style. To ensure readability of Python code, please make sure to try
and follow this style guide as much as possible when contributing code.

.. _dependencies-and-reusing-code:

Dependencies and reusing code
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

In case a contribution/issue involves changes to the API principles or changes
to dependencies, said changes should be carefully considered and discussed in
`GitHub <https://github.com/Aalto-Electric-Drives/motulator/issues>`__. Including
code from other packages is possible, as long as their license permits it and
is compatible with ours, but again should be considered carefully and discussed
in the group.

.. _contributing-to-documentation:

Contributing to documentation
-----------------------------

motulator uses `Sphinx <https://www.sphinx-doc.org/en/master/index.html>`__
to auto-generate documentation. Every method and every class should have a
`docstring <https://www.python.org/dev/peps/pep-0257/>`__ that describes what
it does, and what the expected input and output is. For docstrings, the
preferred style is `Numpy's format
<https://numpydoc.readthedocs.io/en/latest/format.html>`__.
This format works well with Sphinx and is probably the most commonly used
format in Python projects.

Building the documentation
~~~~~~~~~~~~~~~~~~~~~~~~~~

Building the documentation requires installing the required Sphinx
extensions as well as some additional packages:

.. code::

    pip install sphinx m2r2 sphinx_rtd_theme sphinx-gallery numpydoc \
                spinx-copybutton sphinx-autoapi motulator

Also make sure to install the requirements described in the
``requirements.txt`` file with pip install (numpy, matplotlib etc.).

To build the documentation, you first need to be in the ``docs`` folder:

.. code::

    cd docs

When you are in the docs folder, the documentation can be built by running:

.. code::

    make html

This documentation will be generated in the ``docs/build/html/`` directory,
and can be viewed from web browser by opening one of the .html files from
that directory.

Workflow for contributing documentation
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Contributions to documentation include reStructuredText documents,
function docstrings, :ref:`examples <example-python-files>`, etc.
reStructuredText documents live in the source code repository under
the ``docs/source/`` directory.

Workflow for contributing documentation goes similarly to :ref:`how code
is contributed <workflow-for-contributing-code>`, so in summary,
documentation is merged to motulator from branches of motulator forks.

.. _example-python-files:

Example python files
~~~~~~~~~~~~~~~~~~~~

motulator uses `sphinx-gallery <https://sphinx-gallery.github.io/stable/index.html>`__
extension to feature the ``examples/`` folder scripts into the documentation.
The examples in the ``examples/`` folder are categorized inside their respective
subfolder. For example ``examples/im/`` folder features examples on induction motor
drives and ``examples/sm/`` features examples on synchronous motor drives. Please
try your best to follow this convention as well.

If you have made contributions to code that could be considered a "major" feature,
then please consider including this feature in an example. Of course it is
subjective which features are "major", so please discuss on GitHub first!
