Contributing
============

We would be very thankful to receive contributions to motulator, and everyone is welcome to contribute.
In case you're interested in contributing, these guidelines include instructions for different
types of contributions, whether it be implementations in source code and documentation or reviewing pull requests.

The project is hosted on https://github.com/Aalto-Electric-Drives/motulator

Ways to contribute
==================

There are many ways to contribute to motulator...

List different ways to contribute here (for example, programming and documentation)

Submitting a bug report or a feature request
============================================

We use GitHub `issues <https://github.com/Aalto-Electric-Drives/motulator/issues>`_
to track bugs and feature requests. If you have found a bug or wish to see a feature
implemented, feel free to open an issue.

It is recommended to check that your issue complies with the
following rules before submitting:

-  Verify that your issue is not being currently addressed by other
   `issues <https://github.com/Aalto-Electric-Drives/motulator/issues>`_ or
   `pull requests https://github.com/Aalto-Electric-Drives/motulator/pulls>`_

When reporting a bug to `Github
<https://github.com/Aalto-Electric-Drives/motulator/issues>`_, please do your best to
follow these guidelines! This will make it a lot easier to provide you with good
feedback:

- The ideal bug report contains a short reproducible code snippet, this way
  anyone can try to reproduce the bug easily (see `this
  <https://stackoverflow.com/help/mcve>`_ for more details). If your snippet is
  longer than around 50 lines, please link to a `gist
  <https://gist.github.com>`_ or a github repo.

- If not feasible to include a reproducible snippet, please be specific about
  what **estimators and/or functions are involved and the shape of the data**.

- If an exception is raised, please **provide the full traceback**.

- Please include your **operating system type and version number**, as well as
  your **Python, scikit-learn, numpy, and scipy versions**. This information
  can be found by running the following code snippet::

    >>> import sklearn
    >>> sklearn.show_versions()  # doctest: +SKIP

- Please ensure all **code snippets and error messages are formatted in
  appropriate code blocks**.  See `Creating and highlighting code blocks
  <https://help.github.com/articles/creating-and-highlighting-code-blocks>`_
  for more details.

Contributing code
=================

.. note::

  To avoid duplicating work, it is highly advised that you search through the
  `issue tracker <https://github.com/Aalto-Electric-Drives/motulator/issues>`_ and
  the `PR list <https://github.com/Aalto-Electric-Drives/motulator/pulls>`_.
  If in doubt about duplicated work, or if you want to work on a non-trivial
  feature, it's recommended to first open an issue in
  the `issue tracker <https://github.com/scikit-learn/scikit-learn/issues>`_
  to get some feedbacks from core developers.

How to contribute code
----------------------

The preferred way to contribute to motulator is to fork the `main
repository <https://github.com/Aalto-Electric-Drives/motulator>`_ on GitHub,
then submit a "pull request"...

Documentation
=============

We are glad to accept any sort of documentation: function docstrings,
reStructuredText documents (like this one), tutorials, etc. reStructuredText
documents live in the source code repository under the ``docs/`` directory.

You can edit the documentation using any text editor, and then generate the
HTML output by typing ``make`` from the ``docs/`` directory. Alternatively,
``make html`` may be used to generate the documentation **with** the example
gallery (which takes quite some time). The resulting HTML files will be
placed in ``_build/html/stable`` and are viewable in a web browser...
