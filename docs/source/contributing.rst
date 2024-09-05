Contributing
============

Style Guide
-----------

We follow the `Style Guide for Python Code PEP8 <https://peps.python.org/pep-0008/>`_. Furthermore, we use the `YAPF Python code formatter <https://github.com/google/yapf>`_, configured based on the PEP8 conventions in the ``pyproject.toml`` file. Public methods and classes should have proper `docstrings <https://peps.python.org/pep-0257/>`_, formatted according to the `NumPy docstring guide <https://numpydoc.readthedocs.io/en/latest/format.html>`_.

`Sphinx <https://www.sphinx-doc.org>`_ is used to auto-generate documentation. We use `Sphinx-Gallery <https://sphinx-gallery.github.io/stable/index.html>`_ to automatically generate the `examples <https://aalto-electric-drives.github.io/motulator/drive_examples/index.html>`_ from the scripts in the ``examples`` folder. The format of the example scripts is essential for their proper rendering in the documentation (see the existing example scripts).

Install Optional Dependencies
-----------------------------

We recommend to install the ``dev`` dependencies (see also :doc:`/installation`). To enable YAPF in VS Code, you can install the `YAPF extension <https://marketplace.visualstudio.com/items?itemName=eeyore.yapf>`_. This extension will automatically format the code according when you save the file.

If you aim to work with the documentation, install also the ``doc`` dependencies. For previewing the documentation in VS Code, you can install the `Esbonio extension <https://marketplace.visualstudio.com/items?itemName=swyddfa.esbonio>`_. Alternatively, you can build the documentation locally using the command ``make html`` in the ``docs`` directory. The documentation will be built in the ``docs/build`` directory.


Submitting a Bug Report or a Feature Request
--------------------------------------------

If you have found bugs or have feature requests, please submit them to the `issue tracker <https://github.com/Aalto-Electric-Drives/motulator/issues>`_. If you are reporting a bug, please include the following in your report:

- Name and version of your operating system.
- Any relevant details about your local setup, such as the Python interpreter version and installed libraries.
- A short reproducible code snippet, so that anyone can reproduce the bug easily.
- Try to be specific about what functions are involved and the shape of the data, especially when including a code snippet is not feasible.
- Provide the full traceback to any exceptions that were raised.

Workflow for Contributing
-------------------------

When contributing code, please follow this workflow:

1. Create a new `issue <https://guides.github.com/features/issues/>`_ to discuss your proposal before starting.
2. Create your own fork and clone the repository to your local machine.
3. Commit and push your changes to your fork.
4. When your code is finished, create a pull request.

Once a pull request has been created, it will be reviewed, and changes might be suggested, which you can make by simply adding new commits to your branch. After the successful review, we will merge your changes into the main repository.

We aim to keep the codebase clean and maintainable. Therefore, we may not be able to accept all contributions (even if they are well-written and feasible). For this reason, it is advisable to discuss possible contributions beforehand in an issue.
