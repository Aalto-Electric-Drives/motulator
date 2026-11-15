"""Sort key of the examples within the subsections of the gallery."""

import os

from sphinx_gallery.sorting import NumberOfCodeLinesSortKey


class ExampleSortKey:
    """Sort examples by the number of code lines, but place the listed ones last."""

    last = ["plot_13kva_do_gfm_sag.py"]

    def __init__(self, src_dir):
        self.n_lines = NumberOfCodeLinesSortKey(src_dir)

    def __call__(self, filename):
        name = os.path.basename(filename)
        rank = self.last.index(name) if name in self.last else -1
        return (rank, self.n_lines(filename))
