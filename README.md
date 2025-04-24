# *motulator:* Motor Drive and Grid Converter Simulator in Python
[![DOI](https://zenodo.org/badge/377399301.svg)](https://zenodo.org/doi/10.5281/zenodo.10223090)
[![Build Status](https://github.com/Aalto-Electric-Drives/motulator/actions/workflows/update_gh-pages.yml/badge.svg)](https://github.com/Aalto-Electric-Drives/motulator/actions/workflows/update_gh-pages.yml)
[![License](https://img.shields.io/github/license/mashape/apistatus)](https://github.com/Aalto-Electric-Drives/motulator/blob/main/LICENSE)
[![PyPI version shields.io](https://img.shields.io/pypi/v/motulator.svg)](https://pypi.org/project/motulator/)
[![All Contributors](https://img.shields.io/badge/all_contributors-6-orange.svg?style=flat-square)](#contributors-)

Introduction
------------
This open-source software includes simulation models and controllers for electric machine drives and grid converter systems. The machine models include an induction machine, a synchronous reluctance machine, and a permanent-magnet synchronous machine. Various subsystem models are provided for modeling grid converter systems, such as an LCL filter connected to an inductive-resistive grid.

The system models are simulated in the continuous-time domain while the control algorithms run in discrete time. The default solver is the explicit Runge-Kutta method of order 5(4) from scipy.integrate.solve_ivp. Various control algorithms are provided as examples. The documentation is available here:

https://aalto-electric-drives.github.io/motulator/

Installation
------------
This software can be installed using pip:

```bash
pip install motulator
```
Alternatively, the repository can be cloned:

https://aalto-electric-drives.github.io/motulator/installation.html

Usage
-----
The system models, controllers, reference sequences etc. are easy to configure. As a starting point, example scripts and Jupyter notebooks can be downloaded here:

https://aalto-electric-drives.github.io/motulator/drive_examples/index.html

https://aalto-electric-drives.github.io/motulator/grid_examples/index.html

New system models and controllers can be developed using the existing ones as templates.

Contributing
------------
If you would like to help us develop *motulator*, see these [guidelines](https://aalto-electric-drives.github.io/motulator/contributing.html) first.

Contributors
------------
Thanks go to these wonderful people:

<!-- ALL-CONTRIBUTORS-LIST:START - Do not remove or modify this section -->
<!-- prettier-ignore-start -->
<!-- markdownlint-disable -->
<table>
  <tbody>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/lauritapio"><img src="https://avatars.githubusercontent.com/u/85596019?v=4?s=50" width="50px;" alt="Lauri Tiitinen"/><br /><sub><b>Lauri Tiitinen</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=lauritapio" title="Code">💻</a> <a href="#ideas-lauritapio" title="Ideas, Planning, & Feedback">🤔</a> <a href="#example-lauritapio" title="Examples">💡</a> <a href="#mentoring-lauritapio" title="Mentoring">🧑‍🏫</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/HannuHar"><img src="https://avatars.githubusercontent.com/u/96597650?v=4?s=50" width="50px;" alt="HannuHar"/><br /><sub><b>HannuHar</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=HannuHar" title="Code">💻</a> <a href="https://github.com/Aalto-Electric-Drives/motulator/issues?q=author%3AHannuHar" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://research.aalto.fi/en/persons/marko-hinkkanen"><img src="https://avatars.githubusercontent.com/u/76600872?v=4?s=50" width="50px;" alt="Marko Hinkkanen"/><br /><sub><b>Marko Hinkkanen</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=mhinkkan" title="Code">💻</a> <a href="#ideas-mhinkkan" title="Ideas, Planning, & Feedback">🤔</a> <a href="#example-mhinkkan" title="Examples">💡</a> <a href="#mentoring-mhinkkan" title="Mentoring">🧑‍🏫</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/silundbe"><img src="https://avatars.githubusercontent.com/u/81169347?v=4?s=50" width="50px;" alt="silundbe"/><br /><sub><b>silundbe</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=silundbe" title="Code">💻</a> <a href="#example-silundbe" title="Examples">💡</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/JoonaKukkonen"><img src="https://avatars.githubusercontent.com/u/85099403?v=4?s=50" width="50px;" alt="JoonaKukkonen"/><br /><sub><b>JoonaKukkonen</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=JoonaKukkonen" title="Code">💻</a> <a href="#infra-JoonaKukkonen" title="Infrastructure (Hosting, Build-Tools, etc)">🚇</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/jarno-k"><img src="https://avatars.githubusercontent.com/u/84438313?v=4?s=50" width="50px;" alt="jarno-k"/><br /><sub><b>jarno-k</b></sub></a><br /><a href="#ideas-jarno-k" title="Ideas, Planning, & Feedback">🤔</a> <a href="https://github.com/Aalto-Electric-Drives/motulator/pulls?q=is%3Apr+reviewed-by%3Ajarno-k" title="Reviewed Pull Requests">👀</a> <a href="#mentoring-jarno-k" title="Mentoring">🧑‍🏫</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/angelicaiaderosa"><img src="https://avatars.githubusercontent.com/u/112799415?v=4?s=50" width="50px;" alt="angelicaiaderosa"/><br /><sub><b>angelicaiaderosa</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=angelicaiaderosa" title="Code">💻</a> <a href="#example-angelicaiaderosa" title="Examples">💡</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://www.kth.se/profile/lucap"><img src="https://avatars.githubusercontent.com/u/64190518?v=4?s=50" width="50px;" alt="Luca Peretti"/><br /><sub><b>Luca Peretti</b></sub></a><br /><a href="#ideas-lucaperetti" title="Ideas, Planning, & Feedback">🤔</a> <a href="#promotion-lucaperetti" title="Promotion">📣</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/GianmarioPellegrinoPolito"><img src="https://avatars.githubusercontent.com/u/70333484?v=4?s=50" width="50px;" alt="GianmarioPellegrinoPolito"/><br /><sub><b>GianmarioPellegrinoPolito</b></sub></a><br /><a href="#data-GianmarioPellegrinoPolito" title="Data">🔣</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/SimFerr"><img src="https://avatars.githubusercontent.com/u/67151973?v=4?s=50" width="50px;" alt="Simone Ferrari"/><br /><sub><b>Simone Ferrari</b></sub></a><br /><a href="#data-SimFerr" title="Data">🔣</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/Jialed0303"><img src="https://avatars.githubusercontent.com/u/118135952?v=4?s=50" width="50px;" alt="Jialed0303"/><br /><sub><b>Jialed0303</b></sub></a><br /><a href="#ideas-Jialed0303" title="Ideas, Planning, & Feedback">🤔</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/murgui"><img src="https://avatars.githubusercontent.com/u/29175623?v=4?s=50" width="50px;" alt="murgui"/><br /><sub><b>murgui</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/issues?q=author%3Amurgui" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/iam-nithin-10"><img src="https://avatars.githubusercontent.com/u/125553207?v=4?s=50" width="50px;" alt="Nithin Valiyaveettil Sadanandan"/><br /><sub><b>Nithin Valiyaveettil Sadanandan</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/issues?q=author%3Aiam-nithin-10" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/saarela"><img src="https://avatars.githubusercontent.com/u/10281832?v=4?s=50" width="50px;" alt="saarela"/><br /><sub><b>saarela</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/issues?q=author%3Asaarela" title="Bug reports">🐛</a></td>
    </tr>
    <tr>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/UshnishChowdhury"><img src="https://avatars.githubusercontent.com/u/35863166?v=4?s=50" width="50px;" alt="Ushnish"/><br /><sub><b>Ushnish</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/issues?q=author%3AUshnishChowdhury" title="Bug reports">🐛</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/Francesco-Lelli"><img src="https://avatars.githubusercontent.com/u/127111681?v=4?s=50" width="50px;" alt="Francesco-Lelli"/><br /><sub><b>Francesco-Lelli</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=Francesco-Lelli" title="Code">💻</a> <a href="#example-Francesco-Lelli" title="Examples">💡</a> <a href="#ideas-Francesco-Lelli" title="Ideas, Planning, & Feedback">🤔</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/MiSaren"><img src="https://avatars.githubusercontent.com/u/166725462?v=4?s=50" width="50px;" alt="Mikko Sarén"/><br /><sub><b>Mikko Sarén</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=MiSaren" title="Code">💻</a> <a href="#example-MiSaren" title="Examples">💡</a> <a href="#ideas-MiSaren" title="Ideas, Planning, & Feedback">🤔</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/maattaj11"><img src="https://avatars.githubusercontent.com/u/165767331?v=4?s=50" width="50px;" alt="Juho Määttä"/><br /><sub><b>Juho Määttä</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=maattaj11" title="Code">💻</a> <a href="#example-maattaj11" title="Examples">💡</a> <a href="#ideas-maattaj11" title="Ideas, Planning, & Feedback">🤔</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://github.com/rayanmour"><img src="https://avatars.githubusercontent.com/u/111271373?v=4?s=50" width="50px;" alt="rayanmour"/><br /><sub><b>rayanmour</b></sub></a><br /><a href="https://github.com/Aalto-Electric-Drives/motulator/commits?author=rayanmour" title="Code">💻</a> <a href="#example-rayanmour" title="Examples">💡</a> <a href="#ideas-rayanmour" title="Ideas, Planning, & Feedback">🤔</a> <a href="https://github.com/Aalto-Electric-Drives/motulator/pulls?q=is%3Apr+reviewed-by%3Arayanmour" title="Reviewed Pull Requests">👀</a> <a href="#mentoring-rayanmour" title="Mentoring">🧑‍🏫</a></td>
      <td align="center" valign="top" width="14.28%"><a href="https://cusma.algo.xyz/"><img src="https://avatars.githubusercontent.com/u/65770425?v=4?s=50" width="50px;" alt="Cosimo Bassi"/><br /><sub><b>Cosimo Bassi</b></sub></a><br /><a href="#infra-cusma" title="Infrastructure (Hosting, Build-Tools, etc)">🚇</a></td>
    </tr>
  </tbody>
</table>

<!-- markdownlint-restore -->
<!-- prettier-ignore-end -->

<!-- ALL-CONTRIBUTORS-LIST:END -->

This project follows the [all-contributors](https://github.com/all-contributors/all-contributors) specification. Contributions of any kind welcome!

Acknowledgement
---------------
This project has been sponsored by ABB Oy and by the Research Council of Finland *Centre of Excellence in High-Speed Electromechanical Energy Conversion Systems*. The example control methods included in this repository are based on published algorithms (available in textbooks and scientific articles). They do not present any proprietary control software.
