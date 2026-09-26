"""
Generate the README hero figures (light and dark theme).

Run from the repository root:

    python .github/assets/make_hero.py

The script simulates sensorless flux-vector control of a 2.2-kW IPMSM drive and
writes hero-light.svg and hero-dark.svg next to this file. The curves are revealed
once from left to right using an SMIL animation; viewers without SMIL support see
the static figure.

"""

# %%
import re
from math import pi
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

import motulator.drive.control.sm as control
from motulator.common.utils import complex2abc
from motulator.drive import model, utils

OUT_DIR = Path(__file__).parent
T_STOP = 1.2
ANIM_DUR = 3.0  # Duration of the reveal animation (s)

THEMES = {
    "light": {
        "fg": "#1f2328",
        "muted": "#656d76",
        "grid": "#d0d7de",
        "lines": ["#0969da", "#d1242f", "#1a7f37"],
        "ref": "#8c959f",
    },
    "dark": {
        "fg": "#e6edf3",
        "muted": "#9198a1",
        "grid": "#3d444d",
        "lines": ["#4493f8", "#f85149", "#3fb950"],
        "ref": "#6e7681",
    },
}


# %%
def simulate():
    """Simulate the drive and return the results and base values."""
    nom = utils.NominalValues(U=370, I=4.3, f=75, P=2.2e3, tau=14)
    base = utils.BaseValues.from_nominal(nom, n_p=3)

    par = model.SynchronousMachinePars(
        n_p=3, R_s=3.6, L_d=0.036, L_q=0.051, psi_f=0.545
    )
    machine = model.SynchronousMachine(par)
    mechanics = model.MechanicalSystem(J=0.015)
    converter = model.VoltageSourceConverter(u_dc=540)
    mdl = model.Drive(machine, mechanics, converter)

    cfg = control.FluxVectorControllerCfg(i_s_max=1.5 * base.i, sensorless=True)
    vector_ctrl = control.FluxVectorController(par, cfg)
    speed_ctrl = control.SpeedController(J=0.015, alpha_s=2 * pi * 4)
    ctrl = control.VectorControlSystem(vector_ctrl, speed_ctrl)

    ctrl.set_speed_ref(lambda t: (t > 0.1) * 0.5 * base.w_M)
    mdl.mechanics.set_external_load_torque(lambda t: (t > 0.7) * 0.7 * nom.tau)

    sim = model.Simulation(mdl, ctrl, show_progress=False)
    return sim.simulate(t_stop=T_STOP), base


def plot(res, base, theme):
    """Plot speed, torque, and phase currents."""
    c = THEMES[theme]
    plt.rcdefaults()
    plt.rcParams.update(
        {
            "font.family": "DejaVu Sans",
            "font.size": 10,
            "text.color": c["fg"],
            "axes.edgecolor": c["grid"],
            "axes.labelcolor": c["muted"],
            "xtick.color": c["muted"],
            "ytick.color": c["muted"],
            "axes.grid": True,
            "grid.color": c["grid"],
            "grid.linewidth": 0.6,
            "axes.spines.top": False,
            "axes.spines.right": False,
            "legend.frameon": False,
            "legend.fontsize": 9,
            "lines.linewidth": 1.4,
            "svg.hashsalt": "motulator",  # Deterministic ids in the SVG output
        }
    )

    mdl, ctrl = res.mdl, res.ctrl
    # Resample onto a uniform grid to keep the SVG file small
    t = np.linspace(0, T_STOP, 2400)
    w_m = np.interp(t, mdl.t, mdl.machine.w_m) / base.w
    tau_M = np.interp(t, mdl.t, mdl.machine.tau_M) / base.tau
    tau_L = np.interp(t, mdl.t, mdl.mechanics.tau_L_tot) / base.tau
    i_s_ab = np.interp(t, mdl.t, mdl.machine.i_s_ab.real) + 1j * np.interp(
        t, mdl.t, mdl.machine.i_s_ab.imag
    )
    i_abc = complex2abc(i_s_ab) / base.i

    fig, axs = plt.subplots(3, 1, sharex=True, figsize=(8, 4.6))
    L = c["lines"]

    ax = axs[0]
    ax.plot(ctrl.t, ctrl.ref.w_M / base.w_M, "--", ds="steps-post", c=c["ref"])
    ax.plot(t, w_m, c=L[0], label=r"$\omega_\mathrm{m}$")
    ax.plot(
        ctrl.t,
        ctrl.fbk.w_m / base.w,
        c=L[2],
        lw=1.0,
        ds="steps-post",
        label=r"$\hat{\omega}_\mathrm{m}$",
    )
    ax.set_ylabel("Speed (p.u.)")
    ax.set_ylim(-0.1, 0.7)

    ax = axs[1]
    ax.plot(t, tau_L, "--", c=c["ref"], label=r"$\tau_\mathrm{L}$")
    ax.plot(t, tau_M, c=L[0], label=r"$\tau_\mathrm{M}$")
    ax.set_ylabel("Torque (p.u.)")

    ax = axs[2]
    for k, name in enumerate("abc"):
        ax.plot(t, i_abc[k], c=L[k], lw=1.0, label=rf"$i_\mathrm{{{name}}}$")
    ax.set_ylabel("Current (p.u.)")
    ax.set_xlabel("Time (s)")
    ax.set_xlim(0, T_STOP)

    for ax in axs:
        ax.legend(loc="center left", bbox_to_anchor=(1.0, 0.5))
        ax.yaxis.set_label_coords(-0.07, 0.5)
    fig.align_ylabels(axs)
    fig.tight_layout()
    return fig


def add_reveal_animation(svg: str) -> str:
    """Animate the axes clip rectangles to reveal the curves from left to right."""

    def animate(match):
        rect = match.group(0)
        width = re.search(r'width="([\d.]+)"', rect).group(1)
        anim = (
            f'<animate attributeName="width" from="0" to="{width}" '
            f'dur="{ANIM_DUR}s" fill="freeze" calcMode="linear"/>'
        )
        return rect.replace("/>", f">{anim}</rect>")

    return re.sub(
        r"(?<=<clipPath id=\"p[0-9a-f]{10}\">\n)\s*<rect [^>]*/>", animate, svg
    )


# %%
if __name__ == "__main__":
    res, base = simulate()
    for theme in THEMES:
        fig = plot(res, base, theme)
        path = OUT_DIR / f"hero-{theme}.svg"
        fig.savefig(path, transparent=True, metadata={"Date": None})
        plt.close(fig)
        svg = add_reveal_animation(path.read_text())
        # Strip trailing whitespace to keep the pre-commit hooks happy
        path.write_text("\n".join(line.rstrip() for line in svg.splitlines()) + "\n")
        print(f"Wrote {path} ({path.stat().st_size / 1e3:.0f} kB)")
