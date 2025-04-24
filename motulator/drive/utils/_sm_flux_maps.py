"""Manipulate and plot flux linkage maps of synchronous machines."""

from dataclasses import dataclass
from typing import Any, Callable, Literal, cast

import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import RegularGridInterpolator, griddata
from scipy.io import loadmat

from motulator.common.utils._utils import BaseValues, set_latex_style, set_screen_style


@dataclass
class MagneticModel:
    """
    Store and manipulate flux linkage or current maps for synchronous machines.

    Attributes
    ----------
    i_s_dq : np.ndarray
        Complex array of stator current (A).
    psi_s_dq : np.ndarray
        Complex array of stator flux linkage (Vs).
    lookup_fcn : callable, optional
        Linear interpolation function that evaluates the map at arbitrary points. Takes
        complex inputs (d + j*q) and returns interpolated output values. For flux maps,
        maps i_s_dq → psi_s_dq; for current maps, maps psi_s_dq → i_s_dq. The function
        extrapolates outside the map range.
    tau_M : np.ndarray, optional
        Array of electromagnetic torque (Nm).
    type : Literal["current_map", "flux_map"], optional
        Type of the map, defaults to "flux_map".

    """

    i_s_dq: np.ndarray
    psi_s_dq: np.ndarray
    lookup_fcn: Callable[[complex | np.ndarray], complex | np.ndarray] | None = None
    tau_M: np.ndarray | None = None
    type: Literal["current_map", "flux_map"] = "flux_map"

    def __post_init__(self) -> None:
        """Ensure lookup_fcn is available by creating it if needed."""
        if self.lookup_fcn is None:
            interpolated = self.create_interpolated_model()
            self.lookup_fcn = interpolated.lookup_fcn

    def __call__(self, input_dq: complex | np.ndarray) -> complex | np.ndarray:
        """
        Evaluate the magnetic model mapping at the given input.

        Parameters
        ----------
        input_dq : complex | np.ndarray
            Input values in complex form (d + j*q). For flux maps, this is current (A).
            For current maps, this is flux linkage (Vs).

        Returns
        -------
        complex | np.ndarray
            Output values corresponding to the inputs. For flux maps, this is flux
            linkage (Vs). For current maps, this is current (A).

        """
        if self.lookup_fcn is None:
            raise ValueError
        return self.lookup_fcn(input_dq)

    def is_flux_map(self) -> bool:
        """Return True if this is a flux map (i_s → psi_s)."""
        return self.type != "current_map"

    def is_current_map(self) -> bool:
        """Return True if this is a current map (psi_s → i_s)."""
        return self.type == "current_map"

    def get_input_output(self) -> tuple[np.ndarray, np.ndarray]:
        """Get input and output arrays based on map type."""
        if self.is_flux_map():
            return self.i_s_dq, self.psi_s_dq
        return self.psi_s_dq, self.i_s_dq

    def create_interpolated_model(
        self,
        d_range: np.ndarray | None = None,
        q_range: np.ndarray | None = None,
        num: int | None = None,
        invert: bool = False,
    ) -> "MagneticModel":
        """
        Interpolate or invert this magnetic model onto a regular grid.

        Parameters
        ----------
        d_range : Any, optional
            Range for the d-axis. If None, the range is determined from the data,
            defaults to None.
        q_range : Any, optional
            Range for the q-axis. If None, the range is determined from the data,
            defaults to None.
        num : int, optional
            Number of points in each axis. If None, uses the maximum dimension from the
            original map to preserve resolution, defaults to None.
        invert : bool, optional
            Invert the map (swap input and output), defaults to False.

        Returns
        -------
        MagneticModel
            Interpolated magnetic model.

        """

        def create_grid(d_range: np.ndarray, q_range: np.ndarray) -> np.ndarray:
            d_grid, q_grid = np.meshgrid(d_range, q_range, indexing="ij")
            return d_grid + 1j * q_grid

        # Get input/output arrays based on map type
        inp, out = self.get_input_output()
        new_type = self.type

        # Set default num value based on input data if not provided
        if num is None:
            num = max(self.i_s_dq.shape)

        if invert:
            inp, out = out, inp
            new_type = "current_map" if self.is_flux_map() else "flux_map"

        # Auto-determine ranges if not provided
        psi_d_min = np.max(np.min(inp.real, axis=0))
        psi_d_max = np.min(np.max(inp.real, axis=0))
        psi_q_min = np.max(np.min(inp.imag, axis=1))
        psi_q_max = np.min(np.max(inp.imag, axis=1))

        if d_range is None:
            d_range = np.linspace(psi_d_min, psi_d_max, num)
        if q_range is None:
            q_range = np.linspace(psi_q_min, psi_q_max, num)

        # Interpolate the map
        new_inp = create_grid(d_range, q_range)  # type: ignore
        points = (inp.real.ravel(), inp.imag.ravel())
        new_out = griddata(points, out.ravel(), (new_inp.real, new_inp.imag))

        # Interpolate torque if available
        new_tau_M = None
        if self.tau_M is not None:
            new_tau_M = griddata(
                points, self.tau_M.ravel(), (new_inp.real, new_inp.imag)
            )

        # Create interpolator
        interpolator = RegularGridInterpolator(
            (d_range, q_range),
            new_out,
            method="linear",
            bounds_error=False,
            fill_value=None,  # type: ignore
        )

        # Create a wrapper function that accepts complex inputs
        def lookup_fcn(dq_input: complex | np.ndarray) -> complex | np.ndarray:
            """
            Interpolator that accepts complex input directly.

            Parameters
            ----------
            dq_input : complex or ndarray of complex
                Input values in complex form (d + j*q).

            Returns
            -------
            Output value(s) corresponding to the input(s).

            """
            if np.isscalar(dq_input) or isinstance(dq_input, complex):
                # Handle scalar complex input
                points = np.array(
                    [
                        [
                            dq_input.real,  # type: ignore
                            dq_input.imag,  # type: ignore
                        ]
                    ]
                )
                return complex(interpolator(points)[0])
            # Handle array of complex inputs
            points = np.column_stack((dq_input.real, dq_input.imag))
            return np.array(interpolator(points))

        # Arrange data based on map type
        if new_type == "flux_map":
            new_i_s_dq, new_psi_s_dq = new_inp, new_out
        else:
            new_i_s_dq, new_psi_s_dq = new_out, new_inp

        return MagneticModel(
            i_s_dq=new_i_s_dq,
            psi_s_dq=new_psi_s_dq,
            lookup_fcn=lookup_fcn,
            tau_M=new_tau_M,
            type=new_type,
        )

    def invert(
        self, d_range: Any = None, q_range: Any = None, num: int | None = None
    ) -> "MagneticModel":
        """
        Invert the map (swap input and output).

        Parameters
        ----------
        d_range : Any, optional
            Range for the d-axis. If None, the range is determined from the data,
            defaults to None.
        q_range : Any, optional
            Range for the q-axis. If None, the range is determined from the data,
            defaults to None.
        num : int, optional
            Number of points in each axis. If None, uses the maximum dimension from the
            original map to preserve resolution, defaults to None.

        """
        if num is None:
            # Extract size from the shape of the original arrays
            num = max(self.i_s_dq.shape)

        return self.create_interpolated_model(d_range, q_range, num, invert=True)


# %%
@dataclass
class SaturationModelBase:
    """
    Base class for analytical saturation models.

    This class implements a callable interface that maps flux linkage to current,
    matching the interface of MagneticModel.lookup_fcn for current maps. It can be used
    directly as a lookup function for a MagneticModel.

    """

    def __call__(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """
        Calculate the stator current from flux linkage (psi_s_dq → i_s_dq).

        This is equivalent to the lookup_fcn of a current map MagneticModel. Must be
        implemented by subclasses.

        Parameters
        ----------
        psi_s_dq : complex | np.ndarray
            Stator flux linkage (Vs) in complex form (d + j*q).

        Returns
        -------
        complex | np.ndarray
            Stator current (A) in complex form (d + j*q).

        """
        raise NotImplementedError

    def as_magnetic_model(
        self, d_range: np.ndarray, q_range: np.ndarray, n_p: int | None = None
    ) -> MagneticModel:
        """
        Create a magnetic model that uses this analytical model.

        Parameters
        ----------
        d_range : ndarray
            Range for the d-axis flux linkage (Vs).
        q_range : ndarray
            Range for the q-axis flux linkage (Vs).
        n_p : int, optional
            Number of pole pairs. If provided, the torque is included.

        Returns
        -------
        MagneticModel
            Current map that uses this saturation model as its lookup_fcn.

        """
        # Create grid of flux values
        psi_d_grid, psi_q_grid = np.meshgrid(d_range, q_range, indexing="ij")
        psi_s_dq = psi_d_grid + 1j * psi_q_grid

        # Calculate corresponding currents
        i_s_dq = cast(np.ndarray, self(psi_s_dq))

        # Calculate torque if needed
        tau_M = None
        if n_p is not None:
            tau_M = 1.5 * n_p * np.imag(i_s_dq * np.conj(psi_s_dq))

        # Magnetic model with this saturation model as the lookup function
        return MagneticModel(
            psi_s_dq=psi_s_dq,
            i_s_dq=i_s_dq,
            lookup_fcn=self,  # Use the model's __call__ directly
            tau_M=tau_M,
            type="current_map",
        )


@dataclass
class SaturationModelSyRM(SaturationModelBase):
    """
    Saturation model for synchronous reluctance machines.

    This model takes into account the self- and cross-saturation effects of the d- and
    q-axis [#Hin2017]_.

    Attributes
    ----------
    a_d0 : float
        Offset coefficient for d-axis inverse inductance.
    a_dd : float
        Self-saturation coefficient for d-axis.
    a_q0 : float
        Offset coefficient for q-axis inverse inductance.
    a_qq : float
        Self-saturation coefficient for q-axis.
    a_dq : float
        Cross-saturation coefficient.
    S : float
        Exponent for d-axis self-saturation.
    T : float
        Exponent for q-axis self-saturation.
    U : float
        First exponent for cross-saturation.
    V : float
        Second exponent for cross-saturation.

    References
    ----------
    .. [#Hin2017] Hinkkanen, Pescetto, Mölsä, Saarakkala, Pellegrino, Bojoi, "Sensorless
       self-commissioning of synchronous reluctance motors at standstill without rotor
       locking," IEEE Trans. Ind. Appl., 2017, https://doi.org/10.1109/TIA.2016.2644624

    """

    a_d0: float
    a_dd: float
    a_q0: float
    a_qq: float
    a_dq: float
    S: float
    T: float
    U: float
    V: float

    def __call__(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Calculate the stator current for SyRMs."""
        G_d = (
            self.a_d0
            + self.a_dd * np.abs(psi_s_dq.real) ** self.S
            + (
                self.a_dq
                / (self.V + 2)
                * np.abs(psi_s_dq.real) ** self.U
                * np.abs(psi_s_dq.imag) ** (self.V + 2)
            )
        )
        G_q = (
            self.a_q0
            + self.a_qq * np.abs(psi_s_dq.imag) ** self.T
            + (
                self.a_dq
                / (self.U + 2)
                * np.abs(psi_s_dq.real) ** (self.U + 2)
                * np.abs(psi_s_dq.imag) ** self.V
            )
        )
        i_s_dq = G_d * psi_s_dq.real + 1j * G_q * psi_s_dq.imag
        return i_s_dq


@dataclass
class SaturationModelPMSyRM(SaturationModelSyRM):
    """
    Saturation model for PM synchronous reluctance machines.

    This model takes into account the bridge saturation in addition to the self- and
    cross-saturation effects of the d- and q-axis [#Lel2024]_. The bridge saturation
    model is based on a nonlinear reluctance element in parallel with the Norton-
    equivalent PM model.

    Attributes
    ----------
    psi_n : float
        PM flux linkage (Vs).
    a_b : float
        Coefficient for bridge inverse inductance.
    a_bp : float
        Coefficient for bridge saturation.
    W : float
        Exponent for bridge saturation.
    k_q : float
        Cross-coupling factor for bridge flux.

    References
    ----------
    .. [#Lel2024] Lelli, Hinkkanen, Giulii Capponi, "A saturation model based on a
       simplified equivalent magnetic circuit for permanent magnet machines," Proc.
       ICEM, 2024, https://doi.org/10.1109/ICEM60801.2024.10700403

    """

    psi_n: float
    a_b: float
    a_bp: float
    W: float
    k_q: float

    def __call__(self, psi_s_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Calculate the stator current for PM-SyRMs."""
        # Calculate base SyRM saturation
        i_s_dq = super().__call__(psi_s_dq)

        # Bridge flux
        psi_b = psi_s_dq.real - self.psi_n
        # State of the bridge saturation depends also on the q-axis flux
        psi_b_sat = np.sqrt(psi_b**2 + self.k_q * psi_s_dq.imag**2)
        # Inverse inductance function for the bridge saturation
        G_b = self.a_b * psi_b_sat**self.W / (1 + self.a_bp * psi_b_sat**self.W)
        # Stator current
        i_s_dq += G_b * psi_b + 1j * self.k_q * G_b * psi_s_dq.imag
        return i_s_dq


# %%
# pyright: reportPrivateImportUsage=false
def plot_maps(
    data: MagneticModel,
    base: BaseValues | None = None,
    x_lims: tuple[float, float] | None = None,
    y_lims: tuple[float, float] | None = None,
    z_lims: tuple[float, float] | None = None,
    raw_data: MagneticModel | None = None,
    latex: bool = False,
) -> None:
    # ruff: noqa: PLR0912, PLR0913, PLR0915
    """
    Plot flux maps and current maps.

    Parameters
    ----------
    data : MagneticModel
        Data containing the flux and current information. The coordinates are selected
        based on the `type` field, which is either "flux_map" or "current_map" (the
        default is "flux_map").
    base : BaseValues, optional
        Base values for scaling the maps.
    x_lims : tuple, optional
        Range for the x-axis as (min, max). If None, the range is determined from the
        data, defaults to None.
    y_lims : tuple, optional
        Range for the y-axis as (min, max). If None, the range is determined from the
        data, defaults to None.
    z_lims : tuple, optional
        Range for the z-axis as (min, max). If None, the range is determined from the
        data, defaults to None.
    raw_data : MagneticModel, optional
        Flux and current information for comparison..
    latex : bool, optional
        Use LaTeX fonts for the labels. Enabling this option requires a working LaTeX
        installation, defaults to False.

    """
    if latex:
        set_latex_style()
        width = plt.rcParams["figure.figsize"][0] * 1.4
        height = plt.rcParams["figure.figsize"][1] * 1.4
        size = (width, height)
        plt.rcParams.update({"savefig.pad_inches": 0.3})
    else:
        set_screen_style()
        size = plt.rcParams["figure.figsize"]

    # Check if the base values were given
    pu_vals = base is not None
    if base is None:
        base = BaseValues.unity()

    # Normalize the data
    i_s_dq = data.i_s_dq / base.i
    psi_s_dq = data.psi_s_dq / base.psi

    fig1, ax1 = plt.subplots(figsize=size, subplot_kw={"projection": "3d"})
    fig2, ax2 = plt.subplots(figsize=size, subplot_kw={"projection": "3d"})

    if data.type == "flux_map":
        x, y, z1, z2 = i_s_dq.real, i_s_dq.imag, psi_s_dq.real, psi_s_dq.imag

        if raw_data is not None:
            raw_i_s_dq = raw_data.i_s_dq / base.i
            raw_psi_s_dq = raw_data.psi_s_dq / base.psi
            ax1.scatter(
                raw_i_s_dq.real,
                raw_i_s_dq.imag,
                raw_psi_s_dq.real,
                marker=".",
                color="r",
            )
            ax2.scatter(
                raw_i_s_dq.real,
                raw_i_s_dq.imag,
                raw_psi_s_dq.imag,
                marker=".",
                color="r",
            )

        if pu_vals:
            xlabel = r"$i_\mathrm{d}$ (p.u.)"
            ylabel = r"$i_\mathrm{q}$ (p.u.)"
            zlabel1 = r"$\psi_\mathrm{d}$ (p.u.)"
            zlabel2 = r"$\psi_\mathrm{q}$ (p.u.)"
        else:
            xlabel = r"$i_\mathrm{d}$ (A)"
            ylabel = r"$i_\mathrm{q}$ (A)"
            zlabel1 = r"$\psi_\mathrm{d}$ (Vs)"
            zlabel2 = r"$\psi_\mathrm{q}$ (Vs)"

    else:
        x, y, z1, z2 = psi_s_dq.real, psi_s_dq.imag, i_s_dq.real, i_s_dq.imag

        if raw_data is not None:
            raw_i_s_dq = raw_data.i_s_dq / base.i
            raw_psi_s_dq = raw_data.psi_s_dq / base.psi
            ax1.scatter(
                raw_psi_s_dq.real,
                raw_psi_s_dq.imag,
                raw_i_s_dq.real,
                marker=".",
                color="r",
            )
            ax2.scatter(
                raw_psi_s_dq.real,
                raw_psi_s_dq.imag,
                raw_i_s_dq.imag,
                marker=".",
                color="r",
            )

        if pu_vals:
            xlabel = r"$\psi_\mathrm{d}$ (p.u.)"
            ylabel = r"$\psi_\mathrm{q}$ (p.u.)"
            zlabel1 = r"$i_\mathrm{d}$ (p.u.)"
            zlabel2 = r"$i_\mathrm{q}$ (p.u.)"
        else:
            xlabel = r"$\psi_\mathrm{d}$ (Vs)"
            ylabel = r"$\psi_\mathrm{q}$ (Vs)"
            zlabel1 = r"$i_\mathrm{d}$ (A)"
            zlabel2 = r"$i_\mathrm{q}$ (A)"

    ax1.plot_surface(  # type: ignore
        x, y, z1, cmap="viridis", alpha=0.75, axlim_clip=True
    )
    ax2.plot_surface(  # type: ignore
        x, y, z2, cmap="viridis", alpha=0.75, axlim_clip=True
    )

    ax1.set_xlabel(xlabel)
    ax1.set_ylabel(ylabel)
    ax1.set_zlabel(zlabel1)  # type: ignore
    ax2.set_xlabel(xlabel)
    ax2.set_ylabel(ylabel)
    ax2.set_zlabel(zlabel2)  # type: ignore

    if x_lims is not None:
        ax1.set_xlim(x_lims)
        ax2.set_xlim(x_lims)
    if y_lims is not None:
        ax1.set_ylim(y_lims)
        ax2.set_ylim(y_lims)
    if z_lims is not None:
        ax1.set_zlim(z_lims)  # type: ignore
        ax2.set_zlim(z_lims)  # type: ignore

    plt.show()


# %%
def plot_flux_vs_current(
    data: MagneticModel,
    base: BaseValues | None = None,
    lims: tuple[float, float] | None = None,
    latex: bool = False,
) -> None:
    """
    Plot the flux vs. current characteristics.

    Parameters
    ----------
    data : MagneticModel
        Flux map data. The current array should be a rectilinear grid.
    base : BaseValues, optional
        Base values for scaling the maps.
    lims : tuple, optional
        Range for the x-axis as (min, max). If None, determined from the data.
    latex : bool, optional
        Use LaTeX fonts for the labels, requires a working LaTeX installation.

    """
    if latex:
        set_latex_style()
    else:
        set_screen_style()

    # Check if the base values were given
    pu_vals = base is not None
    if base is None:
        base = BaseValues.unity()

    # Normalize the data
    i_s_dq = data.i_s_dq / base.i
    psi_s_dq = data.psi_s_dq / base.psi

    # Indices corresponding to i_d = 0 and i_q = 0
    ind_d_0 = np.argmin(np.abs(i_s_dq.real[:, 0]))
    ind_q_0 = np.argmin(np.abs(i_s_dq.imag[0, :]))
    # Indices corresponding to min(i_d) and max(i_d)
    if lims is None:
        ind_d_min = np.argmin(i_s_dq.real[:, 0])
        ind_d_max = np.argmax(i_s_dq.real[:, 0])
    else:
        ind_d_min = np.argmin(np.abs(i_s_dq.real[:, 0] - lims[0]))
        ind_d_max = np.argmin(np.abs(i_s_dq.real[:, 0] - lims[1]))

    fig, ax = plt.subplots()
    ax.plot(
        i_s_dq.real[:, ind_q_0],
        psi_s_dq.real[:, ind_q_0],
        color="r",
        linestyle="-",
        label=r"$\psi_\mathrm{d}(i_\mathrm{d}, 0)$",
    )
    ax.plot(
        i_s_dq.real[:, -1],
        psi_s_dq.real[:, -1],
        color="r",
        linestyle="--",
        label=r"$\psi_\mathrm{d}(i_\mathrm{d}, i_\mathrm{q,max})$",
    )
    ax.plot(
        i_s_dq.imag[ind_d_0, :],
        psi_s_dq.imag[ind_d_0, :],
        color="b",
        linestyle="-",
        label=r"$\psi_\mathrm{q}(0, i_\mathrm{q})$",
    )
    ax.plot(
        i_s_dq.imag[ind_d_min, :],
        psi_s_dq.imag[ind_d_min, :],
        color="b",
        linestyle=":",
        label=r"$\psi_\mathrm{q}(i_\mathrm{d,min}, i_\mathrm{q})$",
    )
    ax.plot(
        i_s_dq.imag[ind_d_max, :],
        psi_s_dq.imag[ind_d_max, :],
        color="b",
        linestyle="--",
        label=r"$\psi_\mathrm{q}(i_\mathrm{d,max}, i_\mathrm{q})$",
    )

    if lims is not None:
        ax.set_xlim(lims)

    if pu_vals:
        ax.set_xlabel(r"$i_\mathrm{d}$, $i_\mathrm{q}$ (p.u.)")
        ax.set_ylabel(r"$\psi_\mathrm{d}$, $\psi_\mathrm{q}$ (p.u.)")
    else:
        ax.set_xlabel(r"$i_\mathrm{d}$, $i_\mathrm{q}$ (A)")
        ax.set_ylabel(r"$\psi_\mathrm{d}$, $\psi_\mathrm{q}$ (Vs)")
    ax.legend()

    plt.show()


# %%
def import_syre_data(fname: str, add_negative_q_axis: bool = True) -> MagneticModel:
    """
    Import a flux map from the MATLAB data file in the SyR-e format.

    For more information on the SyR-e project and the MATLAB file format, please visit:

        https://github.com/SyR-e/syre_public

    The imported data is converted to the PMSM coordinate convention, in which the PM
    flux is along the d axis.

    Parameters
    ----------
    fname : str
        MATLAB file name.
    add_negative_q_axis : bool, optional
        Adds the negative q-axis data based on the symmetry, defaults to True.

    Returns
    -------
    MagneticModel
        Magnetic model data.

    Notes
    -----
    Some example data files (including THOR.mat) are available in the SyR-e repository,
    licensed under the Apache License, Version 2.0.

    """
    # Read the data from mat-file
    mat = loadmat(fname)

    # Use the PMSM convention in coordinates.
    i_d = -mat["motorModel"][0, 0]["FluxMap_dq"][0, 0]["Iq"]
    i_q = mat["motorModel"][0, 0]["FluxMap_dq"][0, 0]["Id"]
    psi_d = -mat["motorModel"][0, 0]["FluxMap_dq"][0, 0]["Fq"]
    psi_q = mat["motorModel"][0, 0]["FluxMap_dq"][0, 0]["Fd"]
    tau_M = mat["motorModel"][0, 0]["FluxMap_dq"][0, 0]["T"]

    # Clip the negative q-axis values
    i_q = np.clip(i_q, 0, np.inf)
    psi_q = np.clip(psi_q, 0, np.inf)

    if add_negative_q_axis:
        # Add the negative q-axis data, flipped based on the symmetry
        i_d = np.concatenate((np.fliplr(i_d), i_d), axis=1)
        i_q = np.concatenate((-np.fliplr(i_q), i_q), axis=1)
        psi_d = np.concatenate((np.fliplr(psi_d), psi_d), axis=1)
        psi_q = np.concatenate((-np.fliplr(psi_q), psi_q), axis=1)
        tau_M = np.concatenate((-np.fliplr(tau_M), tau_M), axis=1)

    # Pack the data in the complex form
    i_s_dq = i_d + 1j * i_q
    psi_s_dq = psi_d + 1j * psi_q

    return MagneticModel(i_s_dq, psi_s_dq, tau_M=tau_M, type="flux_map")
