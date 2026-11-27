"""Manipulate flux linkage and current lookup tables of synchronous machines."""

from bisect import bisect_right
from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from typing import ClassVar, Literal

import numpy as np
from scipy.interpolate import RegularGridInterpolator, griddata
from scipy.io import loadmat


# %%
def _interp_bilinear(
    d_grid: list[float], q_grid: list[float], values: np.ndarray, x: float, y: float
) -> complex:
    """
    Bilinear interpolation at a single point on a strictly ascending grid.

    This gives the same result as `RegularGridInterpolator` with `method="linear"` and
    `fill_value=None`, i.e., the edge cells are extrapolated linearly, but avoids its
    overhead for a single point.

    """
    i = min(max(bisect_right(d_grid, x) - 1, 0), len(d_grid) - 2)
    j = min(max(bisect_right(q_grid, y) - 1, 0), len(q_grid) - 2)
    t_d = (x - d_grid[i]) / (d_grid[i + 1] - d_grid[i])
    t_q = (y - q_grid[j]) / (q_grid[j + 1] - q_grid[j])
    v00, v01 = values[i, j].item(), values[i, j + 1].item()
    v10, v11 = values[i + 1, j].item(), values[i + 1, j + 1].item()
    v0 = v00 + t_q * (v01 - v00)
    v1 = v10 + t_q * (v11 - v10)
    return complex(v0 + t_d * (v1 - v0))


class _GridInterpolator:
    """
    Linear interpolator of a map on a regular grid, accepting complex input (d + j*q).

    The interpolator extrapolates linearly outside the grid. It is a class instead of a
    closure, so that the models and controllers using the map can be pickled.

    """

    def __init__(
        self, d_range: np.ndarray, q_range: np.ndarray, values: np.ndarray
    ) -> None:
        self.interpolator = RegularGridInterpolator(
            (d_range, q_range),
            values,
            method="linear",
            bounds_error=False,
            fill_value=None,  # type: ignore
        )
        self.values = values
        # Faster interpolation of single points on a strictly ascending grid
        self.d_grid = [float(v) for v in np.asarray(d_range)]
        self.q_grid = [float(v) for v in np.asarray(q_range)]
        self.fast = all(
            len(g) > 1 and np.all(np.diff(g) > 0) for g in (self.d_grid, self.q_grid)
        )

    def __call__(self, dq_input: complex | np.ndarray) -> complex | np.ndarray:
        """Interpolate at the input point(s) given in complex form (d + j*q)."""
        if np.isscalar(dq_input):
            # Handle scalar complex input
            x, y = dq_input.real, dq_input.imag  # type: ignore
            if self.fast:
                return _interp_bilinear(self.d_grid, self.q_grid, self.values, x, y)
            return complex(self.interpolator(np.array([[x, y]]))[0])
        # Handle arrays of complex inputs, the output having the shape of the input
        points = np.stack((np.real(dq_input), np.imag(dq_input)), axis=-1)
        return np.array(self.interpolator(points))


# %%
@dataclass
class MagneticModel:
    """
    Store and manipulate flux linkage or current maps for synchronous machines.

    Parameters
    ----------
    i_s_dq : np.ndarray
        Complex array of stator current (A).
    psi_s_dq : np.ndarray
        Complex array of stator flux linkage (Vs).
    lookup_fcn : Callable[[complex | np.ndarray], complex | np.ndarray], optional
        Function that evaluates the map at arbitrary points. Takes complex inputs (d +
        j*q) and returns the output values. For flux maps, maps i_s_dq → psi_s_dq; for
        current maps, maps psi_s_dq → i_s_dq. If not given, a linear interpolation
        function, which extrapolates outside the map range, is created from the data.
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
            raise ValueError("Lookup table function is missing")
        return self.lookup_fcn(input_dq)

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
        d_range : np.ndarray | None, optional
            Range for the d axis. If None, the range is determined from the data,
            defaults to None.
        q_range : np.ndarray | None, optional
            Range for the q axis. If None, the range is determined from the data,
            defaults to None.
        num : int | None, optional
            Number of points in each axis. If None, uses the maximum dimension from the
            original map to preserve resolution, defaults to None.
        invert : bool, optional
            Invert the map (swap input and output), defaults to False.

        Returns
        -------
        MagneticModel
            Interpolated magnetic model.

        """
        if self.type == "flux_map":
            inp, out = self.i_s_dq, self.psi_s_dq
        else:
            inp, out = self.psi_s_dq, self.i_s_dq
        new_type = self.type

        # Set default num value based on input data if not provided
        if num is None:
            num = int(max(self.i_s_dq.shape))

        if invert:
            inp, out = out, inp
            new_type = "current_map" if self.type == "flux_map" else "flux_map"

        # Auto-determine ranges if not provided
        if inp.ndim == 2:
            d_min = np.max(np.min(np.real(inp), axis=0))
            d_max = np.min(np.max(np.real(inp), axis=0))
            q_min = np.max(np.min(np.imag(inp), axis=1))
            q_max = np.min(np.max(np.imag(inp), axis=1))
        else:
            d_min = np.min(np.real(inp))
            d_max = np.max(np.real(inp))
            q_min = np.min(np.imag(inp))
            q_max = np.max(np.imag(inp))

        d_range = np.linspace(d_min, d_max, num) if d_range is None else d_range
        q_range = np.linspace(q_min, q_max, num) if q_range is None else q_range

        # Interpolate the map
        d_grid, q_grid = np.meshgrid(d_range, q_range, indexing="ij")
        new_inp = d_grid + 1j * q_grid
        points = (np.real(inp).ravel(), np.imag(inp).ravel())
        new_out = griddata(points, out.ravel(), (np.real(new_inp), np.imag(new_inp)))

        # Interpolate torque if available
        new_tau_M = None
        if self.tau_M is not None:
            new_tau_M = griddata(
                points, self.tau_M.ravel(), (np.real(new_inp), np.imag(new_inp))
            )

        lookup_fcn = _GridInterpolator(d_range, q_range, new_out)

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
        self,
        d_range: np.ndarray | None = None,
        q_range: np.ndarray | None = None,
        num: int | None = None,
    ) -> "MagneticModel":
        """
        Invert the map (swap input and output).

        Parameters
        ----------
        d_range : np.ndarray | None, optional
            Range for the d axis. If None, the range is determined from the data,
            defaults to None.
        q_range : np.ndarray | None, optional
            Range for the q axis. If None, the range is determined from the data,
            defaults to None.
        num : int | None, optional
            Number of points in each axis. If None, uses the maximum dimension from the
            original map to preserve resolution, defaults to None.

        Returns
        -------
        MagneticModel
            Inverted magnetic model on a regular grid.

        """
        return self.create_interpolated_model(d_range, q_range, num, invert=True)


# %%
class SaturationModelBase:
    """
    Base class for saturation models given as functions.

    A subclass implements `__call__`, which evaluates the map: the current as a function
    of the flux linkage for a current map, or the flux linkage as a function of the
    current for a flux map, as declared by `map_type`. The model can be used directly as
    the lookup function of a `MagneticModel`, and `as_magnetic_model` samples it on a
    grid.

    """

    map_type: ClassVar[Literal["current_map", "flux_map"]] = "current_map"

    def __call__(self, x_dq: complex | np.ndarray) -> complex | np.ndarray:
        """Evaluate the map."""
        raise NotImplementedError

    def as_magnetic_model(
        self, d_range: np.ndarray, q_range: np.ndarray, n_p: int | None = None
    ) -> MagneticModel:
        """
        Sample the map on a grid as a magnetic model.

        Parameters
        ----------
        d_range : ndarray
            Range for the d-axis input. For current maps, this is psi_d (Vs). For flux
            maps, this is i_d (A).
        q_range : ndarray
            Range for the q-axis input. For current maps, this is psi_q (Vs). For flux
            maps, this is i_q (A).
        n_p : int, optional
            Number of pole pairs. If provided, the torque is included.

        Returns
        -------
        MagneticModel
            Magnetic model using this object as its `lookup_fcn`.

        """
        d_grid, q_grid = np.meshgrid(d_range, q_range, indexing="ij")
        inp_dq = d_grid + 1j * q_grid
        out_dq = np.asarray(self(inp_dq), dtype=complex)
        if self.map_type == "flux_map":
            i_s_dq, psi_s_dq = inp_dq, out_dq
        else:
            i_s_dq, psi_s_dq = out_dq, inp_dq
        tau_M = None
        if n_p is not None:
            tau_M = 1.5 * n_p * np.imag(i_s_dq * np.conj(psi_s_dq))
        return MagneticModel(
            i_s_dq=i_s_dq,
            psi_s_dq=psi_s_dq,
            lookup_fcn=self,
            tau_M=tau_M,
            type=self.map_type,
        )


@dataclass
class SaturationModelSyRM(SaturationModelBase):
    """
    Saturation model for synchronous reluctance machines.

    This model takes into account the self- and cross-saturation effects of the d and q
    axes [#Hin2017]_.

    Parameters
    ----------
    a_d0 : float
        Offset coefficient for the d-axis inverse inductance.
    a_dd : float
        Self-saturation coefficient for the d axis.
    a_q0 : float
        Offset coefficient for the q-axis inverse inductance.
    a_qq : float
        Self-saturation coefficient for the q axis.
    a_dq : float
        Cross-saturation coefficient.
    S : float
        Exponent for the d-axis self-saturation.
    T : float
        Exponent for the q-axis self-saturation.
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
            + self.a_dd * np.abs(np.real(psi_s_dq)) ** self.S
            + (
                self.a_dq
                / (self.V + 2)
                * np.abs(np.real(psi_s_dq)) ** self.U
                * np.abs(np.imag(psi_s_dq)) ** (self.V + 2)
            )
        )
        G_q = (
            self.a_q0
            + self.a_qq * np.abs(np.imag(psi_s_dq)) ** self.T
            + (
                self.a_dq
                / (self.U + 2)
                * np.abs(np.real(psi_s_dq)) ** (self.U + 2)
                * np.abs(np.imag(psi_s_dq)) ** self.V
            )
        )
        i_s_dq = G_d * np.real(psi_s_dq) + 1j * G_q * np.imag(psi_s_dq)
        return i_s_dq


@dataclass
class SaturationModelPMSyRM(SaturationModelSyRM):
    """
    Saturation model for PM synchronous reluctance machines.

    This model takes into account the bridge saturation in addition to the self- and
    cross-saturation effects of the d and q axes [#Lel2024]_. The bridge saturation
    model is based on a nonlinear reluctance element in parallel with the
    Norton-equivalent PM model. The other parameters are those of `SaturationModelSyRM`.

    Parameters
    ----------
    psi_n : float
        Flux linkage (Vs) of the Norton-equivalent PM model, i.e., the d-axis flux
        linkage at which the bridge current ``G_b*(psi_d - psi_n)`` vanishes. It is
        higher than the PM flux linkage at zero current.
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
        psi_b = np.real(psi_s_dq) - self.psi_n
        # State of the bridge saturation depends also on the q-axis flux
        psi_b_sat = np.sqrt(psi_b**2 + self.k_q * np.imag(psi_s_dq) ** 2)
        # Inverse inductance function for the bridge saturation
        G_b = self.a_b * psi_b_sat**self.W / (1 + self.a_bp * psi_b_sat**self.W)
        # Stator current
        i_s_dq += G_b * psi_b + 1j * self.k_q * G_b * np.imag(psi_s_dq)
        return i_s_dq


# %%
def import_syre_data(
    fname: Path | str, add_negative_q_axis: bool = True
) -> MagneticModel:
    """
    Import a flux map from the MATLAB data file in the SyR-e format.

    For more information on the SyR-e project and the MATLAB file format, please visit:

        https://github.com/SyR-e/syre_public

    The imported data is converted to the PMSM coordinate convention, in which the PM
    flux is along the d axis.

    Parameters
    ----------
    fname : Path | str
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
    flux_map = loadmat(fname)["motorModel"][0, 0]["FluxMap_dq"][0, 0]
    # Use the PMSM convention in coordinates
    i_d, i_q = -flux_map["Iq"], flux_map["Id"]
    psi_d, psi_q = -flux_map["Fq"], flux_map["Fd"]
    tau_M = flux_map["T"]

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
