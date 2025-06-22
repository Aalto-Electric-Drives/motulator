"""Manipulate flux linkage and current maps of synchronous machines."""

from dataclasses import dataclass
from typing import Callable, Literal, cast

import numpy as np
from scipy.interpolate import RegularGridInterpolator, griddata
from scipy.io import loadmat


# %%
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
    lookup_fcn : Callable[[complex | np.ndarray], complex | np.ndarray], optional
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
        d_range : np.ndarray | None, optional
            Range for the d-axis. If None, the range is determined from the data,
            defaults to None.
        q_range : np.ndarray | None, optional
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
            num = int(max(self.i_s_dq.shape))

        if invert:
            inp, out = out, inp
            new_type = "current_map" if self.is_flux_map() else "flux_map"

        # Auto-determine ranges if not provided
        d_min = np.max(np.min(np.real(inp), axis=0))
        d_max = np.min(np.max(np.real(inp), axis=0))
        q_min = np.max(np.min(np.imag(inp), axis=1))
        q_max = np.min(np.max(np.imag(inp), axis=1))

        if d_range is None:
            d_range = np.linspace(d_min, d_max, num)
        if q_range is None:
            q_range = np.linspace(q_min, q_max, num)

        # Interpolate the map
        new_inp = create_grid(d_range, q_range)  # type: ignore
        points = (np.real(inp).ravel(), np.imag(inp).ravel())
        new_out = griddata(points, out.ravel(), (np.real(new_inp), np.imag(new_inp)))

        # Interpolate torque if available
        new_tau_M = None
        if self.tau_M is not None:
            new_tau_M = griddata(
                points, self.tau_M.ravel(), (np.real(new_inp), np.imag(new_inp))
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
            dq_input : complex | np.ndarray
                Input values in complex form (d + j*q).

            Returns
            -------
            np.ndarray
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
            points = np.column_stack((np.real(dq_input), np.imag(dq_input)))
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
            Range for the d-axis. If None, the range is determined from the data,
            defaults to None.
        q_range : np.ndarray | None, optional
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
        psi_b = np.real(psi_s_dq) - self.psi_n
        # State of the bridge saturation depends also on the q-axis flux
        psi_b_sat = np.sqrt(psi_b**2 + self.k_q * np.imag(psi_s_dq) ** 2)
        # Inverse inductance function for the bridge saturation
        G_b = self.a_b * psi_b_sat**self.W / (1 + self.a_bp * psi_b_sat**self.W)
        # Stator current
        i_s_dq += G_b * psi_b + 1j * self.k_q * G_b * np.imag(psi_s_dq)
        return i_s_dq


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
