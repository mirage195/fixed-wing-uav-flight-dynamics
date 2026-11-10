"""Shared validated continuous-time state-space model container."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

FloatMatrix = NDArray[np.float64]


@dataclass(frozen=True)
class StateSpaceModel:
    A: FloatMatrix
    B: FloatMatrix
    C: FloatMatrix
    D: FloatMatrix
    state_names: tuple[str, ...]
    input_names: tuple[str, ...]
    output_names: tuple[str, ...]
    name: str

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("state-space model name cannot be empty")
        for names, label in (
            (self.state_names, "state"),
            (self.input_names, "input"),
            (self.output_names, "output"),
        ):
            if not names or any(not name.strip() for name in names):
                raise ValueError(f"{label} names cannot be empty")
            if len(set(names)) != len(names):
                raise ValueError(f"{label} names must be unique")
        A = np.asarray(self.A, dtype=float)
        B = np.asarray(self.B, dtype=float)
        C = np.asarray(self.C, dtype=float)
        D = np.asarray(self.D, dtype=float)
        state_count = len(self.state_names)
        input_count = len(self.input_names)
        output_count = len(self.output_names)

        expected_shapes = {
            "A": (state_count, state_count),
            "B": (state_count, input_count),
            "C": (output_count, state_count),
            "D": (output_count, input_count),
        }
        for matrix_name, matrix in (("A", A), ("B", B), ("C", C), ("D", D)):
            if matrix.shape != expected_shapes[matrix_name]:
                raise ValueError(
                    f"{matrix_name} has shape {matrix.shape}; "
                    f"expected {expected_shapes[matrix_name]}"
                )
            if not np.all(np.isfinite(matrix)):
                raise ValueError(f"{matrix_name} contains non-finite values")
            matrix = matrix.copy()
            matrix.setflags(write=False)
            object.__setattr__(self, matrix_name, matrix)

    @property
    def eigenvalues(self) -> NDArray[np.complex128]:
        return np.linalg.eigvals(self.A)
