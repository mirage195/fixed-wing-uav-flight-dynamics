"""Continuous-time simulations for linear state-space models."""

from __future__ import annotations

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from numpy.typing import ArrayLike, NDArray
from scipy.signal import StateSpace, lsim

from .state_space import StateSpaceModel

FloatArray = NDArray[np.float64]


def _readonly_array(values: ArrayLike, *, dimensions: int, name: str) -> FloatArray:
    array = np.asarray(values, dtype=float)
    if array.ndim != dimensions:
        raise ValueError(f"{name} must have {dimensions} dimensions")
    if not np.all(np.isfinite(array)):
        raise ValueError(f"{name} contains non-finite values")
    array = array.copy()
    array.setflags(write=False)
    return array


@dataclass(frozen=True)
class SimulationResult:
    """Time, inputs, states, and outputs from one deterministic simulation."""

    name: str
    time: FloatArray
    inputs: FloatArray
    states: FloatArray
    outputs: FloatArray
    state_names: tuple[str, ...]
    input_names: tuple[str, ...]
    output_names: tuple[str, ...]

    def __post_init__(self) -> None:
        if not self.name.strip():
            raise ValueError("simulation name cannot be empty")
        time = _readonly_array(self.time, dimensions=1, name="time")
        inputs = _readonly_array(self.inputs, dimensions=2, name="inputs")
        states = _readonly_array(self.states, dimensions=2, name="states")
        outputs = _readonly_array(self.outputs, dimensions=2, name="outputs")
        sample_count = time.size
        if sample_count < 2 or not np.all(np.diff(time) > 0.0):
            raise ValueError("simulation time must be strictly increasing")
        expected_shapes = {
            "inputs": (sample_count, len(self.input_names)),
            "states": (sample_count, len(self.state_names)),
            "outputs": (sample_count, len(self.output_names)),
        }
        for label, array in (
            ("inputs", inputs),
            ("states", states),
            ("outputs", outputs),
        ):
            if array.shape != expected_shapes[label]:
                raise ValueError(
                    f"{label} has shape {array.shape}; expected {expected_shapes[label]}"
                )
        object.__setattr__(self, "time", time)
        object.__setattr__(self, "inputs", inputs)
        object.__setattr__(self, "states", states)
        object.__setattr__(self, "outputs", outputs)

    def state(self, name: str) -> FloatArray:
        return self.states[:, self.state_names.index(name)]

    def input(self, name: str) -> FloatArray:
        return self.inputs[:, self.input_names.index(name)]


def time_vector(duration: float, time_step: float) -> FloatArray:
    """Build an evenly spaced time vector that includes both endpoints."""

    if not np.isfinite(duration) or duration <= 0.0:
        raise ValueError("duration must be finite and positive")
    if not np.isfinite(time_step) or time_step <= 0.0:
        raise ValueError("time_step must be finite and positive")
    sample_count = int(np.ceil(duration / time_step)) + 1
    return np.linspace(0.0, duration, sample_count)


def state_vector(
    model: StateSpaceModel,
    perturbations: Mapping[str, float],
) -> FloatArray:
    """Convert a state-name mapping to a state vector in model order."""

    unknown = set(perturbations) - set(model.state_names)
    if unknown:
        raise ValueError(f"unknown state names: {sorted(unknown)}")
    vector = np.zeros(len(model.state_names), dtype=float)
    for name, value in perturbations.items():
        if not np.isfinite(value):
            raise ValueError(f"state perturbation {name} must be finite")
        vector[model.state_names.index(name)] = value
    return vector


def step_inputs(
    model: StateSpaceModel,
    time: ArrayLike,
    commands: Mapping[str, tuple[float, float]],
) -> FloatArray:
    """Build input histories from name -> (amplitude, start time) commands."""

    time_array = _validate_time(time)
    unknown = set(commands) - set(model.input_names)
    if unknown:
        raise ValueError(f"unknown input names: {sorted(unknown)}")
    inputs = np.zeros((time_array.size, len(model.input_names)), dtype=float)
    for name, (amplitude, start_time) in commands.items():
        if not np.isfinite(amplitude) or not np.isfinite(start_time):
            raise ValueError(f"step command for {name} must be finite")
        if start_time < time_array[0] or start_time > time_array[-1]:
            raise ValueError(f"step start time for {name} is outside the simulation")
        inputs[time_array >= start_time, model.input_names.index(name)] = amplitude
    return inputs


def _validate_time(time: ArrayLike) -> FloatArray:
    time_array = np.asarray(time, dtype=float)
    if time_array.ndim != 1 or time_array.size < 2:
        raise ValueError("time must be one-dimensional with at least two samples")
    if not np.all(np.isfinite(time_array)) or not np.all(np.diff(time_array) > 0.0):
        raise ValueError("time must be finite and strictly increasing")
    if not np.isclose(time_array[0], 0.0, atol=1e-12):
        raise ValueError("time must begin at zero")
    steps = np.diff(time_array)
    if not np.allclose(steps, steps[0], rtol=1e-8, atol=1e-12):
        raise ValueError("scipy.signal.lsim requires evenly spaced time samples")
    return time_array


def simulate_response(
    model: StateSpaceModel,
    time: ArrayLike,
    *,
    inputs: ArrayLike | None = None,
    initial_state: ArrayLike | None = None,
    name: str = "Linear response",
) -> SimulationResult:
    """Simulate a continuous-time LTI model with optional inputs and x(0)."""

    time_array = _validate_time(time)
    if inputs is None:
        input_array = np.zeros((time_array.size, len(model.input_names)))
    else:
        input_array = np.asarray(inputs, dtype=float)
    if input_array.shape != (time_array.size, len(model.input_names)):
        raise ValueError(
            f"inputs have shape {input_array.shape}; expected "
            f"{(time_array.size, len(model.input_names))}"
        )
    if not np.all(np.isfinite(input_array)):
        raise ValueError("inputs contain non-finite values")

    if initial_state is None:
        initial_array = np.zeros(len(model.state_names))
    else:
        initial_array = np.asarray(initial_state, dtype=float)
    if initial_array.shape != (len(model.state_names),):
        raise ValueError(
            f"initial_state has shape {initial_array.shape}; "
            f"expected {(len(model.state_names),)}"
        )
    if not np.all(np.isfinite(initial_array)):
        raise ValueError("initial_state contains non-finite values")

    system = StateSpace(model.A, model.B, model.C, model.D)
    returned_time, outputs, states = lsim(
        system,
        U=input_array,
        T=time_array,
        X0=initial_array,
    )
    outputs = np.asarray(outputs, dtype=float)
    states = np.asarray(states, dtype=float)
    if outputs.ndim == 1:
        outputs = outputs[:, np.newaxis]
    if states.ndim == 1:
        states = states[:, np.newaxis]
    return SimulationResult(
        name=name,
        time=returned_time,
        inputs=input_array,
        states=states,
        outputs=outputs,
        state_names=model.state_names,
        input_names=model.input_names,
        output_names=model.output_names,
    )


def simulate_initial_response(
    model: StateSpaceModel,
    time: ArrayLike,
    perturbations: Mapping[str, float],
    *,
    name: str = "Initial-condition response",
) -> SimulationResult:
    return simulate_response(
        model,
        time,
        initial_state=state_vector(model, perturbations),
        name=name,
    )


def simulate_step_response(
    model: StateSpaceModel,
    time: ArrayLike,
    commands: Mapping[str, tuple[float, float]],
    *,
    initial_state: ArrayLike | None = None,
    name: str = "Control-step response",
) -> SimulationResult:
    return simulate_response(
        model,
        time,
        inputs=step_inputs(model, time, commands),
        initial_state=initial_state,
        name=name,
    )
