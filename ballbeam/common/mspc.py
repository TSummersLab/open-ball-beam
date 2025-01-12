from dataclasses import dataclass
from typing import TypeAlias, Self
from itertools import pairwise
from pathlib import Path

import numpy as np
import numpy.typing as npt
from scipy.optimize import lsq_linear


# TODO move to type_defs
ArrF64: TypeAlias = npt.NDArray[np.float64]


# TODO move to window_utils
def create_rolling_window(arr, k):
    T = len(arr)
    if not k < T:
        raise ValueError("k must be less than the length of the input array")

    result = np.lib.stride_tricks.sliding_window_view(arr, window_shape=k)
    return result[:-1]


import pandas as pd


def bi_ewm(x, alpha=0.5):
    """Bidirectional exponential weighted moving average."""
    return pd.Series(x).ewm(alpha=alpha).mean().iloc[::-1].ewm(alpha=alpha).mean().iloc[::-1].to_numpy()


@dataclass
class Horizons:
    past: int
    futr: int


@dataclass
class MultiStepPredictorParams:
    theta_u_past: ArrF64
    theta_y_past: ArrF64
    theta_u_futr: ArrF64


@dataclass
class RegularizationParameters:
    u_past: float = 0.0
    y_past: float = 0.0
    u_futr: float = 0.0


def roll(U: ArrF64, Y: ArrF64, horizons: Horizons):
    # Dimensions
    N, T = Y.shape
    PF = horizons.past + horizons.futr
    Tm = T - PF

    # Pre-allocate
    U_past_r = np.zeros((N * Tm, horizons.past))
    Y_past_r = np.zeros((N * Tm, horizons.past))
    U_futr_r = np.zeros((N * Tm, horizons.futr))
    Y_futr_r = np.zeros((N * Tm, horizons.futr))

    # Fill
    for i in range(N):
        u = U[i]
        y = Y[i]

        u_past_futr_r = create_rolling_window(u, PF)
        y_past_futr_r = create_rolling_window(y, PF)

        u_past_r = u_past_futr_r[:, : horizons.past]
        y_past_r = y_past_futr_r[:, : horizons.past]
        u_futr_r = u_past_futr_r[:, horizons.past :]
        y_futr_r = y_past_futr_r[:, horizons.past :]

        U_past_r[(i) * Tm : (i + 1) * Tm] = u_past_r
        Y_past_r[(i) * Tm : (i + 1) * Tm] = y_past_r
        U_futr_r[(i) * Tm : (i + 1) * Tm] = u_futr_r
        Y_futr_r[(i) * Tm : (i + 1) * Tm] = y_futr_r

    # TODO should be a dataclass
    return U_past_r, Y_past_r, U_futr_r, Y_futr_r


class MultiStepPredictor:

    def __init__(self, horizons: Horizons, params: MultiStepPredictorParams | None = None) -> None:
        self.horizons = horizons
        self.params = params

    def fit(
        self,
        U: ArrF64,
        Y: ArrF64,
        regularization_parameters: RegularizationParameters | None = None,
    ) -> None:

        U_past_r, Y_past_r, U_futr_r, Y_futr_r = roll(U, Y, self.horizons)

        # Train linear prediction model
        # Train one model per future timestep, enforcing a causal structure.
        theta_map = {}
        for k in range(self.horizons.futr):
            # Regressors
            A = np.hstack([U_past_r, Y_past_r, U_futr_r[:, 0 : k + 1]])
            # Targets
            B = Y_futr_r[:, k]

            # Solve (regularized) least squares problem
            # TODO use a more efficient routine for this solve
            if regularization_parameters is None:
                sol = np.linalg.solve(A.T @ A, A.T @ B)
            else:
                # Regularizer
                R = np.diag(
                    np.hstack(
                        [
                            regularization_parameters.u_past * np.ones(self.horizons.past),
                            regularization_parameters.y_past * np.ones(self.horizons.past),
                            regularization_parameters.u_futr * np.ones(k),
                        ]
                    )
                )
                sol = np.linalg.solve(A.T @ A + R, A.T @ B)

            theta_map[k] = sol

        # Fill array from dict
        theta = np.zeros((self.horizons.past + self.horizons.past + self.horizons.futr, self.horizons.futr))
        for k in range(self.horizons.futr):
            theta[0 : len(theta_map[k]), k] = theta_map[k]

        # Split the predictor parameters
        horizons = [self.horizons.past, self.horizons.past, self.horizons.futr]
        self.params = MultiStepPredictorParams(
            *[theta[start:stop] for start, stop in pairwise(np.concatenate([[0], np.cumsum(horizons)]))]
        )

    def predict_from_u_past(self, u_past: ArrF64) -> ArrF64:
        return u_past @ self.params.theta_u_past

    def predict_from_y_past(self, y_past: ArrF64) -> ArrF64:
        return y_past @ self.params.theta_y_past

    def predict_from_past(self, u_past: ArrF64, y_past: ArrF64) -> ArrF64:
        return self.predict_from_u_past(u_past) + self.predict_from_y_past(y_past)

    def predict_from_u_futr(self, u_futr: ArrF64) -> ArrF64:
        return u_futr @ self.params.theta_u_futr

    def predict(self, u_past: ArrF64, y_past: ArrF64, u_futr: ArrF64) -> ArrF64:
        return self.predict_from_past(u_past, y_past) + self.predict_from_u_futr(u_futr)


@dataclass
class ActionBounds:
    lower: ArrF64 | float
    upper: ArrF64 | float


class MultiStepPredictiveController:

    def __init__(self, predictor: MultiStepPredictor, action_bounds: ActionBounds, action_reg: float, action_diff_reg: float) -> None:
        self.predictor = predictor
        self.action_bounds = action_bounds
        self.action_reg = action_reg
        self.action_diff_reg = action_diff_reg

        # Pre-compute the A matrix once on initialization
        I = np.eye(self.predictor.horizons.futr)
        D = np.diff(I)
        DD = D @ D.T
        self.A = np.vstack([
            self.predictor.params.theta_u_futr.T, 
            self.action_reg * I,
            self.action_diff_reg * DD
                            ])

    @classmethod
    def from_file_path(cls, path: Path) -> Self:
        params = np.load(path)
        predictor = MultiStepPredictor(
            horizons=Horizons(params["past"], params["futr"]),
            params=MultiStepPredictorParams(
                params["theta_u_past"],
                params["theta_y_past"],
                params["theta_u_futr"],
            ),
        )
        action_bounds = ActionBounds(params["action_bound_lower"], params["action_bound_upper"])
        action_reg = params["action_reg"]
        action_diff_reg = params["action_diff_reg"]
        return cls(predictor=predictor, action_bounds=action_bounds, action_reg=action_reg, action_diff_reg=action_diff_reg)

    def to_file_path(self, path: Path) -> None:
        np.savez(
            path,
            past=self.predictor.horizons.past,
            futr=self.predictor.horizons.futr,
            theta_u_past=self.predictor.params.theta_u_past,
            theta_y_past=self.predictor.params.theta_y_past,
            theta_u_futr=self.predictor.params.theta_u_futr,
            action_bound_lower=self.action_bounds.lower,
            action_bound_upper=self.action_bounds.upper,
            action_reg=self.action_reg,
            action_diff_reg=self.action_diff_reg,
        )

    def act(self, u_past: ArrF64, y_past: ArrF64, y_ref_futr: ArrF64) -> ArrF64:
        # Prediction contribution from u_past and y_past, which are not subject to decision-making.
        y_futr_pred_from_past = self.predictor.predict_from_past(u_past, y_past)
        # Least-squares target
        Z = np.zeros(self.predictor.horizons.futr)
        b = np.concatenate([y_ref_futr - y_futr_pred_from_past, Z, Z])
        # Solve constrained least-squares problem
        return lsq_linear(self.A, b, bounds=(self.action_bounds.lower, self.action_bounds.upper)).x
