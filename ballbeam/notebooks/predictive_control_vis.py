from abc import ABC, abstractmethod

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import plotly.graph_objs as go
from plotly.subplots import make_subplots
import rerun as rr
import rerun.blueprint as rrb

from ballbeam.common.colors import Monokai


def color255to1(c):
    return np.array(c) / 255


class PredictiveControlVisualizer(ABC):

    obs_display_lim = (-0.11, 0.11)
    act_display_lim = (-0.45, 0.45)

    # obs_display_lim = (-0.06, 0.06)
    # act_display_lim = (-0.45, 0.45)

    def __init__(self, controller, dt):
        self.controller = controller
        self.predictor = self.controller.predictor

        past = self.predictor.horizons.past
        futr = self.predictor.horizons.futr

        self.past_range = np.arange(-past, 0)
        self.futr_range = np.arange(0, futr)

        self.past_range_sec = dt * self.past_range
        self.futr_range_sec = dt * self.futr_range

        self.dt = dt

    def prep_data_for_controller(self, offset):
        past = self.predictor.horizons.past
        futr = self.predictor.horizons.futr
        y_past = self.example.y[offset : offset + past]
        u_past = self.example.u[offset : offset + past]

        y_futr = self.example.y[offset + past : offset + past + futr]
        u_futr = self.example.u[offset + past : offset + past + futr]
        y_futr_ref = self.example.s[offset + past : offset + past + futr]
        return y_past, u_past, y_futr, u_futr, y_futr_ref

    def apply_controller(self, offset):
        y_past, u_past, y_futr, u_futr, y_futr_ref = self.prep_data_for_controller(offset)

        # Plan actions
        u_futr_new = self.controller.act(u_past, y_past, y_futr_ref)

        # Predict the future
        y_futr_pred = self.predictor.predict(u_past, y_past, u_futr)
        y_futr_new_pred = self.predictor.predict(u_past, y_past, u_futr_new)

        return {
            "y_past": y_past,
            "u_past": u_past,
            "y_futr": y_futr,
            "y_futr_pred": y_futr_pred,
            "y_futr_new_pred": y_futr_new_pred,
            "y_futr_ref": y_futr_ref,
            "u_futr": u_futr,
            "u_futr_new": u_futr_new,
        }

    @abstractmethod
    def visualize(self, example, frame_range):
        pass


class MplPredictiveControlVisualizer(PredictiveControlVisualizer):

    def init_plot(self):
        """Initialize plot."""
        past = self.predictor.horizons.past
        futr = self.predictor.horizons.futr
        y_past = np.zeros(past)
        y_futr_ref = np.zeros(futr)
        y_futr = np.zeros(futr)
        y_futr_pred = np.zeros(futr)
        y_futr_new_pred = np.zeros(futr)
        u_past = np.zeros(past)
        u_futr = np.zeros(futr)
        u_futr_new = np.zeros(futr)

        fig, axs = plt.subplots(nrows=2)
        axs[0].set_ylim(self.obs_display_lim)
        axs[1].set_ylim(self.act_display_lim)

        self.line_dict = {}

        ax = axs[0]
        (self.line_dict["y_past"],) = ax.plot(
            self.past_range_sec,
            y_past,
            c=color255to1(Monokai.s),
            label="y_past",
        )

        (self.line_dict["y_futr"],) = ax.plot(
            self.futr_range_sec,
            y_futr,
            c=color255to1(Monokai.s),
            label="y_futr",
        )
        (self.line_dict["y_futr_pred"],) = ax.plot(
            self.futr_range_sec,
            y_futr_pred,
            c=color255to1(Monokai.o),
            label="y_futr_pred",
        )
        (self.line_dict["y_futr_new_pred"],) = ax.plot(
            self.futr_range_sec,
            y_futr_new_pred,
            c=color255to1(Monokai.r),
            label="y_futr_new_pred",
        )
        (self.line_dict["y_futr_ref"],) = ax.plot(
            self.futr_range_sec,
            y_futr_ref,
            c=color255to1(Monokai.b),
            label="y_futr_ref",
        )

        ax.grid()
        ax.legend(loc="upper left")

        ax = axs[1]
        (self.line_dict["u_past"],) = ax.plot(
            self.past_range_sec,
            u_past,
            c=color255to1(Monokai.s),
            label="u_past",
        )
        (self.line_dict["u_futr"],) = ax.plot(
            self.futr_range_sec,
            u_futr,
            c=color255to1(Monokai.s),
            label="u_futr",
        )
        (self.line_dict["u_futr_new"],) = ax.plot(
            self.futr_range_sec,
            u_futr_new,
            c=color255to1(Monokai.r),
            label="u_futr_new",
        )
        ax.grid()
        ax.legend(loc="upper left")

        self.fig = fig
        self.axs = axs

        plt.close(fig)

    def update(self, offset):
        ydata_dict = self.apply_controller(offset)
        for key in self.line_dict:
            self.line_dict[key].set_ydata(ydata_dict[key])
        return tuple(self.line_dict.values())

    def visualize(self, example, frame_range):
        self.example = example
        self.init_plot()
        ani = FuncAnimation(fig=self.fig, func=self.update, frames=frame_range, interval=20, blit=True)
        return ani.to_jshtml()


class RerunPredictiveControlVisualizer(PredictiveControlVisualizer):

    def init_plot(self):
        # Initialize rerun
        rr.init("timeseries_example")
        blueprint = rrb.Vertical(
            rrb.TimeSeriesView(
                name="Outputs",
                origin="/output",
                time_ranges=rrb.VisibleTimeRange(
                    "time",
                    start=rrb.TimeRangeBoundary.cursor_relative(seconds=-5.0),
                    end=rrb.TimeRangeBoundary.cursor_relative(),
                ),
                axis_y=rrb.ScalarAxis(range=self.obs_display_lim, lock_range_during_zoom=True),
            ),
            rrb.TimeSeriesView(
                name="Actions",
                origin="/action",
                time_ranges=rrb.VisibleTimeRange(
                    "time",
                    start=rrb.TimeRangeBoundary.cursor_relative(seconds=-5.0),
                    end=rrb.TimeRangeBoundary.cursor_relative(),
                ),
                axis_y=rrb.ScalarAxis(range=self.act_display_lim, lock_range_during_zoom=True),
            ),
        )

        # Use this to pop open separate windows instead of using in-notebook display.
        rr.init("timeseries_example", spawn=True)
        rr.send_blueprint(blueprint)

    def update(self, offset):
        ydata_dict = self.apply_controller(offset)

        # Log statics
        futr_idx = self.predictor.horizons.futr - 1
        rr.set_time_seconds("time", self.dt * (offset + futr_idx))
        rr.log(f"output/futr", rr.Scalar(ydata_dict["y_futr"][futr_idx]))
        rr.log(f"output/futr_ref", rr.Scalar(ydata_dict["y_futr_ref"][futr_idx]))

        # Log dynamics
        offset_str = f"offset_{offset:04d}"
        for futr_idx in range(self.predictor.horizons.futr):
            rr.set_time_seconds("time", self.dt * (offset + futr_idx))
            rr.log(f"output/futr_pred/{offset_str}", rr.Scalar(ydata_dict["y_futr_new_pred"][futr_idx]))
            rr.log(f"action/futr/{offset_str}", rr.Scalar(ydata_dict["u_futr_new"][futr_idx]))

    def visualize(self, example, frame_range):
        self.example = example
        self.init_plot()

        # Set static plot styles
        rr.log(
            f"output/futr_ref",
            rr.SeriesLine(color=[*Monokai.g, 255], width=2, name="Reference Output"),
            static=True,
        )
        rr.log(
            f"output/futr",
            rr.SeriesLine(color=[*Monokai.b, 255], width=2, name="Actual Output"),
            static=True,
        )

        for offset in frame_range:
            offset_str = f"offset_{offset:04d}"
            rr.log(
                f"output/futr_pred/{offset_str}",
                rr.SeriesLine(color=[*Monokai.r, 64], width=0.5, name="Predicted Output"),
                static=True,
            )
            rr.log(
                f"action/futr/{offset_str}",
                rr.SeriesLine(color=[*Monokai.v, 64], width=0.5, name="Planned Action"),
                static=True,
            )

        # Log static data
        offset = frame_range[0]
        past = self.predictor.horizons.past
        futr = self.predictor.horizons.futr
        y_futr_ref = example.s[offset + past : offset + past + futr]
        y_futr = example.y[offset + past : offset + past + futr]
        for futr_idx in range(futr - 1):
            rr.set_time_seconds("time", self.dt * (offset + futr_idx))
            rr.log(f"output/futr", rr.Scalar(y_futr[futr_idx]))
            rr.log(f"output/futr_ref", rr.Scalar(y_futr_ref[futr_idx]))

        # Compute and log dynamic data
        for offset in frame_range:
            self.update(offset)


def visualize(controller, dt, example, frame_range, engine):
    if engine == "matplotlib":
        return MplPredictiveControlVisualizer(controller, dt).visualize(example, frame_range)
    if engine == "rerun":
        return RerunPredictiveControlVisualizer(controller, dt).visualize(example, frame_range)
