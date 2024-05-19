"""Monitor the system while in operation."""

from __future__ import annotations

import dataclasses
from abc import ABC, abstractmethod
from typing import TYPE_CHECKING, Any

import numpy as np
import pyqtgraph as pg  # type: ignore[import]
from pyqtgraph.Qt import QtCore  # type: ignore[import]

from ballbeam.common.colors import Monokai
from ballbeam.common.qt_utils import center_qt_window, setup_legend
from ballbeam.configurators.configs import load_config_as_object

if TYPE_CHECKING:
    from ballbeam.common.logger import LogData


class Monitor(ABC):
    """A monitor of log data."""

    @abstractmethod
    def start(self) -> None:
        """Start the monitor."""

    @abstractmethod
    def update(self, log_data: LogData) -> None:
        """Update the monitor."""


class PrintMonitor(Monitor):
    """A monitor using printing."""

    def __init__(self) -> None:
        """Initialize."""
        super().__init__()
        self.header_strings = ["     t", "    dt", "ps_ref", "ps_mes", "ps_est", "vl_est", "ip_est", "     u", "  cost"]
        self.spacer = "    "
        self.header_str = self.spacer.join(self.header_strings)

    def start(self) -> None:
        """Start the monitor."""
        print(self.header_str)

    def update(self, log_data: LogData) -> None:
        """Update the monitor."""
        strings = []
        strings.append("%6d" % log_data.time_step)
        vals = [
            log_data.time_since_last,
            log_data.setpoint,
            log_data.observation,
            log_data.setpoint + log_data.state_estimate[0],
            log_data.state_estimate[1],
            log_data.state_estimate[2],
            log_data.action,
            log_data.cost_dict["c"],
        ]
        strings.append(self.spacer.join(["%6.3f" % val for val in vals]))
        row_str = self.spacer.join(strings)
        print("\r" + row_str, end="")


@dataclasses.dataclass
class PlotMonitorConfig:
    """Configuration for a plot monitor."""

    antialias: bool
    window_size: tuple[int, int]
    show_keys: list[str]
    scroll: bool
    length_steps: int
    length_seconds: float


class PlotManager:
    """Plot manager."""

    def __init__(self, plot: pg.PlotItem, name: str, *, scroll: bool, length_steps: int) -> None:
        """Initialize."""
        self.plot = plot
        self.scroll = scroll
        self.length_steps = length_steps

        if name == "position":
            ymin, ymax = -120.0, 120.0
            pens = [
                pg.mkPen(color=Monokai.b, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.g, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.y, width=1, style=QtCore.Qt.DashLine),
                pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine),
            ]
            names = ["Position (measured)", "Position (estimated)", "Position (reference)", None]
        elif name == "state_estimate":
            ymin, ymax = -1.0, 1.0
            pens = [
                pg.mkPen(color=Monokai.b, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.g, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.y, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine),
            ]
            names = ["Position error (estimated)", "Velocity (estimated)", "Position integral (estimated)", None]
        elif name == "action":
            ymin, ymax = -0.40, 0.40
            pens = [
                pg.mkPen(color=Monokai.r, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine),
            ]
            names = ["Action", None]
        elif name == "cost":
            ymin, ymax = 0.0, 1.0
            pens = [
                pg.mkPen(color=Monokai.b, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.g, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.y, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.o, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine),
            ]
            names = ["Cost", "Error cost", "Action cost", "Action diff cost", None]
        else:
            msg = f'Invalid plot data name "{name}"'
            raise ValueError(msg)

        self.plot.enableAutoRange(axis="y", enable=False)
        self.plot.setYRange(min=ymin, max=ymax, padding=0)
        self.num_curves = len(names)
        if self.scroll:
            self.datas = [np.zeros(self.length_steps) for i in range(self.num_curves)]
        else:
            self.datas = [np.zeros(1) for i in range(self.num_curves)]
        self.curves = [self.plot.plot(data, pen=pen, name=name) for data, pen, name in zip(self.datas, pens, names)]


class PlotMonitor(Monitor):
    """A monitor using Qt plotting."""

    def __init__(self) -> None:
        """Initialize."""
        super().__init__()
        raw_config: Any = load_config_as_object("plot_config.yaml")
        self.config = PlotMonitorConfig(
            antialias=raw_config.ANTIALIAS,
            window_size=raw_config.WINDOW_SIZE,
            show_keys=raw_config.SHOW_KEYS,
            scroll=raw_config.SCROLL,
            length_steps=raw_config.LENGTH_STEPS,
            length_seconds=raw_config.LENGTH_SECONDS,
        )

        # Set up the QT window
        # NOTE: Enable antialiasing to get rid of jaggies, turn it off to reduce render time
        pg.setConfigOptions(antialias=self.config.antialias)
        pg.setConfigOption("background", Monokai.k)
        pg.setConfigOption("foreground", Monokai.wt)
        self.win = pg.GraphicsLayoutWidget(show=True, title="Ball & Beam control data")
        self.win.resize(*self.config.window_size)  # Set the window size
        center_qt_window(self.win)

        # Create plots and attach managers.
        self.plot_managers = {}
        for key in self.config.show_keys:
            plot = self.win.addPlot()
            setup_legend(plot)
            self.plot_managers[key] = PlotManager(
                plot,
                key,
                scroll=self.config.scroll,
                length_steps=self.config.length_steps,
            )
            self.win.nextRow()

    def start(self) -> None:
        """Start the monitor."""

    def update(self, log_data: LogData) -> None:
        """Update the monitor."""
        for key in self.config.show_keys:
            if key == "position":
                new_vals = [
                    1000 * log_data.observation,
                    1000 * (log_data.setpoint + log_data.state_estimate[0]),
                    1000 * log_data.setpoint,
                ]
            elif key == "state_estimate":
                new_vals = [
                    5 * log_data.state_estimate[0],
                    log_data.state_estimate[1],
                    10.0 * log_data.state_estimate[2],
                ]
            elif key == "action":
                new_vals = [log_data.action]
            elif key == "cost":
                new_vals = [log_data.cost_dict[key] for key in ["c", "c_error", "c_action", "c_action_diff"]]
            else:
                raise ValueError

            plot_manager = self.plot_managers[key]

            for i in range(plot_manager.num_curves):
                if plot_manager.scroll:
                    if (
                        i < plot_manager.num_curves - 1
                    ):  # skip the last curve, which is assumed to be a constant zero (dashed line)
                        plot_manager.datas[i][:-1] = plot_manager.datas[i][
                            1:
                        ]  # shift data in the array one sample left  # (see also: np.roll)
                        plot_manager.datas[i][-1] = new_vals[i]  # add the new value to the end
                        plot_manager.curves[i].setData(plot_manager.datas[i])  # update the data in each curve
                    plot_manager.curves[i].setPos(
                        log_data.time_step - self.config.length_steps,
                        0,
                    )  # shift the x range of each curve to achieve the scrolling effect
                else:  # noqa: PLR5501
                    if i < plot_manager.num_curves - 1:
                        plot_manager.datas[i] = np.append(
                            plot_manager.datas[i],
                            new_vals[i],
                        )  # shift data in the array one sample left  # (see also: np.roll)
                        plot_manager.curves[i].setData(plot_manager.datas[i])  # update the data in each curve


# TODO(bgravell): Use a decorator to register systems with this registry  # noqa: TD003, FIX002
MONITOR_CLASS_MAP = {
    "Print": PrintMonitor,
    "Plot": PlotMonitor,
}
