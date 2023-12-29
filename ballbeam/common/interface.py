"""Interface to the Open Ball & Beam."""
from __future__ import annotations

import sys
from dataclasses import dataclass
from time import time
from typing import Any

import keyboard
import numpy as np
import PyQt5
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets

from ballbeam.common.colors import Monokai
from ballbeam.common.controller import Controller, LQGController, MPCController, PIDController, SineController
from ballbeam.common.cost import Cost
from ballbeam.common.hardware import Hardware
from ballbeam.common.reference import ConstantReference, FastSquareReference, SlowSineReference
from ballbeam.common.simulator import Simulator
from ballbeam.configurators.configs import CONFIG


def center_qt_window(win: pg.GraphicsLayoutWidget) -> None:
    """Center a QT window on the screen."""
    frameGm = win.frameGeometry()
    screen = PyQt5.QtWidgets.QApplication.desktop().screenNumber(PyQt5.QtWidgets.QApplication.desktop().cursor().pos())
    centerPoint = PyQt5.QtWidgets.QApplication.desktop().screenGeometry(screen).center()
    frameGm.moveCenter(centerPoint)
    win.move(frameGm.topLeft())


def setup_legend(plot: pg.PlotItem) -> None:
    """Create legend in plot and apply style settings."""
    legend = plot.addLegend(pen=pg.mkPen(color="w", width=1, style=QtCore.Qt.SolidLine))
    legendLabelStyle = {"size": "10pt", "color": "w"}
    for item in legend.items:
        for single_item in item:
            if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
                single_item.setText(single_item.text, **legendLabelStyle)
    legend.setBrush((64, 64, 64, 224))  # legend background color


class MyPlotData:
    """Plot data."""

    def __init__(self, plot: pg.PlotItem, name: str, *, scrolling: bool | None = None) -> None:
        """Initialize."""
        self.plot = plot
        if scrolling is None:
            self.scrolling = CONFIG.plot.SCROLL

        if name == "position":
            ymin, ymax = -120, 120
            pens = [
                pg.mkPen(color=Monokai.b, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.g, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.y, width=1, style=QtCore.Qt.DashLine),
                pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine),
            ]
            names = ["Position (measured)", "Position (estimated)", "Position (reference)", None]
        elif name == "state_estimate":
            ymin, ymax = -1, 1
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
            ymin, ymax = 0, 1
            pens = [
                pg.mkPen(color=Monokai.b, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.g, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.y, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.o, width=2, style=QtCore.Qt.SolidLine),
                pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine),
            ]
            names = ["Cost", "Error cost", "Action cost", "Action diff cost", None]
        else:
            raise ValueError

        self.plot.enableAutoRange(axis="y", enable=False)
        self.plot.setYRange(min=ymin, max=ymax, padding=0)
        self.num_curves = len(names)
        if self.scrolling:
            self.datas = [np.zeros(CONFIG.plot.LENGTH_STEPS) for i in range(self.num_curves)]
        else:
            self.datas = [np.zeros(1) for i in range(self.num_curves)]
        self.curves = [self.plot.plot(data, pen=pen, name=name) for data, pen, name in zip(self.datas, pens, names)]


@dataclass
class Data:
    """Interface data."""

    state_estimate: np.ndarray
    observation: float
    action: float
    setpoint: float
    cost_dict: dict
    time_since_last: float


def update_print(data: Data) -> str:
    """Update the printed data."""
    strings = []
    strings.append("%6d" % t)
    vals = [
        data.time_since_last,
        data.setpoint,
        data.observation,
        data.setpoint + data.state_estimate[0],
        data.state_estimate[1],
        data.state_estimate[2],
        data.action,
        data.cost_dict["c"],
    ]
    spacer = "    "
    strings.append(spacer.join(["%6.3f" % val for val in vals]))
    row_str = spacer.join(strings)
    print("\r" + row_str, end="")
    return row_str


def update_plot_data(plot_data_dict: dict[str, MyPlotData], data: Data) -> None:
    """Update the plot data."""
    for key in plot_data_dict:
        if key == "position":
            new_vals = [1000 * data.observation, 1000 * (data.setpoint + data.state_estimate[0]), 1000 * data.setpoint]
        elif key == "state_estimate":
            new_vals = [5 * data.state_estimate[0], data.state_estimate[1], 10.0 * data.state_estimate[2]]
        elif key == "action":
            new_vals = [data.action]
        elif key == "cost":
            new_vals = [data.cost_dict[key] for key in ["c", "c_error", "c_action", "c_action_diff"]]
        else:
            raise ValueError

        plot_data = plot_data_dict[key]

        for i in range(plot_data.num_curves):
            if plot_data.scrolling:
                if (
                    i < plot_data.num_curves - 1
                ):  # skip the last curve, which is assumed to be a constant zero (dashed line)
                    plot_data.datas[i][:-1] = plot_data.datas[i][
                        1:
                    ]  # shift data in the array one sample left  # (see also: np.roll)
                    plot_data.datas[i][-1] = new_vals[i]  # add the new value to the end
                    plot_data.curves[i].setData(plot_data.datas[i])  # update the data in each curve
                plot_data.curves[i].setPos(
                    t - CONFIG.plot.LENGTH_STEPS,
                    0,
                )  # shift the x range of each curve to achieve the scrolling effect
            else:  # noqa: PLR5501
                if i < plot_data.num_curves - 1:
                    plot_data.datas[i] = np.append(
                        plot_data.datas[i],
                        new_vals[i],
                    )  # shift data in the array one sample left  # (see also: np.roll)
                    plot_data.curves[i].setData(plot_data.datas[i])  # update the data in each curve


def update_displayed_data(data: Data) -> None:
    """Update the displayed data."""
    update_print(data)

    if CONFIG.plot.SHOW_KEYS is not None:
        # TODO(bgravell): get rid of globals  # noqa: TD003, FIX002
        global plot_data_dict  # noqa: PLW0602
        update_plot_data(plot_data_dict, data)


def update() -> None:
    """Update the interface."""
    # TODO(bgravell): get rid of globals  # noqa: TD003, FIX002
    global time_start, time_last  # noqa: PLW0602, PLW0603
    global t  # noqa: PLW0603

    time_now = time()
    time_now - time_start
    time_since_last = time_now - time_last
    time_last = time_now

    # Get a new observation from the system
    observation = system.observe()

    # Get the current setpoint from the reference
    setpoint = reference.setpoint(t)

    # Update the controller with latest information
    controller.update_aux(saturated=system.saturated, ball_removed=system.ball_removed)
    controller.update(observation, setpoint, t)

    # Get the control action from the controller based on latest information
    action = controller.action

    # Get the current estimate of the state from the controller
    state_estimate = controller.state_estimate

    # Get the current cost of the observation and action
    cost_dict = cost.take(observation, action)

    # Evolve the system forward by one step
    system.process(action)

    # Increment the time index
    t += 1

    # Data display
    data = Data(state_estimate, observation, action, setpoint, cost_dict, time_since_last)
    update_displayed_data(data)


SYSTEM_CLASS_MAP = {
    "Simulator": Simulator,
    "Hardware": Hardware,
}

CONTROLLER_CLASS_MAP = {
    "Null": Controller,
    "Sine": SineController,
    "PID": PIDController,
    "LQG": LQGController,
    "MPC": MPCController,
}

REFERENCE_CLASS_MAP = {
    "Constant": ConstantReference,
    "SlowSine": SlowSineReference,
    "FastSquare": FastSquareReference,
}

COST_CLASS_MAP = {"Default": Cost}


def instantiate_object_by_class_name(class_name: str, class_map: dict[str, Any]) -> Any:
    """Instantiate an object by class name using a class map."""
    selected_class = class_map.get(class_name)
    if selected_class is None:
        msg = f'Failed to find class for class name "{class_name}", must be one of {sorted(class_map)}.'
        raise ValueError(msg)
    return selected_class()


if __name__ == "__main__":
    # Instantiate the system
    system = instantiate_object_by_class_name(CONFIG.interface.system_type, SYSTEM_CLASS_MAP)

    # Instantiate the controller
    controller = instantiate_object_by_class_name(CONFIG.interface.controller_type, CONTROLLER_CLASS_MAP)

    # Instantiate the reference
    reference = instantiate_object_by_class_name(CONFIG.interface.reference_type, REFERENCE_CLASS_MAP)

    # Instantiate the cost
    cost = instantiate_object_by_class_name(CONFIG.interface.cost_type, COST_CLASS_MAP)

    # Initialization
    time_start = time()
    time_last = time()

    t = 0

    strings = ["     t", "    dt", "ps_ref", "ps_mes", "ps_est", "vl_est", "ip_est", "     u", "  cost"]
    spacer = "    "
    header_str = spacer.join(strings)
    print(header_str)

    if CONFIG.plot.SHOW_KEYS is None:
        while True:
            # TODO(bgravell): make data history logging independent of plotting, and pass history data to realtime plotter  # noqa: FIX002, TD003, E501
            # TODO(bgravell): add option to save history data  # noqa: FIX002, TD003
            update()
            if keyboard.is_pressed("enter"):
                break
        system.shutdown()
    else:
        # enable antialiasing to get rid of jaggies, turn off to save render time
        pg.setConfigOptions(antialias=CONFIG.plot.ANTIALIAS)
        pg.setConfigOption("background", Monokai.k)
        pg.setConfigOption("foreground", Monokai.wt)
        win = pg.GraphicsLayoutWidget(show=True, title="Ball & Beam control data")
        win.resize(*CONFIG.plot.WINDOW_SIZE)  # Set the window size
        center_qt_window(win)

        plot_data_dict = {}
        for key in CONFIG.plot.SHOW_KEYS:
            plot = win.addPlot()
            setup_legend(plot)
            plot_data_dict[key] = MyPlotData(plot, key)
            win.nextRow()

        timer = pg.QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(0)

        if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
            app = QtWidgets.QApplication(sys.argv)
            app.instance().exec_()

        system.shutdown()
