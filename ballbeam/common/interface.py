"""Interface to the Open Ball & Beam."""

from __future__ import annotations

import sys
from time import time
from typing import Any

import keyboard
import pyqtgraph as pg  # type: ignore[import]
from pyqtgraph.Qt import QtCore, QtWidgets  # type: ignore[import]

from ballbeam.common.controller import CONTROLLER_CLASS_MAP, Controller
from ballbeam.common.cost import COST_CLASS_MAP, Cost
from ballbeam.common.logger import LOGGER_CLASS_MAP, LogData, Logger
from ballbeam.common.monitor import MONITOR_CLASS_MAP, Monitor
from ballbeam.common.reference import REFERENCE_CLASS_MAP, Reference
from ballbeam.common.system import SYSTEM_CLASS_MAP, System
from ballbeam.common.utils import instantiate_object_by_class_name
from ballbeam.paths import ROOT_PATH


class Interface:
    """Interface to the Open Ball and Beam system."""

    def __init__(self, config: Any) -> None:
        """Initialize the interface."""
        # Set the config
        self.config = config

        # Instantiate the system
        self.system: System = instantiate_object_by_class_name(
            self.config.system_type,
            SYSTEM_CLASS_MAP,
        )

        # Instantiate the controller
        self.controller: Controller = instantiate_object_by_class_name(
            self.config.controller_type,
            CONTROLLER_CLASS_MAP,
        )

        # Instantiate the reference
        self.reference: Reference = instantiate_object_by_class_name(
            self.config.reference_type,
            REFERENCE_CLASS_MAP,
        )

        # Instantiate the cost
        self.cost: Cost = instantiate_object_by_class_name(
            self.config.cost_type,
            COST_CLASS_MAP,
        )

        # Instantiate the data logger
        self.logger: Logger = instantiate_object_by_class_name(
            self.config.logger_type,
            LOGGER_CLASS_MAP,
        )

        # Instantiate the monitors
        self.monitors: list[Monitor] = [
            instantiate_object_by_class_name(
                monitor_type,
                MONITOR_CLASS_MAP,
            )
            for monitor_type in self.config.monitor_types
        ]

        # Initialization
        self.time_start = time()
        self.time_last = time()
        self.t = 0

        # Create the Qt application
        self.app = QtWidgets.QApplication(sys.argv)
        self.app.aboutToQuit.connect(self.stop)
        self.timer = pg.QtCore.QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(0)

        # Hook up the keyboard listener
        keyboard.on_press_key("enter", self.on_enter_press)

    def run(self) -> None:
        """Run the interface."""
        # Start the monitors
        for monitor in self.monitors:
            monitor.start()

        # Start the Qt app
        if (sys.flags.interactive != 1) or not hasattr(QtCore, "PYQT_VERSION"):
            self.app.exec_()

    def update(self) -> None:
        """Update the interface."""
        # Update timestamps
        time_now = time()
        time_since_last = time_now - self.time_last
        self.time_last = time_now

        # Get a new observation from the system
        observation = self.system.observe()

        # Get the current setpoint from the reference
        setpoint = self.reference.setpoint(self.t)

        # Update the controller with latest information
        self.controller.update_aux(saturated=self.system.saturated, ball_removed=self.system.ball_removed)
        self.controller.update(observation, setpoint, self.t)

        # Get the control action from the controller based on latest information
        action = self.controller.action

        # Get the current estimate of the state from the controller
        state_estimate = self.controller.state_estimate.tolist()

        # Get the current cost of the observation and action
        cost_dict = self.cost.take(observation, action)

        # Evolve the system forward by one step
        self.system.process(action)

        # Increment the time index
        self.t += 1

        # Get log data
        log_data = LogData(
            state_estimate=state_estimate,
            observation=observation,
            action=action,
            setpoint=setpoint,
            cost_dict=cost_dict,
            time_since_last=time_since_last,
            time_now=time_now,
            time_step=self.t,
        )

        # Update logger
        self.logger.append(log_data)

        # Update monitors
        for monitor in self.monitors:
            monitor.update(log_data)

    def stop(self) -> None:
        """Stop the interface."""
        # Stop the timer
        self.timer.stop()

        # Shut the system down
        self.system.shutdown()

        # Dump data
        file_name = str(int(time() * 1e9)) + ".json"
        log_data_file_path = ROOT_PATH / "data" / file_name
        self.logger.dump(log_data_file_path)

    def on_enter_press(self, event: Any) -> None:  # noqa: ARG002
        """Handle "enter" key press to quit the application."""
        self.app.quit()
