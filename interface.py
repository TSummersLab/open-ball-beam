import sys
from time import time, sleep

import numpy as np

import PyQt5
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import keyboard

from extramath import saturate
from controller import Controller, PIDController, LQGController
from reference import ConstantReference, PeriodicReference
from cost import Cost
from simulator import Simulator
from hardware import Hardware
from settings import DT, SENSOR_BACKSTOP, TIMEOUT_STEPS, \
    RAD2DEG, SERVO_CMD_MID, SERVO_CMD_MIN, SERVO_CMD_MAX, SERVO_CALIBRATION_COEFFICIENTS, \
    SHOW_KEYS, PLOT_SCROLL, PLOT_LENGTH_SECONDS, PLOT_ANTIALIAS, PLOT_WINDOW_SIZE
from colors import Monokai


# TODO move these two functions to hardware.py
def detect_ball_removed(observation, ball_removed, timeout):
    # Timeout logic: detect if ball has been removed and cease operation if so, resume if not
    if not ball_removed:
        if observation > SENSOR_BACKSTOP:
            timeout += 1
        else:
            timeout = 0
    else:
        if observation <= SENSOR_BACKSTOP:
            timeout -= 1
        else:
            timeout = 2*TIMEOUT_STEPS
    ball_removed = timeout > TIMEOUT_STEPS
    return ball_removed, timeout


def action2actuation(action, ball_removed=False, coefficients=None):
    # Convert standard action to a raw actuation
    # arg: action, beam_angle in radians
    # arg: ball_removed, boolean flag if ball has been removed from beam
    # arg: coefficients, polynomial coefficients in decreasing power order from d down to 0
    # return action, in pwm microseconds for servo
    if ball_removed:
        return SERVO_CMD_MID, False
    else:
        if coefficients is None:
            coefficients = SERVO_CALIBRATION_COEFFICIENTS
        beam_angle_rad = action
        beam_angle_deg = beam_angle_rad*RAD2DEG
        d = len(coefficients) - 1
        beam_angle_powers = np.array([beam_angle_deg**d for d in range(d, -1, -1)])
        actuation_deviation = int(np.sum(coefficients*beam_angle_powers))
        actuation = SERVO_CMD_MID - actuation_deviation
        return saturate(actuation, SERVO_CMD_MIN, SERVO_CMD_MAX)


def center_qt_window(win):
    frameGm = win.frameGeometry()
    screen = PyQt5.QtWidgets.QApplication.desktop().screenNumber(PyQt5.QtWidgets.QApplication.desktop().cursor().pos())
    centerPoint = PyQt5.QtWidgets.QApplication.desktop().screenGeometry(screen).center()
    frameGm.moveCenter(centerPoint)
    win.move(frameGm.topLeft())
    return


def setup_legend(plot):
    # Create legend in plot and apply style settings
    legend = plot.addLegend(pen=pg.mkPen(color='w', width=1, style=QtCore.Qt.SolidLine))
    legendLabelStyle = {'size': '10pt', 'color': 'w'}
    for item in legend.items:
        for single_item in item:
            if isinstance(single_item, pg.graphicsItems.LabelItem.LabelItem):
                single_item.setText(single_item.text, **legendLabelStyle)
    legend.setBrush((64, 64, 64, 224))  # legend background color
    return


class MyPlotData:
    def __init__(self, plot, name=None, scrolling=None):
        self.plot = plot
        if scrolling is None:
            self.scrolling = PLOT_SCROLL

        if name == 'position':
            ymin, ymax = -150, 150
            pens = [pg.mkPen(color=Monokai.b, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.g, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.y, width=1, style=QtCore.Qt.DashLine),
                    pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine)]
            names = ['Position (measured)', 'Position (estimated)', 'Position (reference)', None]
        elif name == 'state_estimate':
            ymin, ymax = -1, 1
            pens = [pg.mkPen(color=Monokai.b, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.g, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.y, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine)]
            names = ['Position error (estimated)', 'Velocity (estimated)', 'Position integral (estimated)', None]
        elif name == 'action':
            ymin, ymax = -0.40, 0.40
            pens = [pg.mkPen(color=Monokai.r, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine)]
            names = ['Action', None]
        elif name == 'cost':
            ymin, ymax = 0, 1
            pens = [pg.mkPen(color=Monokai.b, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.g, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.y, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.o, width=2, style=QtCore.Qt.SolidLine),
                    pg.mkPen(color=Monokai.wt, width=1, style=QtCore.Qt.DashLine)]
            names = ['Cost', 'Error cost', 'Action cost', 'Action diff cost', None]
        else:
            raise ValueError

        self.plot.enableAutoRange('y', False)
        self.plot.setYRange(ymin, ymax, padding=0)
        self.num_curves = len(names)
        if self.scrolling:
            self.datas = [np.zeros(plot_length) for i in range(self.num_curves)]
        else:
            self.datas = [np.zeros(1) for i in range(self.num_curves)]
        self.curves = [self.plot.plot(data, pen=pen, name=name) for data, pen, name in zip(self.datas, pens, names)]


def update_print(state_estimate, observation, action, setpoint, cost_dict, time_since_last):
    strings = []
    strings.append('%6d' % t)
    vals = [time_since_last, setpoint, observation, setpoint + state_estimate[0], state_estimate[1], state_estimate[2], action, cost_dict['c']]
    spacer = '    '
    strings.append(spacer.join(['%6.3f' % val for val in vals]))
    row_str = spacer.join(strings)
    print(row_str)
    return row_str


def update_plot_data(plot_data_dict, state_estimate, observation, action, setpoint, cost_dict, time_since_last):
    for key in plot_data_dict.keys():
        if key == 'position':
            new_vals = [1000*observation, 1000*(setpoint + state_estimate[0]), 1000*setpoint]
        elif key == 'state_estimate':
            new_vals = [5*state_estimate[0], state_estimate[1], 0.1*state_estimate[2]]
        elif key == 'action':
            new_vals = [action]
        elif key == 'cost':
            new_vals = [cost_dict[key] for key in ['c', 'c_error', 'c_action', 'c_action_diff']]
        else:
            raise ValueError

        plot_data = plot_data_dict[key]

        for i in range(plot_data.num_curves):
            if plot_data.scrolling:
                if i < plot_data.num_curves - 1:  # skip the last curve, which is assumed to be a constant zero (dashed line)
                    plot_data.datas[i][:-1] = plot_data.datas[i][1:]  # shift data in the array one sample left  # (see also: np.roll)
                    plot_data.datas[i][-1] = new_vals[i]  # add the new value to the end
                    plot_data.curves[i].setData(plot_data.datas[i])  # update the data in each curve
                plot_data.curves[i].setPos(t - plot_length, 0)  # shift the x range of each curve to achieve the scrolling effect
            else:
                if i < plot_data.num_curves - 1:
                    plot_data.datas[i] = np.append(plot_data.datas[i], new_vals[i])  # shift data in the array one sample left  # (see also: np.roll)
                    plot_data.curves[i].setData(plot_data.datas[i])  # update the data in each curve
    return


def update():
    global time_start, time_last
    global saturated, ball_removed, timeout
    global t

    time_now = time()
    time_since_start = time_now - time_start
    time_since_last = time_now - time_last
    time_last = time_now

    observation = system.observe()
    ball_removed, timeout = detect_ball_removed(observation, ball_removed, timeout)
    setpoint = reference.setpoint(t)
    control_kwargs = dict(saturated=saturated, ball_removed=ball_removed)
    controller.update(observation, setpoint, t, **control_kwargs)
    action = controller.u
    state_estimate = controller.z
    if ball_removed:
        action = 0.0
        state_estimate = np.zeros(4)
    actuation, saturated = action2actuation(action, ball_removed)
    cost_dict = cost.take(observation, action)
    system.process(action, actuation)
    t += 1

    update_print(state_estimate, observation, action, setpoint, cost_dict, time_since_last)
    if SHOW_KEYS is not None:
        global plot_data_dict
        update_plot_data(plot_data_dict, state_estimate, observation, action, setpoint, cost_dict, time_since_last)
    return


if __name__ == '__main__':
    # Fundamentals
    # system = Simulator()
    system = Hardware()

    # controller = Controller()
    # controller = PIDController()
    controller = LQGController()

    reference = ConstantReference()
    # reference = PeriodicReference(waveform='sine', frequency=0.05)
    # reference = PeriodicReference(waveform='square', frequency=0.05)

    cost = Cost()

    # Init
    time_start = time()
    time_last = time()

    t = 0
    saturated = False
    ball_removed = False
    timeout = 0

    strings = ['time', 'time_since_last', 'setpoint', 'position (measured)', 'position (estimate)', 'velocity (estimated)', 'position integral (estimated)', 'action', 'cost']
    spacer = '    '
    header_str = spacer.join(strings)
    print(header_str)

    if SHOW_KEYS is not None:
        plot_length = int(PLOT_LENGTH_SECONDS/DT)  # convert from seconds to # of steps
        pg.setConfigOptions(antialias=PLOT_ANTIALIAS)  # enable antialiasing to get rid of jaggies, turn off to save render time
        pg.setConfigOption('background', Monokai.k)
        pg.setConfigOption('foreground', Monokai.wt)
        win = pg.GraphicsLayoutWidget(show=True, title='Ball and beam control data')
        win.resize(*PLOT_WINDOW_SIZE)  # Set the window size
        center_qt_window(win)

        plot_data_dict = {}
        for key in SHOW_KEYS:
            plot = win.addPlot()
            setup_legend(plot)
            plot_data_dict[key] = MyPlotData(plot, key)
            win.nextRow()

        timer = pg.QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(0)

        if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
            QtGui.QApplication.instance().exec_()

        system.shutdown()
    else:
        while True:
            # TODO - make data history logging independent of plotting, and pass history data to realtime plotter
            # TODO - add option to save history data
            update()
            if keyboard.is_pressed("enter"):
                break
        system.shutdown()

