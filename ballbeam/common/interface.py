import sys
from time import time, sleep
from dataclasses import dataclass

import numpy as np

import PyQt5
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtGui
import keyboard

from ballbeam.common.controller import Controller, SineController, PIDController, LQGController
from ballbeam.common.reference import ConstantReference, PeriodicReference
from ballbeam.common.cost import Cost
from ballbeam.common.simulator import Simulator
from ballbeam.common.hardware import Hardware
from ballbeam.common.settings import DT, SHOW_KEYS, PLOT_SCROLL, PLOT_LENGTH, PLOT_ANTIALIAS, PLOT_WINDOW_SIZE
from ballbeam.common.colors import Monokai


## PLOTTING

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
            ymin, ymax = -120, 120
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
            self.datas = [np.zeros(PLOT_LENGTH) for i in range(self.num_curves)]
        else:
            self.datas = [np.zeros(1) for i in range(self.num_curves)]
        self.curves = [self.plot.plot(data, pen=pen, name=name) for data, pen, name in zip(self.datas, pens, names)]


@dataclass
class Data:
    state_estimate: np.ndarray
    observation: float
    action: float
    setpoint: float
    cost_dict: dict
    time_since_last: float


def update_print(data):
    strings = []
    strings.append('%6d' % t)
    vals = [data.time_since_last,
            data.setpoint,
            data.observation,
            data.setpoint + data.state_estimate[0],
            data.state_estimate[1],
            data.state_estimate[2],
            data.action,
            data.cost_dict['c']]
    spacer = '    '
    strings.append(spacer.join(['%6.3f' % val for val in vals]))
    row_str = spacer.join(strings)
    print('\r' + row_str, end='')
    return row_str


def update_plot_data(plot_data_dict, data):
    for key in plot_data_dict.keys():
        if key == 'position':
            new_vals = [1000*data.observation, 1000*(data.setpoint + data.state_estimate[0]), 1000*data.setpoint]
        elif key == 'state_estimate':
            new_vals = [5*data.state_estimate[0], data.state_estimate[1], 0.1*data.state_estimate[2]]
        elif key == 'action':
            new_vals = [data.action]
        elif key == 'cost':
            new_vals = [data.cost_dict[key] for key in ['c', 'c_error', 'c_action', 'c_action_diff']]
        else:
            raise ValueError

        plot_data = plot_data_dict[key]

        for i in range(plot_data.num_curves):
            if plot_data.scrolling:
                if i < plot_data.num_curves - 1:  # skip the last curve, which is assumed to be a constant zero (dashed line)
                    plot_data.datas[i][:-1] = plot_data.datas[i][1:]  # shift data in the array one sample left  # (see also: np.roll)
                    plot_data.datas[i][-1] = new_vals[i]  # add the new value to the end
                    plot_data.curves[i].setData(plot_data.datas[i])  # update the data in each curve
                plot_data.curves[i].setPos(t - PLOT_LENGTH, 0)  # shift the x range of each curve to achieve the scrolling effect
            else:
                if i < plot_data.num_curves - 1:
                    plot_data.datas[i] = np.append(plot_data.datas[i], new_vals[i])  # shift data in the array one sample left  # (see also: np.roll)
                    plot_data.curves[i].setData(plot_data.datas[i])  # update the data in each curve
    return


def update_displayed_data(data):
    update_print(data)

    # # DEBUG
    # print("%.6f    %.6f" % (system.x[2], controller.u))
    # print(actuation)

    if SHOW_KEYS is not None:
        global plot_data_dict
        update_plot_data(plot_data_dict, data)
    return


def update():
    # TODO get rid of globals
    global time_start, time_last
    global t

    time_now = time()
    time_since_start = time_now - time_start
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
    return


if __name__ == '__main__':
    # Choose the system type
    # system = Simulator()
    system = Hardware()

    # Choose the controller
    # controller = Controller()
    # controller = SineController()
    # controller = PIDController()
    controller = LQGController()

    # Choose the reference
    reference = ConstantReference()
    # reference = PeriodicReference(waveform='sine', frequency=0.10)
    # reference = PeriodicReference(waveform='square', frequency=0.10)

    # Choose the cost
    cost = Cost()

    # Initialization
    time_start = time()
    time_last = time()

    t = 0

    strings = ['     t', '    dt', 'ps_ref', 'ps_mes', 'ps_est', 'vl_est', 'ip_est', '     u', '  cost']
    spacer = '    '
    header_str = spacer.join(strings)
    print(header_str)

    if SHOW_KEYS is None:
        while True:
            # TODO - make data history logging independent of plotting, and pass history data to realtime plotter
            # TODO - add option to save history data
            update()
            if keyboard.is_pressed("enter"):
                break
        system.shutdown()
    else:
        pg.setConfigOptions(
            antialias=PLOT_ANTIALIAS)  # enable antialiasing to get rid of jaggies, turn off to save render time
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

