"""Utilities for working with QT."""

import PyQt5
import pyqtgraph as pg  # type: ignore[import]
from pyqtgraph.Qt import QtCore  # type: ignore[import]


def center_qt_window(win: pg.GraphicsLayoutWidget) -> None:
    """Center a QT window on the screen."""
    frameGm = win.frameGeometry()
    screen = PyQt5.QtWidgets.QApplication.desktop().screenNumber(PyQt5.QtWidgets.QApplication.desktop().cursor().pos())  # type: ignore[attr-defined]
    centerPoint = PyQt5.QtWidgets.QApplication.desktop().screenGeometry(screen).center()  # type: ignore[attr-defined]
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
    # Set legend background color
    legend.setBrush((64, 64, 64, 224))
