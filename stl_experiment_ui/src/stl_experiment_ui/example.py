#!/usr/bin/env python

import sys, os
from PyQt5 import QtCore
from PyQt5 import QtGui
from PyQt5 import QtWidgets

from qrangeslider import QRangeSlider

app = QtWidgets.QApplication(sys.argv)

# Example 1
rs3 = QRangeSlider()
rs3.show()
rs3.setWindowTitle('example 3')
rs3.setFixedHeight(50)
rs3.setMin(0)
rs3.setMax(2000)
rs3.setRange(500, 1253)
rs3.setBackgroundStyle('background: qlineargradient(x1:0, y1:0, x2:0, y2:1, stop:0 #aaa, stop:1 #777);')
rs3.handle.setStyleSheet('background: url(data/sin.png) repeat-x; border: 0px;')
rs3.setStyleSheet("""
QRangeSlider > QSplitter::handle {
    background: #777;
    border: 1px solid #555;
}
QRangeSlider > QSplitter::handle:vertical {
    height: 2px;
}
QRangeSlider > QSplitter::handle:pressed {
    background: #ca5;
}
""")
rs3.handle.setTextColor(150)

app.exec_()