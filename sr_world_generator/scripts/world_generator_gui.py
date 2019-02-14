#!/usr/bin/env python

import sys
from python_qt_binding.QtGui import *
from python_qt_binding.QtCore import *
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi

try:
    from python_qt_binding.QtWidgets import *
except ImportError:
    pass

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import matplotlib.pyplot as plt
from matplotlib import __version__ as matplotlibversion

import signal
import rospy
import os
import rospkg
import sqlite3
import rviz
import subprocess


class SrWorldGeneratorGui(Plugin):
    def __init__(self, context):
        super(SrWorldGeneratorGui, self).__init__(context)
        self.setObjectName("SrWorldGeneratorGui")
        self._widget = QWidget()

        self.available_databases = []
        self.available_benchmarks = []
        self.planners = []

        ui_file = os.path.join(rospkg.RosPack().get_path(
            'sr_world_generator'), 'uis', 'moveit_planner_benchmarking.ui')

        loadUi(ui_file, self._widget)
        if __name__ != "__main__":
            context.add_widget(self._widget)

        self._widget.setWindowTitle("Gazebo World Generator")
        self.init_widget_children()
        self.example_button.clicked.connect(self.example_method)

    def init_widget_children(self):
        self.example_button = self._widget.findChild(QPushButton, "example_button")
        # self.clearance_layout = self._widget.findChild(QVBoxLayout, "clearance_layout")
        # self.initial_z_combo_box = self._widget.findChild(QComboBox, "initial_z_comboBox")

    def destruct(self):
        self._widget = None
        rospy.loginfo("Closing gazebo world generator")

    def example_method(self):
        print "Example!"

if __name__ == "__main__":
    rospy.init_node("sr_gazebo_world_generator")
    app = QApplication(sys.argv)
    planner_benchmarking_gui = SrWorldGeneratorGui(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
