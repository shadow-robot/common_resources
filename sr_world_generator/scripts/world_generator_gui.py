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

        self.open_gazebo_button.clicked.connect(self.start_gazebo_process)
        self.close_gazebo_button.clicked.connect(self.stop_gazebo_process)
        self.transform_world_file_button.clicked.connect(self.transform_world_file)

        self.empty_world_yes_radio.clicked.connect(self.disable_world_path)
        self.empty_world_no_radio.clicked.connect(self.enable_world_path)

    def init_widget_children(self):
        self.open_gazebo_button = self._widget.findChild(QPushButton, "open_gazebo_button")
        self.close_gazebo_button = self._widget.findChild(QPushButton, "close_gazebo_button")
        self.world_browse_button = self._widget.findChild(QPushButton, "world_browse_button")
        self.gazebo_generated_file_browse = self._widget.findChild(QPushButton, "gazebo_generated_file_browse")
        self.transform_world_file_button = self._widget.findChild(QPushButton, "transform_world_file_button")

        self.start_home_yes_radio = self._widget.findChild(QRadioButton, "start_home_yes_radio")
        self.start_home_no_radio = self._widget.findChild(QRadioButton, "start_home_no_radio")
        self.empty_world_yes_radio = self._widget.findChild(QRadioButton, "empty_world_yes_radio")
        self.empty_world_no_radio = self._widget.findChild(QRadioButton, "empty_world_no_radio")

        self.world_line_edit = self._widget.findChild(QLineEdit, "world_line_edit")
        self.initial_z_line_edit = self._widget.findChild(QLineEdit, "initial_z_line_edit")
        self.gazebo_generated_world_path_line_edit = self._widget.findChild(QLineEdit, "gazebo_generated_world_path_line_edit")

        self.transform_file_group_box = self._widget.findChild(QGroupBox, "transform_file_group_box")

    def destruct(self):
        self._widget = None
        rospy.loginfo("Closing gazebo world generator")

    def start_gazebo_process(self):
        print "Starting gazebo!"
        self.open_gazebo_button.setEnabled(False)
        self.close_gazebo_button.setEnabled(True)
        self.transform_file_group_box.setEnabled(False)

    def stop_gazebo_process(self):
        print "Stopping gazebo!"
        self.open_gazebo_button.setEnabled(True)
        self.close_gazebo_button.setEnabled(False)
        self.transform_file_group_box.setEnabled(True)

    def transform_world_file(self):
        print "Transforming world!"

    def enable_world_path(self):
        self.world_line_edit.setEnabled(True)
        self.world_browse_button.setEnabled(True)

    def disable_world_path(self):
        self.world_line_edit.setEnabled(False)
        self.world_browse_button.setEnabled(False)

if __name__ == "__main__":
    rospy.init_node("sr_gazebo_world_generator")
    app = QApplication(sys.argv)
    planner_benchmarking_gui = SrWorldGeneratorGui(None)
    planner_benchmarking_gui._widget.show()
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    sys.exit(app.exec_())
