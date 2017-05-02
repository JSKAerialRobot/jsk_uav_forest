#!/usr/bin/env python

import roslib
import os
import rospy

from qt_gui.plugin import Plugin
import python_qt_binding as pyqt
if pyqt.QT_BINDING_VERSION[0] == '4':
    from python_qt_binding.QtGui import QWidget
elif pyqt.QT_BINDING_VERSION[0] == '5':
    from python_qt_binding.QtWidgets import QWidget
from python_qt_binding import loadUi

from python_qt_binding.QtCore import QTimer

from std_msgs.msg import Empty

class MyPlugin(Plugin):
    def __init__(self, context):
        super(MyPlugin, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('MyPlugin')
        
        self.task_start_pub_ = rospy.Publisher('task_start', Empty, queue_size = 10)

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",dest="quiet",help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns

        # Create QWidget
        self._widget = QWidget()
        # Get path to UI file which is a sibling of this file
        # in this example the .ui and .py file are in the same folder
        ui_file = os.path.join(os.path.dirname(os.path.realpath(__file__)), 'start_button.ui')

        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('MyPluginUi')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 

        self._widget.startButton.clicked.connect(self.taskStartPub)
        
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        # Add widget to the user interface
        context.add_widget(self._widget)
        
        
    def taskStartPub(self):
        self.task_start_pub_.publish()
