import os
import rospy
import rospkg

import functools
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QDialog, QWidget
from std_msgs.msg import String
from PyQt5.QtCore import QTimer, Signal, Qt
from PyQt5.QtWidgets import QTableWidgetItem
import numpy as np

ss = 0
ms = 0
handover_status = 0 #0 - Fail, 1- Success, 2-Abort
class ExperimentUI(Plugin):
    start_test_signal = Signal(bool)
    def __init__(self, context):
        super(ExperimentUI, self).__init__(context)
        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print 'arguments: ', args
            print 'unknowns: ', unknowns
        # Create QDialog
        self._widget = QDialog()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('stl_experiment_ui'), 'user_interface', 'common_dashboard.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('STLExperiment')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle())
        # context.add_widget(self._widget)
        # self._widget.hide()

        self.vaccine_stack_status_subscriber = rospy.Subscriber("/vaccine_stack_status", String, self.vaccine_stack_status_subscriber_callback)
        self.vaccine_stack_status = [6,6,6]
        self.start_test_signal.connect(self.update_vaccine_stack)
        ## Controller Design Windows
        self.stl_design_ui = STLdesignUI()
        self.pv_design_ui = PVdesignUI()
        self.dummy_design_ui = DummyedesignUI()
        self.timer_ui = TimerUI()

        self.timer_ui._widget.show()
        if unknowns[0] not in ["design", "test"]:
            print("Incorrect mode type. Change the mode to either design or test")
            raise rospy.ROSInterruptException("Incorrect task mode")
        if unknowns[1] not in ["dummy","mpc", "pid"]:
            print("Incorrect controller type. Change the controller type to either pid or mpc")
            raise rospy.ROSInterruptException("Incorrect controller type")
        if unknowns[0] == 'design':
            if unknowns[1] == 'mpc':
                self.stl_design_ui._widget.show()
            elif unknowns[1] == 'pid':
                self.pv_design_ui._widget.show()
            elif unknowns[1] == 'dummy':
                self.dummy_design_ui._widget.show()
        else:
            print("Here")
            # self._widget.show()
            self.start_test_signal.emit(True)


    def vaccine_stack_status_subscriber_callback(self, message):
        vaccine_stack_status = message.data.split(',')
        for i in range(0,3):
            self.vaccine_stack_status[i] = int(float(vaccine_stack_status[i]))
        self.start_test_signal.emit(True)

    def update_vaccine_stack(self):
        # self._widget.show()
        print(self.vaccine_stack_status)
        for i in range(0,3):
            item = QTableWidgetItem(str(self.vaccine_stack_status[i]))
            item.setTextAlignment(Qt.AlignCenter)
            self._widget.RemainingVaccineSupply.setItem(i-1, 3, item)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog


class TimerUI(Plugin):
    timer_start_signal = Signal(bool)
    stl_abort_signal = Signal(bool)
    duration_display_signal = Signal(str)

    def __init__(self):
        QDialog.__init__(self)
        # Create QDialog
        self._widget = QDialog()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('stl_experiment_ui'), 'user_interface', 'timer.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Timer UI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        self._widget.setWindowTitle(self._widget.windowTitle())

        self.task_controller = 'pid'
        #Timer
        self.timing_constraints = [2, 4]
        self.timer_start_signal.connect(self.start_timer)
        self.timer = QTimer()
        self.timer.timeout.connect(self.Time)
        time = "{0:02d}:{1:01d}".format(0, 0)
        self._widget.lcdNumber.setDigitCount(len(time))
        self._widget.lcdNumber.display(time)

        #STL failure feedback
        self.stl_abort_signal.connect(self.stl_feedback)

        #Duration display
        self.duration_display_signal.connect(self.duration_table_update)

        #Data logger publisher
        self.data_logger_publisher = rospy.Publisher('data_logger_signal', String, queue_size = 10)

        # Subscribers
        self.timer_start_request = rospy.Subscriber("/timer_start_request", String, self.start_timer_subscriber_callback)
        self.duration_display_subscriber = rospy.Subscriber("/duration_display", String, self.duration_display_callback)
        

        self.red_stylesheet = \
            "QWidget {\n" \
            + "color: rgb(255,255,255);\n" \
            + "background-color: rgb(255, 0, 0);\n" \
            + "}"
        self.green_stylesheet = \
            "QWidget {\n" \
            + "color: rgb(0,0,0);\n" \
            + "background-color: rgb(0, 255, 0);\n" \
            + "}"
        self._widget.lcdNumber.setStyleSheet(self.red_stylesheet)

    def start_timer_subscriber_callback(self, message):
        message_list = message.data.split(',')
        self.task_controller = message_list[-1]
        if message_list[0] == 'stop':
            self.timer_start_signal.emit(False)
        elif message_list[0] == 'abort':
            self.stl_abort_signal.emit(True)
        else:
            self.timing_constraints = message_list[0:-1]
            self.stl_abort_signal.emit(False)
            self.timer_start_signal.emit(True)

    def duration_display_callback(self, message):
        print(message.data)
        self.duration_display_signal.emit(message.data)

    def duration_table_update(self, event):
        duration_table = rospy.get_param('duration_table')
        for i in range(0,4):
            for j in range(0,2):
                item = QTableWidgetItem(str(duration_table[i+4*j]))
                item.setTextAlignment(Qt.AlignCenter)
                self._widget.durationTable.setItem(i, j, item)

    def start_timer(self, event):
        global ss,ms, handover_status
        if event:
            self.Time()
            self.timer.start(100)
            self.data_logger_publisher.publish('handover_start'+','+self.task_controller)
        else:
            self.timer.stop()
            if handover_status == 1:
                self.data_logger_publisher.publish('handover_success'+','+self.task_controller)
            elif handover_status == 0:
                self.data_logger_publisher.publish('handover_fail'+','+self.task_controller)
            elif handover_status == 2:
                self.data_logger_publisher.publish('handover_abort'+','+self.task_controller)

            ss = 0
            ms = 0

        
    def Time(self):
        global ss, ms, handover_status
        if ms < 9:
            ms += 1
        else:
            ms =  0
            ss += 1
        time = "{0:02d}:{1:01d}".format(ss, ms)
        print(self.timing_constraints)
        if ss*10+ms > float(self.timing_constraints[1]):
            self._widget.lcdNumber.setStyleSheet(self.red_stylesheet)
            handover_status = 0
        elif ss*10+ms >= float(self.timing_constraints[0]):
            self._widget.lcdNumber.setStyleSheet(self.green_stylesheet)
            handover_status = 1
        elif ss*10+ms < float(self.timing_constraints[0]):
            self._widget.lcdNumber.setStyleSheet(self.red_stylesheet)
            handover_status = 0

    
        self._widget.lcdNumber.setDigitCount(len(time))
        self._widget.lcdNumber.display(time)


    def stl_feedback(self, event):
        global handover_status
        if event:
            stylesheet = \
                "QWidget {\n" \
                + "color: rgb(255,0,0);\n" \
                + "}"
            self._widget.label_2.setStyleSheet(stylesheet)
            handover_status = 2
            self.timer_start_signal.emit(False)
        else:
            stylesheet = \
                "QWidget {\n" \
                + "color: rgb(255,255,255);\n" \
                + "}"
            self._widget.label_2.setStyleSheet(stylesheet)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass


class STLdesignUI(Plugin):

    def __init__(self):
        QDialog.__init__(self)
        # Create QDialog
        self._widget = QDialog()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('stl_experiment_ui'), 'user_interface', 'stl_controller_design.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('STL Controller Design UI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        self._widget.setWindowTitle(self._widget.windowTitle())

        self.min_sec=0.1 
        self.max_sec=6

        # Slider functionality
        # self.slider1value = self._widget.horizontalSlider.value()/10.0
        # self.slider2value = self._widget.horizontalSlider_2.value()/10.0
        # self.slider3value = self._widget.horizontalSlider_3.value()/10.0

        # self._widget.horizontalSlider.valueChanged.connect(self.changeValueSlider1)
        # self._widget.horizontalSlider_2.valueChanged.connect(self.changeValueSlider2)
        # self._widget.horizontalSlider_3.valueChanged.connect(self.changeValueSlider3)
        
       
        # self._widget.stl_g_val.connect(self.stl_g_val_change)
        # self._widget.stl_y_val.connect(self.stl_y_val_change)
        # self._widget.stl_r_val.connect(self.stl_r_val_change)


        # Button functionality
        self._widget.generateControllerButton.clicked.connect(self.generate_controller)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def generate_controller(self):
        self.slider1value = float(self._widget.stl_g_val.toPlainText())
        self.slider2value = float(self._widget.stl_y_val.toPlainText())
        self.slider3value = float(self._widget.stl_r_val.toPlainText())

        self.slider1value = max(min(self.slider1value, self.max_sec), self.min_sec)
        self.slider2value = max(min(self.slider2value, self.max_sec), self.min_sec)
        self.slider3value = max(min(self.slider3value, self.max_sec), self.min_sec)

        rospy.set_param('mpc_t_1', self.slider1value)
        rospy.set_param('mpc_t_2', self.slider2value)
        rospy.set_param('mpc_t_3', self.slider3value)
        
    def changeValueSlider1(self):
        self.slider1value = self._widget.horizontalSlider.value()/10.0
        self._widget.label_12.setText(str(self.slider1value))
    
    def changeValueSlider2(self):
        self.slider2value = self._widget.horizontalSlider_2.value()/10.0
        self._widget.label_13.setText(str(self.slider2value))
    
    def changeValueSlider3(self):
        self.slider3value = self._widget.horizontalSlider_3.value()/10.0
        self._widget.label_14.setText(str(self.slider3value))

    # def stl_g_val_change(self, text):
    #     self.slider1value=text

    # def stl_y_val_change(self, text):
    #     self.slider1value=text

    # def stl_r_val_change(self, text):
    #     self.slider1value=text

class PVdesignUI(Plugin):

    def __init__(self):
        QDialog.__init__(self)
        # Create QDialog
        self._widget = QDialog()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('stl_experiment_ui'), 'user_interface', 'pv_controller_design.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('PV Controller Design UI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        self._widget.setWindowTitle(self._widget.windowTitle())
        # Slider functionality
        # self.slider1value = self._widget.horizontalSlider.value()/100.0
        # self.slider2value = self._widget.horizontalSlider_2.value()/100.0
        # self.slider3value = self._widget.horizontalSlider_3.value()/100.0

        self.min_gain=0.1 
        self.max_gain=1

        # self._widget.horizontalSlider.valueChanged.connect(self.changeValueSlider1)
        # self._widget.horizontalSlider_2.valueChanged.connect(self.changeValueSlider2)
        # self._widget.horizontalSlider_3.valueChanged.connect(self.changeValueSlider3)

        # Button functionality
        self._widget.generateControllerButton.clicked.connect(self.generate_controller)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def generate_controller(self):
        self.slider1value = float(self._widget.pv_g_val.toPlainText())
        self.slider2value = float(self._widget.pv_y_val.toPlainText())
        self.slider3value = float(self._widget.pv_r_val.toPlainText())

        self.slider1value = max(min(self.slider1value, self.max_gain), self.min_gain)
        self.slider2value = max(min(self.slider2value, self.max_gain), self.min_gain)
        self.slider3value = max(min(self.slider3value, self.max_gain), self.min_gain)
        
        rospy.set_param('pid_kp_1', self.slider1value)
        rospy.set_param('pid_kp_2', self.slider2value)
        rospy.set_param('pid_kp_3', self.slider3value)
        
    def changeValueSlider1(self):
        self.slider1value = self._widget.horizontalSlider.value()/100.0
        self._widget.label_12.setText(str(self.slider1value))
    
    def changeValueSlider2(self):
        self.slider2value = self._widget.horizontalSlider_2.value()/100.0
        self._widget.label_13.setText(str(self.slider2value))
    
    def changeValueSlider3(self):
        self.slider3value = self._widget.horizontalSlider_3.value()/100.0
        self._widget.label_14.setText(str(self.slider3value))


class DummyedesignUI(Plugin):

    def __init__(self):
        QDialog.__init__(self)
        # Create QDialog
        self._widget = QDialog()
        # Get path to UI file which should be in the "resource" folder of this package
        ui_file = os.path.join(rospkg.RosPack().get_path('stl_experiment_ui'), 'user_interface', 'dummy_controller_design.ui')
        # Extend the widget with all attributes and children from UI file
        loadUi(ui_file, self._widget)
        # Give QObjects reasonable names
        self._widget.setObjectName('Practice Controller Design UI')
        # Show _widget.windowTitle on left-top of each plugin (when 
        # it's set in _widget). This is useful when you open multiple 
        # plugins at once. Also if you open multiple instances of your 
        # plugin at once, these lines add number to make it easy to 
        # tell from pane to pane.
        self._widget.setWindowTitle(self._widget.windowTitle())
        # Slider functionality
        self.slider1value = self._widget.horizontalSlider.value()/20.0
        self.slider2value = self._widget.horizontalSlider_2.value()/20.0
        self.slider3value = self._widget.horizontalSlider_3.value()/20.0

        self._widget.horizontalSlider.valueChanged.connect(self.changeValueSlider1)
        self._widget.horizontalSlider_2.valueChanged.connect(self.changeValueSlider2)
        self._widget.horizontalSlider_3.valueChanged.connect(self.changeValueSlider3)

        # Button functionality
        self._widget.generateControllerButton.clicked.connect(self.generate_controller)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def generate_controller(self):
        rospy.set_param('pid_kp_1', 5*self.slider1value)
        rospy.set_param('pid_kp_2', 5*self.slider2value)
        rospy.set_param('pid_kp_3', 5*self.slider3value)
        
    def changeValueSlider1(self):
        self.slider1value = self._widget.horizontalSlider.value()/20.0
        self._widget.label_12.setText(str(self.slider1value))
    
    def changeValueSlider2(self):
        self.slider2value = self._widget.horizontalSlider_2.value()/20.0
        self._widget.label_13.setText(str(self.slider2value))
    
    def changeValueSlider3(self):
        self.slider3value = self._widget.horizontalSlider_3.value()/20.0
        self._widget.label_14.setText(str(self.slider3value))