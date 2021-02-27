import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QDialog, QWidget

class ExperimentUI(Plugin):

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
        context.add_widget(self._widget)

        self._widget.hide()
        ## Controller Design Windows
        self.stl_design_ui = STLdesignUI()
        self.pv_design_ui = PVdesignUI()
        if unknowns[0] == 'mpc':
            self.stl_design_ui._widget.show()
        elif unknowns[0] == 'pid':
            self.pv_design_ui._widget.show()


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

        # Slider functionality
        self.slider1value = self._widget.horizontalSlider.value()/10.0
        self.slider2value = self._widget.horizontalSlider_2.value()/10.0
        self.slider3value = self._widget.horizontalSlider_3.value()/10.0

        self._widget.horizontalSlider.valueChanged.connect(self.changeValueSlider1)
        self._widget.horizontalSlider_2.valueChanged.connect(self.changeValueSlider2)
        self._widget.horizontalSlider_3.valueChanged.connect(self.changeValueSlider3)

        # Button functionality
        self._widget.generateControllerButton.clicked.connect(self.generate_controller)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def generate_controller(self):
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
        self.slider1value = self._widget.horizontalSlider.value()/10.0
        self.slider2value = self._widget.horizontalSlider_2.value()/10.0
        self.slider3value = self._widget.horizontalSlider_3.value()/10.0

        self._widget.horizontalSlider.valueChanged.connect(self.changeValueSlider1)
        self._widget.horizontalSlider_2.valueChanged.connect(self.changeValueSlider2)
        self._widget.horizontalSlider_3.valueChanged.connect(self.changeValueSlider3)

        # Button functionality
        self._widget.generateControllerButton.clicked.connect(self.generate_controller)

    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def generate_controller(self):
        rospy.set_param('pid_kp_1', self.slider1value)
        rospy.set_param('pid_kp_2', self.slider2value)
        rospy.set_param('pid_kp_3', self.slider3value)
        
    def changeValueSlider1(self):
        self.slider1value = self._widget.horizontalSlider.value()/10.0
        self._widget.label_12.setText(str(self.slider1value))
    
    def changeValueSlider2(self):
        self.slider2value = self._widget.horizontalSlider_2.value()/10.0
        self._widget.label_13.setText(str(self.slider2value))
    
    def changeValueSlider3(self):
        self.slider3value = self._widget.horizontalSlider_3.value()/10.0
        self._widget.label_14.setText(str(self.slider3value))