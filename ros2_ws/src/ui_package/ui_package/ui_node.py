import sys
import signal
import os
import datetime
from math import pi

# Qt Libraries
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel, QPushButton, QSlider, \
                            QDoubleSpinBox, QAbstractSpinBox, QTextEdit, QLineEdit, QSpacerItem, QSizePolicy, QSplitter
from PyQt5.QtCore import Qt, QSize
from PyQt5.QtGui import QIcon

# Ros Libraries
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Float64MultiArray
from agv_custom_msgs.msg import AGVSingleWheelCtrl
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory


###### Definitions ######
WHEEL_FL = 0
WHEEL_FR = 1
WHEEL_BL = 2
WHEEL_BR = 3


###### Slider with numbered Labels ######
class CustomLabeledSlider(QWidget):
    def __init__(self):
        super().__init__()

        # use this to add the labeled slider to layouts
        self.custom_slider_layout = QVBoxLayout() 

        # Create slider object
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimumWidth(200)
        self.slider.setDisabled(True)  # Read only
        self.multiplier = 1

        # Create measured value spinbox object
        self.sb_measured = QDoubleSpinBox()
        self.sb_measured.setDisabled(True)
        self.sb_measured.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.sb_measured.setPrefix("Meas.: ")
        self.sb_measured.setAlignment(Qt.AlignCenter)
        
        # Create goal value spinbox object
        self.sb_goal = QDoubleSpinBox()
        self.sb_goal.setDisabled(True)
        self.sb_goal.setButtonSymbols(QAbstractSpinBox.NoButtons)
        self.sb_goal.setPrefix("Goal: ")
        self.sb_goal.setAlignment(Qt.AlignCenter)

    #### set the min and max value of the labels ####
    def setSliderParams(self, start=0, min=0, max=0, multiplier=1):
        self.multiplier = multiplier
        
        self.slider.setMinimum(int(min))
        self.slider.setMaximum(int(max * self.multiplier))  # set with 0.01 rad. accuracy
        self.slider.setValue(start)
        
        self.sb_measured.setRange(min, max)
        self.sb_goal.setRange(min, max)

        # set labels and ticks
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.label_min = QLabel("%0.2f" % min, alignment=Qt.AlignLeft)
        self.label_min.setMinimumWidth(50)
        self.label_max = QLabel("%0.2f" % max, alignment=Qt.AlignRight)
        self.label_max.setMinimumWidth(50)

        # set slider layout
        self.custom_slider_layout.addWidget(self.slider)
        

    #### Set the step of the spinbox showing the slider value ####
    def setSpinBoxParams(self, showGoal: bool=False):
        slider_hbox = QHBoxLayout()
        
        slider_hbox.addWidget(self.label_min, Qt.AlignLeft)
        
        if showGoal is True:
            slider_hbox.addWidget(self.sb_goal, Qt.AlignCenter)
        
        slider_hbox.addWidget(self.sb_measured, Qt.AlignCenter)
        slider_hbox.addWidget(self.label_max, Qt.AlignRight)
        self.custom_slider_layout.addLayout(slider_hbox)

    ### Set the value of the slider and spinbox ####
    def setMeasuredValue(self, value):
        self.slider.setValue(int(self.multiplier * value))
        self.sb_measured.setValue(value)
        
    def setGoalValue(self, value):
        self.sb_goal.setValue(value)


###### spinbox with pushbutton for setting individual wheel speed ######
class IndividualWheelControlWidget(QWidget):
    def __init__(self, max_speed):
        super().__init__()
        self.spinbox = QDoubleSpinBox()
        self.spinbox.setDecimals(2)
        self.spinbox.setRange(-max_speed, max_speed)
        self.spinbox.setMinimumWidth(100)
        self.spinbox.setValue(30.0)
        self.spinbox.setSingleStep(10.0)
        self.spinbox.setAlignment(Qt.AlignRight)

        # Pushbutton to send values. 
        self.pushbutton = QPushButton("Rotate")

        self.hbox = QHBoxLayout()  # Use this to add horizontal layout
        self.hbox.addWidget(self.spinbox)
        self.hbox.addWidget(self.pushbutton)
        
    def getValue(self):
        return self.spinbox.value()


###### ROS Node ######
class ROSNode(Node):
    def __init__(self):
        super().__init__('ui_node')
        
        # joint state subscriber
        self.joint_state_msg = JointState()
        self.joint_state_msg.position = [0.0]*4
        self.joint_state_msg.velocity = [0.0]*4
        self.wheel_encoder_subscriber = self.create_subscription(JointState, 'robot/joint_states', self.wheelEncoderCallback, 10)
        self.wheel_encoder_subscriber  # prevent unused variable warning
        
        # wheel velocity command subscriber
        self.current_wheel_command_velocities = Float64MultiArray()
        self.current_wheel_command_velocities.data = [0.0]*4
        self.command_velocity_subscriber = self.create_subscription(Float64MultiArray, 'forward_velocity_controller/commands', self.commandVelocityCallback, 10)
        self.command_velocity_subscriber # prevent unused variable warning
        
        # command twist publisher 
        self.twist_publisher = self.create_publisher(Twist, 'gui/cmd_vel', 10)

        # single wheel control publisher
        self.single_wheel_ctrl_publisher = self.create_publisher(AGVSingleWheelCtrl, 'command/single_wheel_ctrl', 10)
    
    # joint state subscriber callback
    def wheelEncoderCallback(self, msg):
        self.joint_state_msg = msg
    
    # current wheel velocity command callback
    def commandVelocityCallback(self, msg):
        self.current_wheel_command_velocities = msg


###### Main UI with ROS2 Node ######
class UIClass(QWidget):
    def __init__(self, ros_node: ROSNode):
        # super().__init__()
        QWidget.__init__(self)

        # handle ctrl c signal
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        
        # ros node
        self.ros_node = ros_node
        
        # initialize UI
        self.initUI()
        
    ###### User Interface Functions ######
    #### Initialization ####
    def initUI(self):
        #### Window Settings ####
        self.setWindowTitle('AGV Control Interface')
        
        #### Wheel Feedback and Control ####
        self.initWheelFeedBackandCtrl()
        
        #### Basic Control ####
        self.initBasicControl()
        
        #### Show current command ####
        self.le_current_command = QLineEdit()
        self.le_current_command.setEnabled(False)
        self.initSideLogger()

        #### Vertical Spacer ####
        v_spacer = QSpacerItem(20, 20, QSizePolicy.Minimum, QSizePolicy.Expanding)

        #### Set Main Layout ####
        # Control and FB layout
        ctrl_layout = QVBoxLayout()
        ctrl_layout.addItem(v_spacer)
        ctrl_layout.addLayout(self.grid_layout_wheel_fb_ctrl)
        ctrl_layout.addItem(v_spacer)
        ctrl_layout.addLayout(self.basic_control_layout)
        ctrl_layout.addItem(v_spacer)
        ctrl_layout.addWidget(self.le_current_command)
        
        # Container for control and FB layout
        # used to implement splitter, as splitter can only take Widgets and not layouts
        container = QWidget()
        container.setLayout(ctrl_layout)
        
        # Splitter
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(container)
        splitter.addWidget(self.te_side_logger)
        
        layout = QHBoxLayout()
        layout.addWidget(splitter)
        self.setLayout(layout)
        
        #### Change Focus to Button ####
        self.pb_basic_ctrl_forward.setFocus()  # So that the user can directly use keyboard shortcuts
    
    #### Side Logger ####
    def initSideLogger(self):
        self.te_side_logger = QTextEdit()
        self.te_side_logger.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.te_side_logger.setMinimumWidth(200)
        self.te_side_logger.append('<b>Command Logger</b> <br>')
    
    #### Initialize Wheel Feedback and Individual Control section ####
    def initWheelFeedBackandCtrl(self):
        ###### Wheel Encoder Feedback ######
        max_speed = 100
        #### Front Left ####
        ## Position ##
        self.sl_fl_pos = CustomLabeledSlider()
        self.sl_fl_pos.setSliderParams(0, 0, 2 * pi, 100)
        self.sl_fl_pos.setSpinBoxParams(showGoal=False)

        ## Speed ##
        self.sl_fl_spd = CustomLabeledSlider()
        self.sl_fl_spd.setSliderParams(0, -max_speed, max_speed, 1)
        self.sl_fl_spd.setSpinBoxParams(showGoal=True)

        #### Front Right ####
        ## Position ##
        self.sl_fr_pos = CustomLabeledSlider()
        self.sl_fr_pos.setSliderParams(0, 0, 2 * pi, 100)
        self.sl_fr_pos.setSpinBoxParams(showGoal=False)

        ## Speed ##
        self.sl_fr_spd = CustomLabeledSlider()
        self.sl_fr_spd.setSliderParams(0, -max_speed, max_speed, 1)
        self.sl_fr_spd.setSpinBoxParams(showGoal=True)

        #### Back Left ####
        ## Position ##
        self.sl_bl_pos = CustomLabeledSlider()
        self.sl_bl_pos.setSliderParams(0, 0, 2 * pi, 100)
        self.sl_bl_pos.setSpinBoxParams(showGoal=False)

        ## Speed ##
        self.sl_bl_spd = CustomLabeledSlider()
        self.sl_bl_spd.setSliderParams(0, -max_speed, max_speed, 1)
        self.sl_bl_spd.setSpinBoxParams(showGoal=True)

        #### Back Right ####
        ## Position ##
        self.sl_br_pos = CustomLabeledSlider()
        self.sl_br_pos.setSliderParams(0, 0, 2 * pi, 100)
        self.sl_br_pos.setSpinBoxParams(showGoal=False)

        ## Speed ##
        self.sl_br_spd = CustomLabeledSlider()
        self.sl_br_spd.setSliderParams(0, -max_speed, max_speed, 1)
        self.sl_br_spd.setSpinBoxParams(showGoal=True)


        ###### Individual Wheel Control ######
        #### Front Left ####
        self.sb_fl_spd_ctrl = IndividualWheelControlWidget(max_speed)
        self.sb_fl_spd_ctrl.pushbutton.pressed.connect(lambda: self.onIndividualWheelSpeedCtrlPressed(self.sb_fl_spd_ctrl.spinbox.value(), WHEEL_FL))
        self.sb_fl_spd_ctrl.pushbutton.released.connect(self.onIndividualWheelSpeedCtrlReleased)

        #### Front Right ####
        self.sb_fr_spd_ctrl = IndividualWheelControlWidget(max_speed)
        self.sb_fr_spd_ctrl.pushbutton.pressed.connect(lambda: self.onIndividualWheelSpeedCtrlPressed(self.sb_fr_spd_ctrl.spinbox.value(), WHEEL_FR))
        self.sb_fr_spd_ctrl.pushbutton.released.connect(self.onIndividualWheelSpeedCtrlReleased)
        
        #### Back Left ####
        self.sb_bl_spd_ctrl = IndividualWheelControlWidget(max_speed)
        self.sb_bl_spd_ctrl.pushbutton.pressed.connect(lambda: self.onIndividualWheelSpeedCtrlPressed(self.sb_bl_spd_ctrl.spinbox.value(), WHEEL_BL))
        self.sb_bl_spd_ctrl.pushbutton.released.connect(self.onIndividualWheelSpeedCtrlReleased)
        
        #### Back Right ####
        self.sb_br_spd_ctrl = IndividualWheelControlWidget(max_speed)
        self.sb_br_spd_ctrl.pushbutton.pressed.connect(lambda: self.onIndividualWheelSpeedCtrlPressed(self.sb_br_spd_ctrl.spinbox.value(), WHEEL_BR))
        self.sb_br_spd_ctrl.pushbutton.released.connect(self.onIndividualWheelSpeedCtrlReleased)

        ###### Layout ######        
        self.grid_layout_wheel_fb_ctrl = QGridLayout()
        
        #### Headers ####
        self.grid_layout_wheel_fb_ctrl.addWidget(QLabel("<b>Rel. Position / rad.</b>", alignment=Qt.AlignCenter), 0, 1)
        self.grid_layout_wheel_fb_ctrl.addWidget(QLabel("<b>Speed / RPM</b>", alignment=Qt.AlignCenter), 0, 2)
        self.grid_layout_wheel_fb_ctrl.addWidget(QLabel("<b>Individual Wheel Control / RPM</b>", alignment=Qt.AlignCenter), 0, 3)
        
        #### Front Left ####
        label = QLabel("<b>FRONT LEFT</b>", alignment=Qt.AlignLeft)
        label.setMinimumWidth(100)
        self.grid_layout_wheel_fb_ctrl.addWidget(label, 1, 0)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sl_fl_pos.custom_slider_layout, 1, 1)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sl_fl_spd.custom_slider_layout, 1, 2)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sb_fl_spd_ctrl.hbox, 1, 3)
        
        #### Front Right ####
        self.grid_layout_wheel_fb_ctrl.addWidget(QLabel("<b>FRONT RIGHT</b>", alignment=Qt.AlignLeft), 2, 0)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sl_fr_pos.custom_slider_layout, 2, 1)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sl_fr_spd.custom_slider_layout, 2, 2)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sb_fr_spd_ctrl.hbox, 2, 3)
        
        #### Back Left ####
        self.grid_layout_wheel_fb_ctrl.addWidget(QLabel("<b>BACK LEFT</b>", alignment=Qt.AlignLeft), 3, 0)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sl_bl_pos.custom_slider_layout, 3, 1)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sl_bl_spd.custom_slider_layout, 3, 2)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sb_bl_spd_ctrl.hbox, 3, 3)
        
        #### Back Right ####
        self.grid_layout_wheel_fb_ctrl.addWidget(QLabel("<b>BACK RIGHT</b>", alignment=Qt.AlignLeft), 4, 0)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sl_br_pos.custom_slider_layout, 4, 1)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sl_br_spd.custom_slider_layout, 4, 2)
        self.grid_layout_wheel_fb_ctrl.addLayout(self.sb_br_spd_ctrl.hbox, 4, 3)
        
    #### Initialize Basic Control Section (buttons for simple maneuvers) ####
    def initBasicControl(self):
        #### Forward ####
        self.pb_basic_ctrl_forward = QPushButton("")
        self.pb_basic_ctrl_forward.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'front_arrow.png')))
        self.pb_basic_ctrl_forward.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_forward.pressed.connect(lambda: self.onBasicControlPressed("forward"))
        self.pb_basic_ctrl_forward.released.connect(self.stop)
        self.pb_basic_ctrl_forward.setToolTip("Forward [W]")
        
        #### Backward ####
        self.pb_basic_ctrl_backward = QPushButton("")
        self.pb_basic_ctrl_backward.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'back_arrow.png')))
        self.pb_basic_ctrl_backward.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_backward.pressed.connect(lambda: self.onBasicControlPressed("backward"))
        self.pb_basic_ctrl_backward.released.connect(self.stop)
        self.pb_basic_ctrl_backward.setToolTip("Backward [X]")
        
        #### Right ####
        self.pb_basic_ctrl_right = QPushButton("")
        self.pb_basic_ctrl_right.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'right_arrow.png')))
        self.pb_basic_ctrl_right.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_right.pressed.connect(lambda: self.onBasicControlPressed("right"))
        self.pb_basic_ctrl_right.released.connect(self.stop)
        self.pb_basic_ctrl_right.setToolTip("Right [D]")
        
        #### Left ####
        self.pb_basic_ctrl_left = QPushButton("")
        self.pb_basic_ctrl_left.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'left_arrow.png')))
        self.pb_basic_ctrl_left.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_left.pressed.connect(lambda: self.onBasicControlPressed("left"))
        self.pb_basic_ctrl_left.released.connect(self.stop)
        self.pb_basic_ctrl_left.setToolTip("Left [A]")
               
        #### Forward Left ####
        self.pb_basic_ctrl_forward_left = QPushButton("")
        self.pb_basic_ctrl_forward_left.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'front_left_arrow.png')))
        self.pb_basic_ctrl_forward_left.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_forward_left.pressed.connect(lambda: self.onBasicControlPressed("forward_left"))
        self.pb_basic_ctrl_forward_left.released.connect(self.stop)
        self.pb_basic_ctrl_forward_left.setToolTip("Forward Left [Q]")
        
        #### Forward Right ####
        self.pb_basic_ctrl_forward_right = QPushButton("")
        self.pb_basic_ctrl_forward_right.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'front_right_arrow.png')))
        self.pb_basic_ctrl_forward_right.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_forward_right.pressed.connect(lambda: self.onBasicControlPressed("forward_right"))
        self.pb_basic_ctrl_forward_right.released.connect(self.stop)
        self.pb_basic_ctrl_forward_right.setToolTip("Forward Right [E]")
        
        #### Backward Left ####
        self.pb_basic_ctrl_backward_left = QPushButton("")
        self.pb_basic_ctrl_backward_left.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'back_left_arrow.png')))
        self.pb_basic_ctrl_backward_left.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_backward_left.pressed.connect(lambda: self.onBasicControlPressed("backward_left"))
        self.pb_basic_ctrl_backward_left.released.connect(self.stop)
        self.pb_basic_ctrl_backward_left.setToolTip("Backward Left [Y]")
        
        #### Backward Right ####
        self.pb_basic_ctrl_backward_right = QPushButton("")
        self.pb_basic_ctrl_backward_right.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'back_right_arrow.png')))
        self.pb_basic_ctrl_backward_right.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_backward_right.pressed.connect(lambda: self.onBasicControlPressed("backward_right"))
        self.pb_basic_ctrl_backward_right.released.connect(self.stop)
        self.pb_basic_ctrl_backward_right.setToolTip("Backward Right [C]")
        
        #### Clockwise ####
        self.pb_basic_ctrl_cw = QPushButton("")
        self.pb_basic_ctrl_cw.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'cw_arrow.png')))
        self.pb_basic_ctrl_cw.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_cw.pressed.connect(lambda: self.onBasicControlPressed("cw"))
        self.pb_basic_ctrl_cw.released.connect(self.stop)
        self.pb_basic_ctrl_cw.setToolTip("Clockwise [L]")
        
        #### Counter Clockwise ####
        self.pb_basic_ctrl_ccw = QPushButton("")
        self.pb_basic_ctrl_ccw.setIcon(QIcon(os.path.join(get_package_share_directory('ui_package'), 'icons', 'ccw_arrow.png')))
        self.pb_basic_ctrl_ccw.setIconSize(QSize(50, 50))
        self.pb_basic_ctrl_ccw.pressed.connect(lambda: self.onBasicControlPressed("ccw"))
        self.pb_basic_ctrl_ccw.released.connect(self.stop)
        self.pb_basic_ctrl_ccw.setToolTip("Counter Clockwise [J]")
        
        #### Speed Settings ####
        self.sb_basic_ctrl_lin_spd = QDoubleSpinBox()
        self.sb_basic_ctrl_lin_spd.setMaximum(3.0)
        self.sb_basic_ctrl_lin_spd.setSingleStep(0.1)
        self.sb_basic_ctrl_lin_spd.setValue(0.2)
        self.sb_basic_ctrl_lin_spd.setSuffix(" m/s")
        self.sb_basic_ctrl_lin_spd.setAlignment(Qt.AlignCenter)
        
        self.sb_basic_ctrl_rot_spd = QDoubleSpinBox()
        self.sb_basic_ctrl_rot_spd.setMaximum(360.0)
        self.sb_basic_ctrl_rot_spd.setSingleStep(15)
        self.sb_basic_ctrl_rot_spd.setValue(30)
        self.sb_basic_ctrl_rot_spd.setSuffix(" °/s")
        self.sb_basic_ctrl_rot_spd.setAlignment(Qt.AlignCenter)
                
        
        #### Layout ####
        basic_control_grid_layout = QGridLayout()
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_forward_left, 0, 1)
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_forward, 0, 2)
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_forward_right, 0, 3)
        
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_ccw, 1, 0)
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_left, 1, 1)
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_right, 1, 3)
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_cw, 1, 4)
        
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_backward_left, 2, 1)
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_backward, 2, 2)
        basic_control_grid_layout.addWidget(self.pb_basic_ctrl_backward_right, 2, 3)
        
        speed_control_hbox = QHBoxLayout()
        speed_control_hbox.addWidget(QLabel("Linear Speed ", alignment=Qt.AlignRight))
        speed_control_hbox.addWidget(self.sb_basic_ctrl_lin_spd)
        speed_control_hbox.addWidget(QLabel("Rotational Speed ", alignment=Qt.AlignRight))
        speed_control_hbox.addWidget(self.sb_basic_ctrl_rot_spd)
        
        self.basic_control_layout = QVBoxLayout()
        self.basic_control_layout.addLayout(basic_control_grid_layout)
        self.basic_control_layout.addLayout(speed_control_hbox)
    
    #### Update Foot logger with current command ####
    def updateCurrentCommand(self, text):

        # Get the current date and time
        current_datetime = datetime.datetime.now()

        # Format the date and time as YYYY-MM-DD: HH:MM:SS:MS
        formatted_datetime = current_datetime.strftime('%Y-%m-%d: %H:%M:%S')
        
        # show current command
        msg = formatted_datetime + " -> " + text
        self.le_current_command.setText(msg)
        self.te_side_logger.append(msg + '<br>')
    
    ###### Handle Buttons and Keypress ######
    #### Callback Function for when "Rotate" of individual wheel ctrl is pressed ####
    def onIndividualWheelSpeedCtrlPressed(self, speed, index):
        
        text = 'Individual Wheel Control: '
        if index == WHEEL_FL:
            text = text + 'Front Left -> %0.2f RPM' % speed
        elif index == WHEEL_FR:
            text = text + 'Front Right -> %0.2f RPM' % speed
        elif index == WHEEL_BL:
            text = text + 'Back Left -> %0.2f RPM' % speed
        elif index == WHEEL_BR:
            text = text + 'Back Right -> %0.2f RPM' % speed
        else:
            return
        
        self.pubIndividualWheelVelocity(index, speed * (2 * pi) / 60)
        self.ros_node.get_logger().info(text)
        self.updateCurrentCommand(text)

    #### Callback Function for when "Rotate" of individual wheel ctrl is released ####
    def onIndividualWheelSpeedCtrlReleased(self):
        self.stop()
        
    #### Handle Shortcut Keys when Pressed ####
    def keyPressEvent(self, event):            
        key = event.key()
        if event.isAutoRepeat() is False:  # to prevent multiple triggers when key is hold down
            if key == Qt.Key_W:
                self.onBasicControlPressed("forward")
            elif key == Qt.Key_X:
                self.onBasicControlPressed("backward")
            elif key == Qt.Key_D:
                self.onBasicControlPressed("right")
            elif key == Qt.Key_A:
                self.onBasicControlPressed("left")
            elif key == Qt.Key_Q:
                self.onBasicControlPressed("forward_left")
            elif key == Qt.Key_E:
                self.onBasicControlPressed("forward_right")
            elif key == Qt.Key_Y:
                self.onBasicControlPressed("backward_left")
            elif key == Qt.Key_C:
                self.onBasicControlPressed("backward_right")
            elif key == Qt.Key_J:
                self.onBasicControlPressed("ccw")
            elif key == Qt.Key_L:
                self.onBasicControlPressed("cw")
            elif key == Qt.Key_Enter:
                self.pb_basic_ctrl_forward.setFocus()  # Reset focus to basic control buttons
            elif key == Qt.Key_Return:
                self.pb_basic_ctrl_forward.setFocus()  # Reset focus to basic control buttons
            
        return super(UIClass, self).keyPressEvent(event)  
    
    #### Handle Shortcut Keys when Released ####
    def keyReleaseEvent(self, event) -> None:
        if event.isAutoRepeat() is False:  # to prevent multiple triggers when key is hold down
            # List contains all basic command shortcut keys
            # AGV should stop moving when user releases key
            list = [Qt.Key_W, Qt.Key_X, Qt.Key_D, Qt.Key_A, Qt.Key_Q, Qt.Key_E, Qt.Key_Y, Qt.Key_C, Qt.Key_J, Qt.Key_L]
            if event.key() in list:
                self.stop()
        
        return super(UIClass, self).keyPressEvent(event)  
    
    #### Publish CMD Msgs for Basic Commands ####
    def onBasicControlPressed(self, command):
        # Get speed setting values
        lin_speed = self.sb_basic_ctrl_lin_spd.value()
        rot_speed = self.sb_basic_ctrl_rot_spd.value()
        
        text = 'Basic Command: '
        speed = [0.0]*3
        
        # Handle different commands
        if command == "forward":
            speed = [lin_speed, 0.0, 0.0]
            text = text + 'Forward '

        elif command == "backward":
            speed = [-lin_speed, 0.0, 0.0]
            text = text + 'Backward '

        elif command == "right":
            speed = [0.0, -lin_speed, 0.0]
            text = text + 'Right '
            
        elif command == "left":
            speed = [0.0, lin_speed, 0.0]
            text = text + 'Left '
            
        elif command == "forward_left":
            speed = [lin_speed / 2, lin_speed / 2, 0.0]
            text = text + 'Forward Left '
            
        elif command == "forward_right":
            speed = [lin_speed / 2, -lin_speed / 2, 0.0]
            text = text + 'Forward Right '
            
        elif command == "backward_left":
            speed = [-lin_speed / 2, lin_speed / 2, 0.0]
            text = text + 'Backward Left '
            
        elif command == "backward_right":
            speed = [-lin_speed / 2, -lin_speed / 2, 0.0]
            text = text + 'Backward Right '
            
        elif command == "cw":
            speed = [0.0, 0.0, -rot_speed * pi / 180]
            text = text + 'Clockwise Rotation '
            
        elif command == "ccw":
            speed = [0.0, 0.0, rot_speed * pi / 180]
            text = text + 'Counter Clockwise Rotation '
        
        text = text + ' -> vy: ' + str(speed[0]) + ' m/s - vy: ' + str(speed[1]) + ' m/s - w: %0.2f' % speed[2] + ' rad/s'
        self.pubTwistMsg(speed[0], speed[1], speed[2])
        self.ros_node.get_logger().info(text)
        self.updateCurrentCommand(text)
    
    #### STOP ####
    def stop(self):
        self.pubTwistMsg(0.0, 0.0, 0.0)
        self.ros_node.get_logger().info('Stop')
        self.updateCurrentCommand("Stop")
    
    #### Handle Ctrl+C ####
    def handleSignal(self, signum, frame):
        QApplication.quit()
        
        
    ###### Subscription Callbacks #######
    #### Wheel encoder measured values ####
    def updateEncoderCommandFeedbackValues(self):
        joint_state = self.ros_node.joint_state_msg
        self.sl_fl_pos.setMeasuredValue(abs(joint_state.position[WHEEL_FL] % (2*pi)))
        self.sl_fl_spd.setMeasuredValue(joint_state.velocity[WHEEL_FL] * 60 / (2 * pi))
        self.sl_fr_pos.setMeasuredValue(abs(-joint_state.position[WHEEL_FR] % (2 * pi)))
        self.sl_fr_spd.setMeasuredValue(-joint_state.velocity[WHEEL_FR] * 60 / (2 * pi))
        self.sl_bl_pos.setMeasuredValue(abs(joint_state.position[WHEEL_BL] % (2 * pi)))
        self.sl_bl_spd.setMeasuredValue(joint_state.velocity[WHEEL_BL] * 60 / (2 * pi))
        self.sl_br_pos.setMeasuredValue(abs(-joint_state.position[WHEEL_BR] % (2 * pi)))
        self.sl_br_spd.setMeasuredValue(-joint_state.velocity[WHEEL_BR] * 60 / (2 * pi))
        
        command = self.ros_node.current_wheel_command_velocities
        self.sl_fl_spd.setGoalValue(command.data[WHEEL_FL] * 60 / (2 * pi))
        self.sl_fr_spd.setGoalValue(-command.data[WHEEL_FR] * 60 / (2 * pi))
        self.sl_bl_spd.setGoalValue(command.data[WHEEL_BL] * 60 / (2 * pi))
        self.sl_br_spd.setGoalValue(-command.data[WHEEL_BR] * 60 / (2 * pi))
    
    
    ###### Publishers #######
    #### Twist ####
    def pubTwistMsg(self, vx=0.0, vy=0.0, w=0.0):
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = w
        
        self.ros_node.twist_publisher.publish(msg)
        
    #### Single Wheel Control ####
    def pubIndividualWheelVelocity(self, iden, speed_rad_sec):
        if iden in range(4):
            msg = AGVSingleWheelCtrl()
            msg.iden = iden
            msg.wheel_speed_rad_sec = speed_rad_sec
            self.ros_node.single_wheel_ctrl_publisher.publish(msg)          
        
    
###### MAIN ######
def main(args=None):
    
    app = QApplication(sys.argv)
    rclpy.init()
    
    # init ros node class
    ros_node = ROSNode()

    # init qt widget with ros node as variable
    window = UIClass(ros_node)
    
    # show UI
    app.processEvents()
    window.show()

    # Multithreadedexecutor needed to run qt and ros simultaneously
    exec = MultiThreadedExecutor()
    exec.add_node(ros_node)
    
    ros_node.get_logger().info("Starting User Interface ...")
    
    while rclpy.ok():
        exec.spin_once(timeout_sec=0.1)
        app.processEvents()
        window.updateEncoderCommandFeedbackValues()

    ros_node.get_logger().info("Received SIGINT, shutting down...")
    exec.remove_node(ros_node)
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit()


if __name__ == '__main__':
    main()
