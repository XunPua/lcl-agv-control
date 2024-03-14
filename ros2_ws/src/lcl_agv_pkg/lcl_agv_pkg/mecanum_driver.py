###### Mecanum Driver ######
# Subscribes to command velocity
# Publishes wheel velocity control array

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist
import numpy as np
from agv_custom_msgs.msg import AGVSingleWheelCtrl


###### Definitions ######

# Indices
WHEEL_FL = 0
WHEEL_FR = 1
WHEEL_BL = 2
WHEEL_BR = 3
VEL_X_INDEX = 0
VEL_Y_INDEX = 1
ROT_VEL_W_INDEX = 2

# AGV Dimensions
WHEEL_RADIUS = 0.05
WHEEL_BASE = 0.125
WHEEL_SEPARATION = 0.2


###### Main Class ######
class MecanumDriver(Node):

    def __init__(self):
        super().__init__('mecanum_driver')

        # Quality of service
        qos_profile = QoSProfile(depth=10)

        # Initiate variable for storing wheel velocities
        self.current_wheel_velocities = Float64MultiArray()
        self.current_wheel_velocities.data = [0.0]*4
        
        # Subscriber for command velocities
        self.subscription_cmd_vel = self.create_subscription(
            Twist,                                              
            'command/cmd_vel',
            self.cmd_vel_callback,
            qos_profile)
        self.subscription_cmd_vel

        # Subscriber for single wheel control speed
        self.subscription_single_wheel_ctrl = self.create_subscription(
            AGVSingleWheelCtrl,                                              
            'command/single_wheel_ctrl',
            self.single_wheel_ctrl_callback,
            qos_profile)
        self.subscription_single_wheel_ctrl

        # Publisher of wheel control velocities
        self.wheel_cmd_vel_pub = self.create_publisher(Float64MultiArray, '/forward_velocity_controller/commands', qos_profile)

    # Command velocity subscriber callback 
    def cmd_vel_callback(self, msg):
        # calculate required wheel velocities for given command
        wheel_vel_array = mecanum_calc_wheel_vel(msg.linear.x, msg.linear.y, msg.angular.z)

        # publish wheel velocities to controller
        # initiate message
        wheel_vel_msg = Float64MultiArray()

        # reverse FR and BR due to motor installation position
        wheel_vel_array[WHEEL_FR] = -wheel_vel_array[WHEEL_FR]
        wheel_vel_array[WHEEL_BR] = -wheel_vel_array[WHEEL_BR]

        # set data in message
        wheel_vel_msg.data = wheel_vel_array

        # publish wheel velocity control
        self.wheel_cmd_vel_pub.publish(wheel_vel_msg)

        # save current wheel velocity
        self.current_wheel_velocities = wheel_vel_msg

    # Single wheel control subscriber callback 
    def single_wheel_ctrl_callback(self, msg):
        # check which wheel to control. if FR or BR, reverse value due to installation position
        if msg.iden == WHEEL_FR or msg.iden == WHEEL_BR:
            msg.wheel_speed_rad_sec = -msg.wheel_speed_rad_sec

        # set new single wheel control speed onto current wheel velocities
        self.current_wheel_velocities.data[msg.iden] = msg.wheel_speed_rad_sec

        # Publish wheel control 
        self.wheel_cmd_vel_pub.publish(self.current_wheel_velocities)


###### Functions ######

# Inverse Kinematics of a Mecanum Wheel AGV
# calculate velocity for all 4 wheels from AGV CoM velocity
# returns velocities as array of [FL, FR, BL, BR]
def mecanum_calc_wheel_vel(vx, vy, w):
    wheel_vel_array = [0.0]*4
    wheel_vel_array[WHEEL_FL] = (vx - vy - (WHEEL_BASE + WHEEL_SEPARATION) * w) / WHEEL_RADIUS
    wheel_vel_array[WHEEL_FR] = (vx + vy + (WHEEL_BASE + WHEEL_SEPARATION) * w) / WHEEL_RADIUS
    wheel_vel_array[WHEEL_BL] = (vx + vy - (WHEEL_BASE + WHEEL_SEPARATION) * w) / WHEEL_RADIUS
    wheel_vel_array[WHEEL_BR] = (vx - vy + (WHEEL_BASE + WHEEL_SEPARATION) * w) / WHEEL_RADIUS
    return wheel_vel_array


###### Main Function ######
def main():
    rclpy.init()
    node = MecanumDriver()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()