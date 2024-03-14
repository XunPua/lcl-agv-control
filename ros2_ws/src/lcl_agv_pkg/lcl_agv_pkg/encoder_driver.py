###### Encoder Driver ######
# Subscribes to Joint state publisher
# Publishes Odometry message
# OPTIONAL broadcasts Odometry transformation for No-EKF node cases

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from math import cos, sin
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster, TransformStamped

###### Definitions ######

# Indices
WHEEL_FL = 0
WHEEL_FR = 1
WHEEL_BL = 2
WHEEL_BR = 3
VEL_X_INDEX = 0
VEL_Y_INDEX = 1
ROT_VEL_W_INDEX = 2

# AGV dimensions
WHEEL_RADIUS = 0.05
WHEEL_BASE = 0.125
WHEEL_SEPARATION = 0.175

# Twist covariance
LIN_X_COV = 0.1
LIN_Y_COV = 0.1
ANG_Z_COV = 0.1

TWIST_COVARIANCE = [
    LIN_X_COV, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, LIN_Y_COV, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, ANG_Z_COV
]


###### Main Class ######
class EncoderDriver(Node):

    def __init__(self):
        
        super().__init__('encoder_driver')

        # define quality of service
        qos_profile = QoSProfile(depth=10)

        # initiate previous time for integration
        self.prev_time = None  

        # initiate robot position
        self.robot_position = [0.0]*3  

        # Subscriber for joint state broadcaster. Gets current AGV Joint positions and velocities
        self.subscription_joint_state_broadcaster = self.create_subscription(
            JointState,                                              
            '/joint_state_broadcaster/joint_states',
            self.joint_state_broadcaster_callback,
            qos_profile)
        self.subscription_joint_state_broadcaster   

        # Publisher of AGV Odometry
        self.odom_pub = self.create_publisher(Odometry, 'robot/odom', qos_profile)

        # Publisher for (corrected) Joint States. Extra step incase special processing needed.
        self.joint_state_pub = self.create_publisher(JointState, 'robot/joint_states', qos_profile)

        # Odometry Transform Broadcaster (ONLY USED WHEN NO EKF NODE!!!)
        self.tf_broadcaster = TransformBroadcaster(self)  # SPECIAL CASE FOR NO EKF NODE!


    # Joint state broadcaster subscriber callback
    def joint_state_broadcaster_callback(self, msg):

        joint_state = msg

        # Add any necessary processing here

        # publish processed joint states
        self.joint_state_pub.publish(joint_state)   

        ### Calculate Odometry ###

        # get current time
        if self.prev_time is None:
            self.prev_time = self.get_clock().now()
        current_time = self.get_clock().now()

        # calculate time diff from previous callback
        time_diff = current_time - self.prev_time

        # invert FR and BR velocity due to installation position
        msg.velocity[WHEEL_FR] = -msg.velocity[WHEEL_FR]
        msg.velocity[WHEEL_BR] = -msg.velocity[WHEEL_BR]

        # estimate AGV position change using forward kinematics and integration
        robot_vel_array, delta_pos_array = mecanum_calc_forward_kin(msg.velocity, (time_diff.nanoseconds / 1e9))

        # set current time as previous time
        self.prev_time = current_time

        # transform AGV pose to global coordinate system
        self.robot_position[0] += delta_pos_array[0] * cos(self.robot_position[2]) - delta_pos_array[1] * sin(self.robot_position[2])
        self.robot_position[1] += delta_pos_array[0] * sin(self.robot_position[2]) + delta_pos_array[1] * cos(self.robot_position[2])
        self.robot_position[2] += delta_pos_array[2]

        # Prepare Odometry message
        # headers
        odom_msg = Odometry()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.header.stamp = self.get_clock().now().to_msg()

        # pose
        odom_msg.pose.pose.position.x = self.robot_position[0]
        odom_msg.pose.pose.position.y = self.robot_position[1]
        odom_msg.pose.pose.orientation = euler_to_quaternion(0, 0, self.robot_position[2])  # convert yaw to quaternion angles

        # twist
        odom_msg.twist.twist.linear.x = robot_vel_array[0]
        odom_msg.twist.twist.linear.y = robot_vel_array[1]
        odom_msg.twist.twist.angular.z = robot_vel_array[2]
        odom_msg.twist.covariance = TWIST_COVARIANCE

        # Publish Odometry msg
        self.odom_pub.publish(odom_msg)


        ### SPECIAL CASE FOR IF EKF NODE IS NOT ACTIVATE!!! ###

        # # Prepare odometry transformation broadcast
        # t = TransformStamped()
        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'odom'
        # t.child_frame_id = 'base_link'

        # t.transform.translation.x = self.robot_position[0]
        # t.transform.translation.y = self.robot_position[1]
        # t.transform.translation.z = 0.0
        # q = euler_to_quaternion(0, 0, self.robot_position[2])
        # t.transform.rotation = q

        # # broadcast odometry transformation
        # self.tf_broadcaster.sendTransform(t)

        ### END OF SPECIAL CASE FOR IF EKF NODE IS NOT ACTIVATE!!! ###
        

###### Functions ######

# Forward Kinematics of Mecanum Wheel AGV
# calculate robot CoM velocity and estimated position change from wheel velocities and time diff between two readings
# returns robot velocity array as [vx, vy, w] and change in position array as [dx, dy, dw]
def mecanum_calc_forward_kin(wheel_vel_array, time_diff):
    robot_vel_array = [0.0] * 3
    robot_vel_array[VEL_X_INDEX] = (wheel_vel_array[WHEEL_FL] + wheel_vel_array[WHEEL_FR] + wheel_vel_array[WHEEL_BL] + wheel_vel_array[WHEEL_BR]) * WHEEL_RADIUS / 4
    robot_vel_array[VEL_Y_INDEX] = (-wheel_vel_array[WHEEL_FL] + wheel_vel_array[WHEEL_FR] + wheel_vel_array[WHEEL_BL] - wheel_vel_array[WHEEL_BR]) * WHEEL_RADIUS / 4
    robot_vel_array[ROT_VEL_W_INDEX] = (-wheel_vel_array[WHEEL_FL] + wheel_vel_array[WHEEL_FR] - wheel_vel_array[WHEEL_BL] + wheel_vel_array[WHEEL_BR]) * WHEEL_RADIUS / (4 * (WHEEL_BASE + WHEEL_SEPARATION))

    delta_pos_array = [0.0] * 3
    delta_pos_array[VEL_X_INDEX] = robot_vel_array[VEL_X_INDEX] * time_diff
    delta_pos_array[VEL_Y_INDEX] = robot_vel_array[VEL_Y_INDEX] * time_diff
    delta_pos_array[ROT_VEL_W_INDEX] = robot_vel_array[ROT_VEL_W_INDEX] * time_diff
    return robot_vel_array, delta_pos_array


# Convert Euler angles to Quaternions
def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)  


###### Main Function ######
def main():
    rclpy.init()
    node = EncoderDriver()

    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()