###### IMU Driver ######
# Subscribes to IMU broadcaster
# Publishes transformed IMU message with covariance
# transformation actually not needed if xacro file is done properly! This is to be changed!

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Quaternion
import math


###### Definitions ######

# orientation covariance
imu_orientation_covariance = [0.05, 0.0, 0.0, 
                            0.0, 0.05, 0.0, 
                            0.0, 0.0, 0.05 ]

# gyroscope covarianve
imu_ang_vel_covariance = [0.05, 0.0, 0.0, 
                        0.0, 0.05, 0.0, 
                        0.0, 0.0, 0.05 ]

# acceleration covariance
imu_accel_covariance = [0.05, 0.0, 0.0, 
                        0.0, 0.05, 0.0, 
                        0.0, 0.0, 0.05 ]


###### Main Class ######
class IMUDriver(Node):
    def __init__(self):
        super().__init__('imu_driver')
        
        # quality of service
        qos_profile = QoSProfile(depth=10)
        
        # Subscriber for IMU broadcaster
        self.subscription_imu = self.create_subscription(
            Imu,                                              
            'robot/imu',
            self.imu_callback,
            10)
        self.subscription_imu

        # Publisher of Imu message with covariance
        self.imu_publisher = self.create_publisher(Imu, 'robot/imu_with_covariance', qos_profile)

    # Gazebo joint state subscriber callback
    # ROBOT LOCALIZATION EKF NODE uses ENU Frame! (x: East, y: North, z: Up)
    # Transformation needed
    def imu_callback(self, msg):

        # message header
        imu_msg = Imu()
        imu_msg.header.frame_id = 'base_link'
        imu_msg.header.stamp = self.get_clock().now().to_msg()

        # Orientation
        roll, pitch, yaw = quaternion_to_euler(msg.orientation)
        imu_msg.orientation = euler_to_quaternion(-pitch, roll, yaw)
        imu_msg.orientation_covariance = imu_orientation_covariance

        # Angular Velocity
        imu_msg.angular_velocity.x = -msg.angular_velocity.y
        imu_msg.angular_velocity.y = msg.angular_velocity.x
        imu_msg.angular_velocity.z = msg.angular_velocity.z
        imu_msg.angular_velocity_covariance = imu_ang_vel_covariance

        # Linear Acceleration
        imu_msg.linear_acceleration.x = -msg.linear_acceleration.y
        imu_msg.linear_acceleration.y = msg.linear_acceleration.x
        imu_msg.linear_acceleration.z = msg.angular_velocity.z
        imu_msg.linear_acceleration_covariance = imu_accel_covariance

        # publish IMU message
        self.imu_publisher.publish(imu_msg)


###### Functions ######

# convert quaternion to euler
def quaternion_to_euler(q):
    # roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch (y-axis rotation)
    sinp = math.sqrt(1 + 2 * (q.w * q.y - q.x * q.z))
    cosp = math.sqrt(1 - 2 * (q.w * q.y - q.x * q.z))
    pitch = 2 * math.atan2(sinp, cosp) - math.pi / 2

    # yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


# Convert Euler angles to Quaternions
def euler_to_quaternion(roll, pitch, yaw):
    qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
    qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
    qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)  


###### Main Function ######
def main(args=None):
    rclpy.init(args=args)
    node = IMUDriver()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    
    # Clean up
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
