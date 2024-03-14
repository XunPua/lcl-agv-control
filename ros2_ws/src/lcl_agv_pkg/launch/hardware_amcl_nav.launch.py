from ament_index_python.packages import get_package_share_directory
import os 
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():

    ###### Definitions #######

    # define package name
    package_name = 'lcl_agv_pkg'

    # define lidar serial by id path
    lidar_device_path = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_220dbf3dc414304aa791ae01d0dce108-if00-port0'

    # define arduino due serial by id path
    arduino_device_path = '/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Due_Prog._Port_14238313834351619231-if00'

    # define twist mux configuration file
    twist_mux_file_path = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    # define ROS2 Control controllers configuration file
    controller_params_file = os.path.join(get_package_share_directory(package_name),
                                          'config','my_controllers.yaml')
    
    # define Rviz configuration file
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_nav.rviz')

    # define EKF configuration file
    ekf_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')
    

    ###### Nodes and Launch Files ######

    # Spawn AGV in Rviz
    rviz_spawn_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
                                                get_package_share_directory(package_name),'launch','rviz_spawn.launch.py')]),
                                                launch_arguments={'use_sim_time': 'false',
                                                                  'use_ros2_control': 'true',
                                                                  'rvizconfig': rviz_config_file}.items()
                                                )
    
    # User Interface 
    gui_node = Node(
        package='ui_package',
        executable='ui_node',
        name='ui_node',
        output='screen',
        # remappings=[('gui/cmd_vel', 'command/cmd_vel')]
        )
    
    # Twist Mux
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        arguments=['--ros-args', '--params-file', twist_mux_file_path],
        remappings=[('cmd_vel_out', 'command/cmd_vel')]
    ) 
    
    # Get robot description urdf published from robot state publisher
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])
    
    # controller manager for ros2 control
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)},
                    controller_params_file],
        remappings=[('/controller_manager/robot_description', '/robot_description'),
                    ('/imu_sensor_broadcaster/imu', '/robot/imu')],
        output="screen"
    )

    # delay start of controller manager to ensure robot spawn is already completed
    delay_controller_manager = TimerAction(period=10.0, actions=[controller_manager])
    
    # joint state broadcaster for ros2 control
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # only start joint state broadcaster after controller manager has started
    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner]
        )
    )

    # IMU broadcaster for ros2 control
    imu_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_sensor_broadcaster"]
    )

    # only start imu broadcaster after controller manager has started
    delayed_imu_sensor_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[imu_sensor_broadcaster_spawner],
        )
    )

    # individual wheel velocity controller for ros2 control
    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller"],
    )

    # only start individual wheel velocity controller after controller manager has started
    delayed_forward_velocity_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[forward_velocity_controller_spawner],
        )
    )

    # Mecanum wheel driver for forward and inverse kinematics
    mecanum_driver_node = Node(
        package='lcl_agv_pkg',
        executable='mecanum_driver',
        name='mecanum_driver',
        output='screen'
        )
    
    # encoder driver for converting encoder values to odometry
    encoder_driver_node = Node(
        package='lcl_agv_pkg',
        executable='encoder_driver',
        name='encoder_driver',
        output='screen'
    )

    # imu driver process imu data if needed
    imu_driver_node = Node(
        package='lcl_agv_pkg',
        executable='imu_driver',
        name='imu_driver',
        output='screen'
        # remappings=[('îmu_plugin/out', 'robot/imu')]
    )

    # EKF node for odometry data fusion
    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config_file,
                        {'use_sim_time': False}],
           )
    
    # LIDAR
    lidar_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
                                                get_package_share_directory('sllidar_ros2'),'launch','sllidar_a2m8_launch.py')]),
                                                launch_arguments={'serial_port': lidar_device_path,
                                                                  'frame_id': 'lidar_link',
                                                                  'inverted': 'false'}.items()
                                                )


    ###### Launch Descriptions ######

    ld = LaunchDescription()

    ld.add_action(rviz_spawn_launch)

    ld.add_action(lidar_launch)

    ld.add_action(delay_controller_manager)
    ld.add_action(delayed_joint_state_broadcaster_spawner)
    ld.add_action(delayed_forward_velocity_controller_spawner)
    ld.add_action(delayed_imu_sensor_broadcaster_spawner)

    ld.add_action(gui_node)
    ld.add_action(mecanum_driver_node)
    ld.add_action(encoder_driver_node)
    ld.add_action(imu_driver_node)
    ld.add_action(twist_mux_node)
    # ld.add_action(ekf_node)
    
    return ld


if __name__ == '__main__':
    generate_launch_description()
