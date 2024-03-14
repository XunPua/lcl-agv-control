from ament_index_python.packages import get_package_share_directory
import os 

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():

    ###### Definitions ######
    # define package name
    package_name = 'lcl_agv_pkg'
    
    # define rviz configuration file
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'config', 'amcl_nav.rviz')
    
    # define twist mux configuration file
    twist_mux_file_path = os.path.join(get_package_share_directory(package_name), 'config', 'twist_mux.yaml')

    # define Gazebo simulation world file
    worlds_path = os.path.join(get_package_share_directory(package_name), 'worlds', 'warehouse.world')

    # define EKF configuration file
    ekf_params_file = os.path.join(get_package_share_directory(package_name), 'config', 'ekf.yaml')


    ###### Nodes and Launch files ######
    
    # Spawn robot in RVIZ
    rviz_spawn_launch = IncludeLaunchDescription(PythonLaunchDescriptionSource([os.path.join(
                                                get_package_share_directory(package_name),'launch','rviz_spawn.launch.py')]),
                                                launch_arguments={'use_sim_time': 'true',
                                                                  'use_ros2_control': 'true',
                                                                  'rvizconfig': rviz_config_file}.items()
                                                )

    # Launch Gazebo Classic
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={'world': str(worlds_path)}.items()
                                      )

    # Spawn AGV in simulation environment
    spawn_gazebo_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'agv'],
                        output='screen',
                        remappings=[('~/out', '/scan')]
                        )

    # User Interface
    gui_node = Node(
        package='ui_package',
        executable='ui_node',
        name='ui_node',
        output='screen'
        )
    
    # Mecanum driver
    mecanum_driver_node = Node(
        package='lcl_agv_pkg',
        executable='mecanum_driver',
        name='mecanum_driver',
        parameters=[{'use_sim_time': True}],
        output='screen'
        )
    
    # IMU driver
    imu_driver_node = Node(
        package='lcl_agv_pkg',
        executable='imu_driver',
        name='imu_driver',
        parameters=[{'use_sim_time': True}],
        output='screen',
        # remappings=[('îmu_plugin/out', 'robot/imu')]
    )
    
    # Encoder driver
    encoder_driver_node = Node(
        package='lcl_agv_pkg',
        executable='encoder_driver',
        name='encoder_driver',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # Twist Mux
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        arguments=['--ros-args', '--params-file', twist_mux_file_path],
        remappings=[('cmd_vel_out', 'command/cmd_vel')]
    )    
    
    # EKF
    ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params_file,
                        {'use_sim_time': True}],
           )
    
    # ROS2 Control Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # ROS2 Control forward velocity controller
    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller"],
    )


    ###### Launch Description ######

    ld = LaunchDescription()

    ld.add_action(rviz_spawn_launch)

    ld.add_action(gazebo)
    ld.add_action(spawn_gazebo_entity)

    ld.add_action(gui_node)
    ld.add_action(twist_mux_node)
    
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(forward_velocity_controller_spawner) 
    
    ld.add_action(mecanum_driver_node)
    ld.add_action(encoder_driver_node)
    ld.add_action(imu_driver_node)

    ld.add_action(ekf_node)
    
    return ld


if __name__ == '__main__':
    generate_launch_description()
