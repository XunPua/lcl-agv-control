from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
import os


def generate_launch_description():

    ###### Definitions ######

    # define package name
    package_name = 'lcl_agv_pkg'

    # define robot description file
    xacro_file = os.path.join(get_package_share_path(package_name), 'description', 'robot.urdf.xacro')

    # define rviz configuration file
    default_rviz_config_path = os.path.join(get_package_share_path(package_name), 'config', 'lcl_agv_pkg.rviz')
    
    # define use sim time as launch argument
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
                                    description='Use simulation (Gazebo) clock if true')
    
    # define use ros2 control as launch argument
    use_ros2_control = LaunchConfiguration('use_ros2_control')
    use_ros2_control_arg =  DeclareLaunchArgument('use_ros2_control', default_value='true',
                                    description='Use ros2_control if true')
    
    # define rviz configuration file as launch argument
    rviz_config = LaunchConfiguration('rvizconfig')
    rviz_config_arg = DeclareLaunchArgument(name='rvizconfig', default_value=str(default_rviz_config_path),
                                    description='Absolute path to rviz config file')
    
    # define robot description configuration for robot state publisher
    robot_description_config = Command(['xacro ', xacro_file, ' use_ros2_control:=', use_ros2_control, ' sim_mode:=', use_sim_time])
    
    ###### Nodes ######
    
    # robot state publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': ParameterValue(robot_description_config, value_type=str),
                     'use_ros2_control': use_ros2_control,
                     'use_sim_time': use_sim_time}]
    )

    # joint state publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 
                     'rate': 30, 
                     'source_list': ['robot/joint_states']}]
    )

    # start rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )


    ###### Launch Description ######

    ld = LaunchDescription()

    ld.add_action(rviz_config_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_ros2_control_arg)
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)
    
    return ld


if __name__ == '__main__':
    generate_launch_description()
