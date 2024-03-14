# LCL_AGV_PKG
This package is the main package for the AGV. It contains the directories:

| Directory | Comment |
| --- | --- |
| [config](/ros2_ws/src/lcl_agv_pkg/config/) | configuration files (YAML) for various tools like nav2 and slam, and also RVIZ configuration files for different launch scenarios |
| [description](/ros2_ws/src/lcl_agv_pkg/description/) | XACRO files describing the robot and definitions for ROS2 Control |
| [launch](/ros2_ws/src/lcl_agv_pkg/launch/) | ROS2 Launch files |
| [lcl_agv_pkg](/ros2_ws/src/lcl_agv_pkg/lcl_agv_pkg/) | custom drivers |
| [maps](/ros2_ws/src/lcl_agv_pkg/maps/) | maps generated from SLAM |
| [meshes](/ros2_ws/src/lcl_agv_pkg/meshes/) | Meshes of the AGV |
| [worlds](/ros2_ws/src/lcl_agv_pkg/worlds/) | Description of the simulation environment (in this case a warehouse) |
| resources, test | automatically generated |

Files of the most important directories will be briefly explained below

**Table of Contents:**
- [LCL\_AGV\_PKG](#lcl_agv_pkg)
  - [Config](#config)
  - [Description](#description)
  - [Launch](#launch)
  - [lcl\_agv\_pkg](#lcl_agv_pkg-1)


## Config
| File | Comment |
| --- | --- |
| amcl_nav.rviz | Rviz configuration for running AMCL + Navigation |
| ekf.yaml | Configuration file for EKF node |
| gazebo_ros2_control_use_sim.yaml | Special yaml file to set the use_sim_time parameter when running simulation |
| hardware_deployment.rviz | Rviz configuration for hardware deployments |
| lcl_agv_pkg.rviz | Default Rviz configuration |
| mapper_params_online_async.yaml | Configuration file for SLAM |
| my_controllers.yaml | Yaml file specifying all ROS2 Control Controllers used in this project |
| nav2_params.yaml | Configuration file for all Navigation related nodes, including AMCL |
| twist_mux.yaml | Configuration file for Twist Mux |

## Description
| File | Comment |
| --- | --- |
| agv_body_original.xacro | Original urdf file generated from Solidworks |
| agv_body.xacro | Modified xacro file for AGV description |
| gazebo_control.xacro | Contains gazebo plugin for planar movement. Not used in this project |
| imu.xacro | specifies link, joint and plugin for IMU |
| inertial_macros.xacro | Macros for calculating ineartial values of sphere, boxes and cylinders |
| lidar.xacro | specifies link, joint and plugin for Lidar |
| robot.urdf.xacro | Main xacro file for AGV |
| ros2_control.xacro | defines Gazebo and Hardware plugins for ROS2 Control |

## Launch
| File | Comment |
| --- | --- |
| hardware_amcl_nav.launch.py | launches base system to run amcl and navigation on hardware. These two have to be launched separately! |
| hardware_slam.launch.py | launches SLAM on hardware |
| localization_launch.py | launches AMCL |
| navigation_launch.py | launches Navigation |
| rviz_spawn.launch.py | spawns robot in Rviz and prepares robot and joint state publishers |
| simulation_amcl_nav.launch.py | launches base system to run amcl and navigation on simulation. These two have to be launched separately! |
| simulation_slam.launch.py | launches SLAM in simulation |

## lcl_agv_pkg
| File | Comment |
| --- | --- |
| encoder_driver.py | subscribes to current wheel velocities and estimates AGV odometry, optionally also publishes Odometry transformation |
| imu_driver.py | subscribes to imu broadcaster and publishes processed imu message |
| mecanum_driver.py | subscribes to command twist and publishes wheel velocity controls |



