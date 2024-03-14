# Mecanumdrive Arduino

This node is designed to provide a ros2_control hardware interface for an Arduino Due running on a Mecanum driven AGV. It is designed to receive commands from a `forward_command_controller` and sends state updates to a `joint_state_broadcaster` and a `imu_sensor_broadcaster`, all from `ros2_control`.

It is expected to communicate via serial and to have four motors, each with velocity control and position/velocity feedback. The motors are each equipped with a quadrature encoder.


| Files | Comment |
| --- | --- |
| hardware/include/.../arduino_comms.hpp | handles serial communication with Arduino and provides basic functions for interaction |
| hardware/include/.../imu.hpp | defines IMU class and related variables |
| hardware/include/.../mecanumbot_system.hpp | ROS2 Control hardware interface main header file |
| hardware/include/.../visibility_control.h | Automatically generated |
| hardware/include/.../wheel.hpp | defines each wheel/motor/encoder and provides functions to calculate motor speed from encoder feedback |
| hardware/mecanumbot_system.cpp | Contains ROS2 Control hardware interface state machine source code |


