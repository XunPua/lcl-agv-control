# Arduino Due Middleware - ros_arduino_bridge

This code is used to turn an Arduino Due to a middleware for a Mecanum drive AGV. The Arduino can control all 4 motors with the appropriate PWM signals and evaluate their corresponding quadrature encoder feedbacks. It can also get readings from an IMU. It receives it commands and sends its status updates to a higher level system, in our case the jetson nano via serial interface.

This is a fork of the ros_arduino_bridge from Josh Newans, who based his code on this original code, with some changes, and removal of the ROS nodes (see [joshnewans' repo](https://github.com/joshnewans/serial_motor_demo) for an alternative to this method). Check out `README-orig.md` for the original README.

To view this code, make sure that Arduino IDE 2 and above is installed on your development PC, which the Adafruit BNO08x library installed.

## Functionality

The main functionality provided is to receive motor speed requests over a serial connection, and provide encoder feedback. It should also be able to provide IMU feedback.
The original code has provisions for other features - e.g. read/write of digital/analog pins, servo control, but these are removed as they are not used.

The main commands to know are: 

| Command | Comments |
| --- | --- |
| `e` | response with current encoder counts for each motor in `<FL> <FR> <BL> <BR>` |
| `m <SPD_FL> <SPD_FR> <SPD_BL> <SPD_BR>` | Set the closed-loop speed of each motor in *counts per loop* (Default loop rate is 30, so `(counts per sec)/30` |
| `o <PWM_FL> <PWM_FR> <PWM_BL> <PWM_BR>` | Set the raw PWM speed of each motor (-255 to 255) |
| `r` | Reset encoder values |
| `p <Kp> <Kd> <Ki> <Ko>` | Update the PID parameters |
| `v` | response with imu orientation (Quaternion) reading `<REAL> <I> <J> <K>` |
| `g` | response with imu gyroscope reading `<X> <Y> <Z> <0>` |
| `a` | response with imu linear acceleration reading `<X> <Y> <Z> <0>` |


## Tuning the Encoders

In this code architecture, the motor speed is control by counting the number of encoder ticks per PID loop. Therefore, it is important to find out the number of **encoder counts per revolution**, so that the value send to the arduino via [ros2_control.xacro](/ros2_ws/src/lcl_agv_pkg/description/ros2_control.xacro). The formula to find out the value is:
```
value = rad_per_sec / (2 * PI / counts_per_rev) / PID_loop_freq
```

To determine the encoder counts per revolution, first, connect one motor and its encoder to the arduino with the correct wiring. Then, connect to the Arduino via miniterm:
 ```
 pyserial-miniterm -e /dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Due_Prog._Port_14238313834351619231-if00 57600
 ```

 Then, put a piece of tape on the motor shaft and remember its starting position. Reset the encoder values using `r`. Now, by using raw pwm (`o 100 100 100 100` for example), rotate the the motor for a few rounds and remember the total number of rounds ran. Now, run `e` and record the encoder value received. Redo this a few times and record the average in the end. thats your counts per rev! From my experiments, the revs per count for the JGY370-EN motors are around `2746`.


## Files
| Files | Comment |
| --- | --- |
| ROSArduinoBridge.ino | Contains the main arduino setup and loop code. it handles the serial inputs and output, also the IMU interrupts |
| commands.h | list of commands and indices definitions |
| pid_controller.h | runs the motor controller PID |
| encoder_driver.h | defines the pins of the encoder feedback |
| encoder:driver.ino | contains interrupt service routines for encoder reading |
| motor_driver.h | defines motor driving pins for L298N (2 PWM per motor) |
| motor_driver.ino | sets the PWM to drive the motors |
