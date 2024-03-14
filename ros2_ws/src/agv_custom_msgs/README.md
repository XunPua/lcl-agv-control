# AGV_CUSTOM_MSGS
This package defines all custom ROS2 interfaces used for the AGV

## AGVSingleWheelCtrl
This interface is used to send single wheel control values to the mecanum driver. It contains:

``````
uint8 iden
float64 wheel_speed_rad_sec
``````

with iden being
| iden  | wheel             |
| ---   | ---               |
| 0     | Front Left (FL)   |
| 1     | Front Right (FR)  |
| 2     | Back Left (BL)    |
| 3     | Back Right (BR)   |

and *wheel_speed_rad_sec* being the desired wheel speed in *radiens per second*.