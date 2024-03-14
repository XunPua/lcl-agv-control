#ifndef MECANUMDRIVE_ARDUINO_IMU_HPP
#define MECANUMDRIVE_ARDUINO_IMU_HPP

#include <string>
#include <cmath>
#include <chrono>
#include <vector>

// Indices for IMU state
enum ImuStateIndex {
  ORIENTATION_W,
  ORIENTATION_X,
  ORIENTATION_Y,
  ORIENTATION_Z,
  ANG_VEL_X,
  ANG_VEL_Y,
  ANG_VEL_Z,
  LIN_ACCEL_X,
  LIN_ACCEL_Y,
  LIN_ACCEL_Z
};

// IMU class
class Imu
{
    public:

    std::vector<double> imu_states_;

    Imu() = default;
};


#endif // MECANUMDRIVE_ARDUINO_IMU_HPP
