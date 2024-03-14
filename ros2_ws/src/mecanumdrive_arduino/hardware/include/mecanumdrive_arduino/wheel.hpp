#ifndef MECANUMDRIVE_ARDUINO_WHEEL_HPP
#define MECANUMDRIVE_ARDUINO_WHEEL_HPP

#include <string>
#include <cmath>
#include <chrono>

/* Wheel Class to describe each wheel/motor/encoder of the AGV */
class Wheel
{
    public:

    // variables
    std::string name = "";
    int enc = 0;
    double cmd = 0;
    double pos = 0;
    double vel = 0;
    double rads_per_count = 0;
    double prev_pos = 0;
    std::chrono::high_resolution_clock::time_point prev_time;

    Wheel() = default;

    // constructor
    Wheel(const std::string &wheel_name, int counts_per_rev)
    {
      setup(wheel_name, counts_per_rev);
      prev_time = std::chrono::high_resolution_clock::now();
    }

    // set wheel configurations
    void setup(const std::string &wheel_name, int counts_per_rev)
    {
      name = wheel_name;
      rads_per_count = (2*M_PI)/counts_per_rev;
    }

    // calculate current encoder angle
    double calc_enc_angle()
    {
      return enc * rads_per_count;
    }

    // calculate motor speed
    double calc_speed()
    {
      double speed_rad_sec = 0;

      // get time difference
      auto current_time = std::chrono::high_resolution_clock::now();
      std::chrono::duration<double> elapsed_duration = current_time - prev_time;
      double time_diff = elapsed_duration.count();

      // calculate speed
      speed_rad_sec = (pos - prev_pos) / time_diff;

      // save current encoder position and read time
      prev_pos = pos;
      prev_time = current_time;

      // return speed
      return speed_rad_sec; 
    }

};


#endif // MECANUMDRIVE_ARDUINO_WHEEL_HPP
