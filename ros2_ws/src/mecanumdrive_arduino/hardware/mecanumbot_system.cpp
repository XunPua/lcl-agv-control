// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mecanumdrive_arduino/mecanumbot_system.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace mecanumdrive_arduino
{
  /* ON INITIALIZE */
hardware_interface::CallbackReturn MecanumDriveArduinoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  // handle initialization fail
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize wheel names
  cfg_.front_left_wheel_name = info_.hardware_parameters["front_left_wheel_name"];
  cfg_.front_right_wheel_name = info_.hardware_parameters["front_right_wheel_name"];
  cfg_.back_left_wheel_name = info_.hardware_parameters["back_left_wheel_name"];
  cfg_.back_right_wheel_name = info_.hardware_parameters["back_right_wheel_name"];

  // initialize serial communication
  cfg_.loop_rate = std::stof(info_.hardware_parameters["loop_rate"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout_ms = std::stoi(info_.hardware_parameters["timeout_ms"]);

  // initialize wheel encoders and pid controller
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  if (info_.hardware_parameters.count("pid_p") > 0)
  {
    cfg_.pid_p = std::stoi(info_.hardware_parameters["pid_p"]);
    cfg_.pid_d = std::stoi(info_.hardware_parameters["pid_d"]);
    cfg_.pid_i = std::stoi(info_.hardware_parameters["pid_i"]);
    cfg_.pid_o = std::stoi(info_.hardware_parameters["pid_o"]);
  }
  else // handle if user didnt specify encoder
  {
    RCLCPP_INFO(rclcpp::get_logger("MecanumDriveArduinoHardware"), "PID values not supplied, using defaults.");
  }

  // if(info_.hardware_parameters["use_imu"] == "True" || info_.hardware_parameters["use_imu"] == "true")
  // {
  //   cfg_.use_imu = true;
  // }
  // imu_.setup("imu");

  // Initialize wheels
  wheel_fl_.setup(cfg_.front_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_fr_.setup(cfg_.front_right_wheel_name, cfg_.enc_counts_per_rev);
  wheel_bl_.setup(cfg_.back_left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_br_.setup(cfg_.back_right_wheel_name, cfg_.enc_counts_per_rev);

  // Initialize all joints by checking if state and command interfaces are correct
  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // MecanumBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumDriveArduinoHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Command expected to be veloctiy
    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumDriveArduinoHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Joint states expected to be position and velocity
    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumDriveArduinoHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // First joint state should be Position
    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumDriveArduinoHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Second joint states should be Velocity
    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("MecanumDriveArduinoHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Initialize IMU sensor states
  imu_.imu_states_.resize(info_.sensors[0].state_interfaces.size(), 0.0);

  // Initialization finished
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* EXPORT STATE INTERFACES */
std::vector<hardware_interface::StateInterface> MecanumDriveArduinoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // position and velocity of front left wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fl_.name, hardware_interface::HW_IF_POSITION, &wheel_fl_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.vel));

  // position and velocity of front right wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fr_.name, hardware_interface::HW_IF_POSITION, &wheel_fr_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.vel));

  // position and velocity of back left wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_bl_.name, hardware_interface::HW_IF_POSITION, &wheel_bl_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_bl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_bl_.vel));

  // position and velocity of back right wheel
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_br_.name, hardware_interface::HW_IF_POSITION, &wheel_br_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_br_.name, hardware_interface::HW_IF_VELOCITY, &wheel_br_.vel));

  // export imu state interfaces
  for (uint i = 0; i < info_.sensors[0].state_interfaces.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.sensors[0].name, info_.sensors[0].state_interfaces[i].name, &imu_.imu_states_[i]));
  }

  return state_interfaces;
}

/* EXPORT COMMAND INTERFACES */
std::vector<hardware_interface::CommandInterface> MecanumDriveArduinoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // front left wheel velocity command
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_fl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fl_.cmd));

  // front right wheel velocity command
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_fr_.name, hardware_interface::HW_IF_VELOCITY, &wheel_fr_.cmd));

  // back left wheel velocity command
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_bl_.name, hardware_interface::HW_IF_VELOCITY, &wheel_bl_.cmd));

  // back right wheel velocity command
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_br_.name, hardware_interface::HW_IF_VELOCITY, &wheel_br_.cmd));

  return command_interfaces;
}

/* ON CONFIGURE */
hardware_interface::CallbackReturn MecanumDriveArduinoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumDriveArduinoHardware"), "Configuring ...please wait...");
  
  // Reconnect Serial
  if (comms_.connected())
  {
    comms_.disconnect();
  }
  comms_.connect(cfg_.device, cfg_.baud_rate, cfg_.timeout_ms);

  RCLCPP_INFO(rclcpp::get_logger("MecanumDriveArduinoHardware"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ON CLEANUP */
hardware_interface::CallbackReturn MecanumDriveArduinoHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumDriveArduinoHardware"), "Cleaning up ...please wait...");
  
  // Disconnect serial
  if (comms_.connected())
  {
    comms_.disconnect();
  }

  RCLCPP_INFO(rclcpp::get_logger("MecanumDriveArduinoHardware"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ON ACTIVATE */
hardware_interface::CallbackReturn MecanumDriveArduinoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumDriveArduinoHardware"), "Activating ...please wait...");
  
  // throws error if serial not connected
  if (!comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // set pid values to motor controller
  if (cfg_.pid_p > 0)
  {
    comms_.set_pid_values(cfg_.pid_p,cfg_.pid_d,cfg_.pid_i,cfg_.pid_o);
  }

  RCLCPP_INFO(rclcpp::get_logger("MecanumDriveArduinoHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ON DEACTIVATE */
hardware_interface::CallbackReturn MecanumDriveArduinoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("MecanumDriveArduinoHardware"), "Deactivating ...please wait...");
  RCLCPP_INFO(rclcpp::get_logger("MecanumDriveArduinoHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

/* READ */
hardware_interface::return_type MecanumDriveArduinoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // throws error if serial is not connected
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // read encoder values
  comms_.read_encoder_values(wheel_fl_.enc, wheel_fr_.enc, wheel_bl_.enc, wheel_br_.enc);

  // calculate front left wheel speed
  wheel_fl_.pos = wheel_fl_.calc_enc_angle();
  wheel_fl_.vel = wheel_fl_.calc_speed();

  // calculate front right wheel speed
  wheel_fr_.pos = wheel_fr_.calc_enc_angle();
  wheel_fr_.vel = wheel_fr_.calc_speed();

  // calculate back left wheel speed
  wheel_bl_.pos = wheel_bl_.calc_enc_angle();
  wheel_bl_.vel = wheel_bl_.calc_speed();

  // calculate back right wheel speed
  wheel_br_.pos = wheel_br_.calc_enc_angle();
  wheel_br_.vel = wheel_br_.calc_speed();

  // read IMU
  comms_.read_imu(imu_.imu_states_);

  return hardware_interface::return_type::OK;
}

/* WRITE */
hardware_interface::return_type mecanumdrive_arduino ::MecanumDriveArduinoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // throws error if serial is not connected
  if (!comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  // calculate required encoder counts per loop for motor speed control
  int motor_fl_counts_per_loop = wheel_fl_.cmd / wheel_fl_.rads_per_count / cfg_.loop_rate;
  int motor_fr_counts_per_loop = wheel_fr_.cmd / wheel_fr_.rads_per_count / cfg_.loop_rate;
  int motor_bl_counts_per_loop = wheel_bl_.cmd / wheel_bl_.rads_per_count / cfg_.loop_rate;
  int motor_br_counts_per_loop = wheel_br_.cmd / wheel_br_.rads_per_count / cfg_.loop_rate;
  
  // send motor speeds to arduino
  comms_.set_motor_values(motor_fl_counts_per_loop, motor_fr_counts_per_loop, motor_bl_counts_per_loop, motor_br_counts_per_loop);
  
  return hardware_interface::return_type::OK;
}

}  // namespace mecanumdrive_arduino

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  mecanumdrive_arduino::MecanumDriveArduinoHardware, hardware_interface::SystemInterface)
