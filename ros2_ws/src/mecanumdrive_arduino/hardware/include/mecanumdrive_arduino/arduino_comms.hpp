#ifndef MECANUMDRIVE_ARDUINO_ARDUINO_COMMS_HPP
#define MECANUMDRIVE_ARDUINO_ARDUINO_COMMS_HPP

#include <sstream>
#include <libserial/SerialPort.h>
#include <iostream>
#include "mecanumdrive_arduino/imu.hpp"

LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  // Just handle some common baud rates
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 57600" << std::endl;
      return LibSerial::BaudRate::BAUD_57600;
  }
}

/* Class for Serial Communication with Arduino */
class ArduinoComms
{

public:

  ArduinoComms() = default;

  /* Open serial connection */
  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  /* Disconnect */
  void disconnect()
  {
    serial_conn_.Close();
  }

  /* Check if serial is connected */
  bool connected() const
  {
    return serial_conn_.IsOpen();
  }

  /* Send message to Arduino */
  std::string send_msg(const std::string &msg_to_send, bool print_output = false)
  {
    // Clear buffer
    serial_conn_.FlushIOBuffers(); 

    // Send message
    serial_conn_.Write(msg_to_send);

    // Handle response
    std::string response = "";
    try
    {
      // Responses end with \r\n so we will read up to (and including) the \n.
      serial_conn_.ReadLine(response, '\n', timeout_ms_);
    }
    catch (const LibSerial::ReadTimeout&)
    {
        // Handle timeout for reading reply from arduino
        std::cerr << "The ReadByte() call has timed out." << std::endl ;
    }

    // If print output is set to true
    if (print_output)
    {
      std::cout << "Sent: " << msg_to_send << " Recv: " << response << std::endl;
    }

    // return arduino reply
    return response;
  }

  /* Send empty message to arduino */
  void send_empty_msg()
  {
    std::string response = send_msg("\r");
  }

  /* Read motor encoder values */
  void read_encoder_values(int &val_1, int &val_2, int &val_3, int &val_4)
  {
    std::string response = send_msg("e\r");

    std::istringstream iss(response); // Create a string stream to process the response

    // Read the four integer values from the response
    iss >> val_1 >> val_2 >> val_3 >> val_4;

    // std::string delimiter = " ";
    // size_t del_pos = response.find(delimiter);
    // std::string token_1 = response.substr(0, del_pos);
    // std::string token_2 = response.substr(del_pos + delimiter.length());

    // val_1 = std::atoi(token_1.c_str());
    // val_2 = std::atoi(token_2.c_str());
  }

  /* Set motor speeds */
  void set_motor_values(int val_1, int val_2, int val_3, int val_4)
  {
    std::stringstream ss;
    ss << "m " << val_1 << " " << val_2 << " " << val_3 << " " << val_4 << "\r";
    send_msg(ss.str());
  }

  /* Set motor control PID values */
  void set_pid_values(int k_p, int k_d, int k_i, int k_o)
  {
    std::stringstream ss;
    ss << "u " << k_p << ":" << k_d << ":" << k_i << ":" << k_o << "\r";
    send_msg(ss.str());
  }

  /* Read imu values */
  void read_imu(std::vector<double>& imu_states_)
  {
    // read orientation
    std::string response = send_msg("v\r");
    std::istringstream iss_orientation(response);
    iss_orientation >> imu_states_[ORIENTATION_W] >> imu_states_[ORIENTATION_X] >> imu_states_[ORIENTATION_Y] >> imu_states_[ORIENTATION_Z];

    // read gyroscope
    response = send_msg("g\r");
    int dummy;
    std::istringstream iss_gyro(response);
    iss_gyro >> imu_states_[ANG_VEL_X] >> imu_states_[ANG_VEL_Y] >> imu_states_[ANG_VEL_Z] >> dummy;

    // read linear acceleration
    response = send_msg("a\r");
    std::istringstream iss_lin_accel(response);
    iss_lin_accel >> imu_states_[LIN_ACCEL_X] >> imu_states_[LIN_ACCEL_Y] >> imu_states_[LIN_ACCEL_Z] >> dummy;
  }
  

private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
};

#endif // MECANUMDRIVE_ARDUINO_ARDUINO_COMMS_HPP