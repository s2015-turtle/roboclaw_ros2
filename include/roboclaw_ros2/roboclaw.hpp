#ifndef ROBOCLAW_ROS2_ROBOCLAW_HPP
#define ROBOCLAW_ROS2_ROBOCLAW_HPP

#include "roboclaw_ros2/serial_port.hpp"
#include <mutex>

namespace roboclaw_ros2
{
class Roboclaw {
private:
  SerialPort serialport_;
  std::mutex mutex_;
  uint8_t address_;

public:
  Roboclaw(const std::string & portName, const unsigned int baudRate = 38400, const uint8_t address = 0x80);

  ~Roboclaw();
  bool open();
  void close();

  bool drive_forward_M1(int8_t speed);
  bool drive_backward_M1(int8_t speed);
  bool set_minimum_main_voltage(int8_t voltage);
  bool set_maximum_main_voltage(int8_t voltage);
  bool drive_forward_M2(int8_t speed);
  bool drive_backward_M2(int8_t speed);
  bool drive_M1(int8_t speed);
  bool drive_M2(int8_t speed);
};
}
#endif // ROBOCLAW_ROS2_ROBOCLAW_HPP
