#ifndef ROBOCLAW_ROS2_ROBOCLAW_HPP
#define ROBOCLAW_ROS2_ROBOCLAW_HPP

#include "roboclaw_ros2/serial_port.hpp"
#include <mutex>
#include <tuple>

namespace roboclaw_ros2
{
class Roboclaw {
private:
  SerialPort serialport_;
  std::mutex mutex_;
  uint8_t address_;

public:
  Roboclaw(
    const std::string & portName, const unsigned int baudRate = 38400,
    const uint8_t address = 0x80);

  ~Roboclaw();
  bool open();
  void close();

  bool set_minimum_main_voltage(int8_t voltage);
  bool set_maximum_main_voltage(int8_t voltage);
  bool drive_M1(int8_t power);
  bool drive_M2(int8_t power);
  std::optional<uint32_t> read_encoder_count_M1(void);
  std::optional<uint32_t> read_encoder_count_M2(void);


  std::pair<uint32_t, uint32_t> read_encoder_counts(void);
};
}
#endif // ROBOCLAW_ROS2_ROBOCLAW_HPP
