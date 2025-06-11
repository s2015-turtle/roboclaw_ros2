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
  

public:
    Roboclaw(const std::string & portName, unsigned int baudRate = 38400)
    : serialport_(portName, baudRate) {}
    
    ~Roboclaw();
    bool open();
    void close();
    bool drive_forward_M1(uint8_t address, int8_t speed);
};
}
#endif // ROBOCLAW_ROS2_ROBOCLAW_HPP