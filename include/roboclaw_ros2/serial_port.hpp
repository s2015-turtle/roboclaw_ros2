#ifndef ROBOCLAW_ROS2_SERIAL_PORT_HPP
#define ROBOCLAW_ROS2_SERIAL_PORT_HPP
#include <string>
#include <cstddef>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <tuple>
#include <memory>
#include <vector>
#include <sys/ioctl.h>
#include <optional>

namespace roboclaw_ros2
{
class SerialPort {
private:
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  uint16_t crc16(const std::vector<uint8_t> & data);
  uint16_t crc16(const char * data, size_t size);

  void handleError(const std::string & message);

  std::string portName_;
  unsigned int baudRate_;
  int fd_;

public:
  SerialPort(const std::string & portName, unsigned int baudRate);
  ~SerialPort();

  bool open();
  void close();
  bool isOpen() const;

  std::optional<std::vector<uint8_t>> receive(size_t size);
  size_t send(const std::vector<uint8_t> && buffer);
  size_t send(const char * buffer, size_t size);
  size_t sendWithCRC(const std::vector<uint8_t> && buffer);
  size_t sendWithCRC(const char * buffer, size_t size);
  void setBaudRate(unsigned int baudRate);
  unsigned int getBaudRate() const;
};
}
#endif // ROBOCLAW_ROS2_SERIAL_PORT_HPP
