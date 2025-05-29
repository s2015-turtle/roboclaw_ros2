#include <linux/serial.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <memory>
#include <string>
#include <vector>
#include <termios.h>
#include "rclcpp/rclcpp.hpp"

#include "serial_port.hpp"

class Sender {
public:
  Sender(std::shared_ptr<SerialPort> device);
  ~Sender();
  bool Sender::send(std::vector<uint8_t> && data);

  bool drive_forward_M1(uint8_t address, uint8_t value);

private:
  std::shared_ptr<SerialPort> device_;

  uint16_t crc16(const std::vector<uint8_t> & data);
  void handleError(const std::string & message);

  bool compatibility_commands(uint8_t address, uint8_t command, uint8_t byte_value)
};

/*
   author: Yamato Kamei
   date: 2025-05-27
 */
