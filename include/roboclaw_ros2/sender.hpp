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
  bool send(std::vector<uint8_t> && data);

private:
  std::shared_ptr<SerialPort> device_;
  uint16_t crc16(const std::vector<uint8_t> & data);
  void handleError(const std::string & message);
};

/*
   author: Yamato Kamei
   date: 2025-05-27
 */
