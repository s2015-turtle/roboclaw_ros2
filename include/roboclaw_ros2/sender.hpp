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
  bool send(const std::vector<uint8_t> & data, size_t length);

private:
  std::shared_ptr<SerialPort> device_;

  bool setBaudRate(int baudrate);
  bool configurePort();
  void handleError(const std::string & message);

};

/*
   author: Yamato Kamei
   date: 2025-05-27
 */
