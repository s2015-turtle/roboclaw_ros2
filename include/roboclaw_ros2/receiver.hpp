#include <linux/serial.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>
#include <memory>
#include <string>
#include <vector>
#include <termios.h>
#include <optional>
#include "rclcpp/rclcpp.hpp"

#include "serial_port.hpp"

class Receiver {
public:
  Receiver(std::shared_ptr<SerialPort> device);
  ~Receiver();

  std::optional<std::vector<uint8_t>> receive(size_t size);

private:
  std::shared_ptr<SerialPort> device_;

  void handleError(const std::string & message);

};
