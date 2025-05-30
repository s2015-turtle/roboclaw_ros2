#include "roboclaw_ros2/receiver.hpp"

Receiver::Receiver(std::shared_ptr<SerialPort> device)
: device_(std::move(device))
{
  if (!device_ || !device_->isOpen()) {
    handleError("Serial port is not open or invalid.");
  }
}
Receiver::~Receiver()
{
}

std::optional<std::vector<uint8_t>> Receiver::receive(size_t size)
{
  if (!device_ || !device_->isOpen()) {
    handleError("Serial port is not open or invalid.");
    return std::nullopt;
  }

  try {
    return device_->read(size);
  } catch (const std::runtime_error & e) {
    handleError(e.what());
    return std::nullopt;
  }
}
void Receiver::handleError(const std::string & message)
{
  RCLCPP_ERROR(rclcpp::get_logger("Receiver"), "%s", message.c_str());
}
