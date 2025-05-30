#include "sender.hpp"

Sender::Sender(std::shared_ptr<SerialPort> device)
: device_(std::move(device))
{
  if (!device_ || !device_->isOpen()) {
    handleError("Serial port is not open or invalid.");
  }
}


Sender::~Sender()
{
}

bool Sender::send(std::vector<uint8_t> && data)
{
  if (!device_ || !device_->isOpen()) {
    handleError("Serial port is not open or invalid.");
    return false;
  }

  uint16_t crc = crc16(data);
  data.push_back(static_cast<uint8_t>(crc >> 8));
  data.push_back(static_cast<uint8_t>(crc & 0xFF));

  try {
    device_->write(std::move(data));
  } catch (const std::runtime_error & e) {
    handleError(e.what());
    return false;
  }

  return true;
}

uint16_t Sender::crc16(const std::vector<uint8_t> & data)
{
  uint16_t crc = 0; // Initial value
  for (int byte = 0; byte < data.size(); byte++) {
    uint16_t crc = crc ^ ((unsigned int)data[byte] << 8);
    for (unsigned char bit = 0; bit < 8; bit++) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc = crc << 1;
      }
    }
  }
  return crc & 0xFFFF;
}

void Sender::handleError(const std::string & message)
{
  RCLCPP_ERROR(rclcpp::get_logger("Sender"), "%s", message.c_str());
}
