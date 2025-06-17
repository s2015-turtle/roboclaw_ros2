#include "roboclaw_ros2/roboclaw.hpp"
#include "roboclaw_ros2/serial_port.hpp"

using namespace roboclaw_ros2;

Roboclaw::Roboclaw(const std::string & portName, unsigned int baudRate, uint8_t address)
: serialport_(portName, baudRate), address_(address)
{
  this->open();
}
Roboclaw::~Roboclaw()
{
  this->drive_M1(64);
  serialport_.close();
}

bool Roboclaw::open()
{
  return serialport_.open();
}
void Roboclaw::close()
{
  serialport_.close();
}

bool Roboclaw::set_minimum_main_voltage(int8_t voltage)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!serialport_.isOpen()) {
    return false;
  }

  uint8_t value = (voltage - 6) * 5;
  if(value < 0 || value > 140) {
    return false;
  }

  std::vector<uint8_t> command = {address_, 0x02, value};

  size_t bytesSent = serialport_.sendWithCRC(std::move(command));
  if (bytesSent == 0) {
    return false;
  }

  auto response = serialport_.receive(1);

  if (!response || response->empty()) {
    return false;
  }

  if ((*response)[0] != 0xFF) {
    return false;
  }

  return true;
}

bool Roboclaw::set_maximum_main_voltage(int8_t voltage)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!serialport_.isOpen()) {
    return false;
  }

  uint8_t value = voltage * 5.12;
  if(value < 30 || value > 175) {
    return false;
  }

  std::vector<uint8_t> command = {address_, 0x03, value};

  size_t bytesSent = serialport_.sendWithCRC(std::move(command));
  if (bytesSent == 0) {
    return false;
  }

  auto response = serialport_.receive(1);

  if (!response || response->empty()) {
    return false;
  }

  if ((*response)[0] != 0xFF) {
    return false;
  }

  return true;
}


bool Roboclaw::drive_M1(int8_t power)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!serialport_.isOpen()) {
    return false;
  }

  if(power > 127 || power < 0) {return false;}

  std::vector<uint8_t> command = {address_, 0x06, static_cast<uint8_t>(power)};

  size_t bytesSent = serialport_.sendWithCRC(std::move(command));
  if (bytesSent == 0) {
    return false;
  }

  auto response = serialport_.receive(1);

  if (!response || response->empty()) {
    return false;
  }

  if ((*response)[0] != 0xFF) {
    return false;
  }

  return true;
}

bool Roboclaw::drive_M2(int8_t power)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!serialport_.isOpen()) {
    return false;
  }

  if(power > 127 || power < 0) {return false;}

  std::vector<uint8_t> command = {address_, 0x07, static_cast<uint8_t>(power)};

  size_t bytesSent = serialport_.sendWithCRC(std::move(command));
  if (bytesSent == 0) {
    return false;
  }

  auto response = serialport_.receive(1);

  if (!response || response->empty()) {
    return false;
  }

  if ((*response)[0] != 0xFF) {
    return false;
  }

  return true;
}

std::pair<uint32_t, uint32_t> Roboclaw::read_encoder_counts(void)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!serialport_.isOpen()) {
    return {0, 0};
  }

  std::vector<uint8_t> command = {address_, 78};

  size_t bytesSent = serialport_.sendWithCRC(std::move(command));
  if (bytesSent == 0) {
    return {0, 0};
  }

  auto response = serialport_.receiveWithCRC(10);

  if (!response || response->size() != 8) {
    return {0, 0};
  }

  uint32_t encoder1 = (static_cast<uint32_t>((*response)[1]) << 24) |
                      (static_cast<uint32_t>((*response)[2]) << 16) |
                      (static_cast<uint32_t>((*response)[3]) << 8) |
                      static_cast<uint32_t>((*response)[4]);

  uint32_t encoder2 = (static_cast<uint32_t>((*response)[5]) << 24) |
                      (static_cast<uint32_t>((*response)[6]) << 16) |
                      (static_cast<uint32_t>((*response)[7]) << 8) |
                      static_cast<uint32_t>((*response)[8]);

  return {encoder1, encoder2};
}