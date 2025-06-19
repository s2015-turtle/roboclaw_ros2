#include "roboclaw_ros2/roboclaw.hpp"
#include "roboclaw_ros2/serial_port.hpp"

#include <iostream>

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
  //std::cout << "response size: " << response->size() << std::endl;

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

  std::vector<uint8_t> command = {address_, (uint8_t)78};

  size_t bytesSent = serialport_.send(std::move(command));
  if (bytesSent == 0) {
    //std::cerr << "Failed to send command to Roboclaw." << std::endl;
    return {0, 0};
  }

  auto response = serialport_.receiveWithCRC(8);
  if (!response) {
    //std::cerr << "Failed to receive response from Roboclaw." << std::endl;
    return {0, 0};
  }
  //std::cout << "Response size: " << response->size() << std::endl;

  if (!response || response->size() != 8) {
    return {0, 0};
  }

  uint32_t encoder1 = (static_cast<uint32_t>((*response)[0]) << 24) |
    (static_cast<uint32_t>((*response)[1]) << 16) |
    (static_cast<uint32_t>((*response)[2]) << 8) |
    static_cast<uint32_t>((*response)[3]);

  uint32_t encoder2 = (static_cast<uint32_t>((*response)[4]) << 24) |
    (static_cast<uint32_t>((*response)[5]) << 16) |
    (static_cast<uint32_t>((*response)[6]) << 8) |
    static_cast<uint32_t>((*response)[7]);

  return {encoder1, encoder2};
}

std::optional<uint32_t> Roboclaw::read_encoder_count_M1(void)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!serialport_.isOpen()) {
    return std::nullopt;
  }

  std::vector<uint8_t> command = {address_, (uint8_t)16};
  uint16_t crc = SerialPort::crc16(command);

  size_t bytesSent = serialport_.send(std::move(command));
  if (bytesSent == 0) {
    //std::cerr << "Failed to send command to Roboclaw." << std::endl;
    return std::nullopt;
  }

  auto response = serialport_.receiveWithCRC(5, crc);
  if (!response) {
    // std::cerr << "Failed to receive response from Roboclaw." << std::endl;
    return std::nullopt;
  }
  //std::cout << "Response size: " << response->size() << std::endl;

  if (!response || response->size() != 5) {
    return std::nullopt;
  }

  uint32_t encoder1 = (static_cast<uint32_t>((*response)[0]) << 24) |
    (static_cast<uint32_t>((*response)[1]) << 16) |
    (static_cast<uint32_t>((*response)[2]) << 8) |
    static_cast<uint32_t>((*response)[3]);


  return encoder1;
}

std::optional<uint32_t> Roboclaw::read_encoder_count_M2(void)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!serialport_.isOpen()) {
    return std::nullopt;
  }

  std::vector<uint8_t> command = {address_, (uint8_t)17};
  uint16_t crc = SerialPort::crc16(command);

  size_t bytesSent = serialport_.send(std::move(command));
  if (bytesSent == 0) {
    //std::cerr << "Failed to send command to Roboclaw." << std::endl;
    return std::nullopt;
  }

  auto response = serialport_.receiveWithCRC(5, crc);
  if (!response) {
    // std::cerr << "Failed to receive response from Roboclaw." << std::endl;
    return std::nullopt;
  }
  //std::cout << "Response size: " << response->size() << std::endl;

  if (!response || response->size() != 5) {
    return std::nullopt;
  }

  uint32_t encoder2 = (static_cast<uint32_t>((*response)[0]) << 24) |
    (static_cast<uint32_t>((*response)[1]) << 16) |
    (static_cast<uint32_t>((*response)[2]) << 8) |
    static_cast<uint32_t>((*response)[3]);


  return encoder2;
}
