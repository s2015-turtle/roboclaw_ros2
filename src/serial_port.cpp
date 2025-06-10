#include "roboclaw_ros2/serial_port.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace roboclaw_ros2;
SerialPort::SerialPort(const std::string & portName, unsigned int baudRate)
: portName_(portName), baudRate_(baudRate), fd_(-1)
{
}

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open()
{
  fd_ = ::open(portName_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ == -1) {
    throw std::runtime_error("Failed to open serial port: " + portName_);
  }

  struct termios options;
  if (tcgetattr(fd_, &options) < 0) {
    close();
    throw std::runtime_error("Failed to get terminal attributes for: " + portName_);
  }

  cfsetispeed(&options, baudRate_);
  cfsetospeed(&options, baudRate_);

  options.c_cflag |= (CLOCAL | CREAD);       // Ignore modem control lines
  options.c_cflag &= ~PARENB;       // No parity
  options.c_cflag &= ~CSTOPB;       // 1 stop bit
  options.c_cflag &= ~CSIZE;       // Clear size bits
  options.c_cflag |= CS8;       // 8 data bits
  options.c_iflag &= ~(IXON | IXOFF | IXANY);
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

  if (tcsetattr(fd_, TCSANOW, &options) < 0) {
    close();
    throw std::runtime_error("Failed to set terminal attributes for: " + portName_);
  }

  return true;
}

void SerialPort::close()
{
  if (fd_ != -1) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::isOpen() const
{
  return fd_ != -1;
}

std::optional<std::vector<uint8_t>> SerialPort::receive(size_t size)
{
  if (!this > isOpen()) {
    handleError("Serial port is not open or invalid.");
    return std::nullopt;
  }

  int available_size = 0;
  ioctl(fd_, FIONREAD, &available_size);

  RCLCPP_INFO(rclcpp::get_logger("SerialPort"), "Available bytes: %d", available_size);

  std::vector<uint8_t> buffer(size);
  ssize_t bytesRead = ::read(fd_, buffer.data(), size);
  if (bytesRead < 0) {
    handleError("Failed to read from serial port: " + portName_);
    perror("read error");
    return std::nullopt;
  }
  if (bytesRead == 0) {
    RCLCPP_WARN(rclcpp::get_logger("SerialPort"), "No data received from serial port: %s",
      portName_.c_str());
    return std::nullopt;
  }

  buffer.resize(static_cast<size_t>(bytesRead));

  return buffer;
}

size_t SerialPort::send(const std::vector<uint8_t> && data)
{
  return this->send(reinterpret_cast<const char *>(data.data()), data.size());
}

size_t SerialPort::sendWithCRC(const std::vector<uint8_t> && data)
{
  return this->sendWithCRC(reinterpret_cast<const char *>(data.data()), data.size());
}

size_t SerialPort::send(const char * data, size_t size)
{
  if (fd_ == -1) {
    handleError("Serial port is not open.");
  }

  int bytesWritten = ::write(fd_, data, size);

  if (bytesWritten <= 0) {
    handleError("Failed to write to serial port: " + portName_);
  }


  for(size_t i = 0; i < static_cast<size_t>(bytesWritten); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("SerialPort"), "Wrote byte: %02X",
      static_cast<unsigned char>(data[i]));
  }

  return static_cast<size_t>(bytesWritten);
}

size_t SerialPort::sendWithCRC(const char * data, size_t size)
{
  if (fd_ == -1) {
    handleError("Serial port is not open.");
  }

  uint16_t crc = crc16(data, size);
  const char crcBytes[2] = {
    static_cast<char>(crc >> 8), 
    static_cast<char>(crc & 0xFF)
  };

  int bytesWritten = ::write(fd_, data, size);
  int crcWritten = ::write(fd_, crcBytes, 2);

  if (bytesWritten <= 0 || crcWritten <= 0) {
    handleError("Failed to write to serial port: " + portName_);
  }


  for(size_t i = 0; i < static_cast<size_t>(bytesWritten); ++i) {
    RCLCPP_INFO(rclcpp::get_logger("SerialPort"), "Wrote byte: %02X",
      static_cast<unsigned char>(data[i]));
  }

  return static_cast<size_t>(bytesWritten);

}


void SerialPort::setBaudRate(unsigned int baudRate)
{
  if (fd_ == -1) {
    handleError("Serial port is not open.");
  }

  struct termios options;
  if (tcgetattr(fd_, &options) < 0) {
    handleError("Failed to get terminal attributes for: " + portName_);
  }

  cfsetispeed(&options, baudRate);
  cfsetospeed(&options, baudRate);

  if (tcsetattr(fd_, TCSANOW, &options) < 0) {
    handleError("Failed to set terminal attributes for: " + portName_);
  }

  baudRate_ = baudRate;
}

unsigned int SerialPort::getBaudRate() const
{
  return baudRate_;
}

uint16_t SerialPort::crc16(const std::vector<uint8_t> & data)
{
  return crc16(reinterpret_cast<const char *>(data.data()), data.size());
}

uint16_t SerialPort::crc16(const char * data, size_t size)
{
  uint16_t crc = 0; // Initial value
  for (int byte = 0; byte < size; byte++) {
    crc = crc ^ ((unsigned int)data[byte] << 8);
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

void SerialPort::handleError(const std::string & message)
{
  RCLCPP_ERROR(rclcpp::get_logger("SerialPort"), "%s", message.c_str());
}
