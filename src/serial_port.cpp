#include "roboclaw_ros2/serial_port.hpp"

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
  fd_ = ::open(portName_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
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

std::optional<std::vector<uint8_t>> SerialPort::read(size_t size)
{
  auto data = std::vector<uint8_t>();

  if (fd_ == -1) {
    throw std::runtime_error("Serial port is not open.");
  }
  if (size == 0) {
    return std::nullopt;
  }

  data.resize(size);

  ssize_t bytesRead = ::read(fd_, data.data(), size);
  if (bytesRead < 0) {
    throw std::runtime_error("Failed to read from serial port: " + portName_);
  }

  return data;
}

size_t SerialPort::write(const std::vector<uint8_t> && buffer)
{
  return this->write(reinterpret_cast<const char *>(buffer.data()), buffer.size());
}

size_t SerialPort::write(const char * buffer, size_t size)
{
  if (fd_ == -1) {
    throw std::runtime_error("Serial port is not open.");
  }

  ssize_t bytesWritten = ::write(fd_, buffer, size);
  if (bytesWritten < 0) {
    throw std::runtime_error("Failed to write to serial port: " + portName_);
  }

  return static_cast<size_t>(bytesWritten);
}

void SerialPort::setBaudRate(unsigned int baudRate)
{
  if (fd_ == -1) {
    throw std::runtime_error("Serial port is not open.");
  }

  struct termios options;
  if (tcgetattr(fd_, &options) < 0) {
    throw std::runtime_error("Failed to get terminal attributes for: " + portName_);
  }

  cfsetispeed(&options, baudRate);
  cfsetospeed(&options, baudRate);

  if (tcsetattr(fd_, TCSANOW, &options) < 0) {
    throw std::runtime_error("Failed to set terminal attributes for: " + portName_);
  }

  baudRate_ = baudRate;
}

unsigned int SerialPort::getBaudRate() const
{
  return baudRate_;
}
