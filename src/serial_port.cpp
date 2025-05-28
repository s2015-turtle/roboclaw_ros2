#include "serial_port.hpp"

SerialPort::SerialPort(const std::string& portName, unsigned int baudRate)
    : portName_(portName), baudRate_(baudRate), fd_(-1) {
}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open() {
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

    options.c_cflag |= (CLOCAL | CREAD); // Ignore modem control lines
    options.c_cflag &= ~PARENB; // No parity
    options.c_cflag &= ~CSTOPB; // 1 stop bit
    options.c_cflag &= ~CSIZE; // Clear size bits
    options.c_cflag |= CS8; // 8 data bits

    if (tcsetattr(fd_, TCSANOW, &options) < 0) {
        close();
        throw std::runtime_error("Failed to set terminal attributes for: " + portName_);
    }

    return true;
}

void SerialPort::close() {
    if (fd_ != -1) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialPort::isOpen() const {
    return fd_ != -1;
}

std::unique_ptr<std::vector<uint8_t>> SerialPort::read(size_t size) {
    int available_size = 0;
    auto data = std::make_unique<std::vector<uint8_t>>();

    if (fd_ == -1) {
        throw std::runtime_error("Serial port is not open.");
    }
    ioctl(fd_, FIONREAD, &available_size);

    if (available_size < static_cast<int>(size)) {
        size = static_cast<size_t>(available_size);
    }
    if (size == 0) {
        return data; 
    }

    data->resize(size);

    ssize_t bytesRead = ::read(fd_, data->data(), size);
    if (bytesRead < 0) {
        throw std::runtime_error("Failed to read from serial port: " + portName_);
    }
    if (static_cast<size_t>(bytesRead) < size) {
        data->resize(bytesRead);
    }
    
    return data;
}