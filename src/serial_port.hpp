#include <string>
#include <cstddef>
#include <stdexcept>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <tuple>
#include <memory>
#include <vector>
#include <sys/ioctl.h>

class SerialPort {
private:
    SerialPort(const SerialPort&) = delete; 
    SerialPort& operator=(const SerialPort&) = delete;

    std::string portName_;
    unsigned int baudRate_;
    int fd_; 

public:
    SerialPort(const std::string& portName, unsigned int baudRate);
    ~SerialPort();

    bool open();
    void close();
    bool isOpen() const;

    std::unique_ptr<std::vector<uint8_t>> read(size_t size);
    size_t write(const char* buffer, size_t size);
    void setBaudRate(unsigned int baudRate);
    unsigned int getBaudRate() const;
};