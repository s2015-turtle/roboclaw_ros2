#include <string>
#include <cstddef>
#include <stdexcept>



class SerialPort {
public:
    SerialPort(const std::string& portName, unsigned int baudRate);
    ~SerialPort();

    bool open();
    void close();
    bool isOpen() const;

    size_t read(char* buffer, size_t size);
    size_t write(const char* buffer, size_t size);
    void setBaudRate(unsigned int baudRate);
    unsigned int getBaudRate() const;
};