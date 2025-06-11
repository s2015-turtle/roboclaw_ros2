#include "roboclaw_ros2/roboclaw.hpp"
#include "roboclaw_ros2/serial_port.hpp"

using namespace roboclaw_ros2;

Roboclaw::Roboclaw(const std::string & portName, unsigned int baudRate)
: serialport_(portName, baudRate)
{
    this->open();
}
Roboclaw::~Roboclaw()
{
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

bool Roboclaw::drive_forward_M1(uint8_t address, int8_t speed)
{
    std::lock_guard<std::mutex> lock(mutex_);
    
    if (!serialport_.isOpen()) {
        return false;
    }

    if(speed > 127) speed = 127;
    if(speed < 0) speed = 0; 
    std::vector<uint8_t> command = {address, 0x00, static_cast<uint8_t>(speed)};
    
    size_t bytesSent = serialport_.sendWithCRC(std::move(command));
    if (bytesSent == 0) {
        return false;
    }

    auto response = serialport_.receive(1);
    
    if (!response || response->empty()) {
        return false;
    }

    if ((*response)[0] != 0xFF) {
        return false; // Error response
    }

    return true;
}