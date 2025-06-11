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

