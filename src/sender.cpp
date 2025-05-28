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
