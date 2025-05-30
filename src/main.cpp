#include "rclcpp/rclcpp.hpp"
#include "roboclaw_ros2/serial_port.hpp"
#include "roboclaw_ros2/sender.hpp"
#include "roboclaw_ros2/receiver.hpp"
#include <vector>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto SerialPort = std::make_shared<roboclaw_ros2::SerialPort>("/dev/ttyACM0", 38400);
  if (!SerialPort->open()) {
    RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to open serial port.");
    rclcpp::shutdown();
    return 1;
  }
  roboclaw_ros2::Sender sender(SerialPort);
  roboclaw_ros2::Receiver receiver(SerialPort);


  while(rclcpp::ok())
  {
    std::vector<uint8_t> dataToSend = {0x80, 0x00, 60}; 
    std::vector<uint8_t> dataToSend2 = {0x80, 0x0, 60}; 
    if (!sender.send(std::move(dataToSend))) {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to send data.");
      break;
    }
    RCLCPP_INFO(rclcpp::get_logger("main"), "Data sent successfully.");

    auto receivedData = receiver.receive(1);
    if (receivedData) {
      RCLCPP_INFO(rclcpp::get_logger("main"), "Received data: %d", (*receivedData)[0]);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to receive data.");
    }
    rclcpp::sleep_for(500ms);
    // if (!sender.send(std::move(dataToSend2))) {
    //   RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to send data.");
    //   break;
    // }
    // RCLCPP_INFO(rclcpp::get_logger("main"), "Data sent successfully.");
    // rclcpp::sleep_for(500ms);
  }
  rclcpp::shutdown();
  return 0;
}
