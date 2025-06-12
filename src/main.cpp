#include "rclcpp/rclcpp.hpp"
#include "roboclaw_ros2/roboclaw.hpp"

#include <vector>
#include <chrono>
using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string port = "/dev/ttyACM0";
  if(argc > 1) {
    //RCLCPP_INFO(rclcpp::get_logger("main"),"argc data: %d", argc);
    port = argv[1];
  }

  auto Roboclaw = std::make_shared<roboclaw_ros2::Roboclaw>(port, 38400, 0x80);

  while(rclcpp::ok()) {
    std::vector<uint8_t> dataToSend = {0x80, 0x0, 30};
    std::vector<uint8_t> dataToSend2 = {0x80, 0x0, 60};
    auto result = Roboclaw->set_minimum_main_voltage(12);
    result = Roboclaw->set_maximum_main_voltage(15);
    rclcpp::sleep_for(10ms);
    result = Roboclaw->drive_forward_M1(50);
    if (!result) {
      RCLCPP_ERROR(rclcpp::get_logger("main"), "Failed to send data.");
      break;
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
