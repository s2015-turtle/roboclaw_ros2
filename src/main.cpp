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

  Roboclaw->set_minimum_main_voltage(6);

  Roboclaw->set_maximum_main_voltage(24);

  while(rclcpp::ok()) {
    Roboclaw->drive_M1(65);
    //  rclcpp::sleep_for(100ms);
    auto result = Roboclaw->read_encoder_count_M1();


    RCLCPP_INFO(rclcpp::get_logger("main"), "Encoder counts: M1: %u", result.value_or(0));
    rclcpp::sleep_for(100ms);
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
