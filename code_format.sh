# !/bin/sh
ament_uncrustify --reformat include/roboclaw_ros2/
ament_uncrustify --reformat src/
echo "Code formatting completed using ament_uncrustify."