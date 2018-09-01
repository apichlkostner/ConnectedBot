cmake_minimum_required(VERSION 2.8.3)
set (CMAKE_CXX_STANDARD 11)

include_directories(${ROS_LIB_DIR} "lib" "lib/SparkFun_RedBot_Arduino_Library/src" )

# Remove this if using an Arduino without native USB (eg, other than Leonardo)
add_definitions(-DUSB_CON)

generate_arduino_firmware(connected_bot_fw
  SRCS main.cpp ${ROS_LIB_DIR}/time.cpp lib/DistanceSensor.cpp
  lib/SparkFun_RedBot_Arduino_Library/src/RedBotEncoder.cpp
  lib/SparkFun_RedBot_Arduino_Library/src/RedBot.cpp
  lib/SparkFun_RedBot_Arduino_Library/src/RedBotMotors.cpp
  BOARD uno
  PORT /dev/ttyUSB0
)