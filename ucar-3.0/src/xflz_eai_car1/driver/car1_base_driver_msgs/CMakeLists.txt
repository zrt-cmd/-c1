cmake_minimum_required(VERSION 3.8)
project(car1_base_driver_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotorState.msg"
  "msg/MotorsState.msg"
  "msg/BusServoState.msg"
  "msg/SetBusServoState.msg"
  "msg/PWMServoState.msg"
  "msg/SetPWMServoState.msg"
  "msg/ServoPosition.msg"
  "msg/ServosPosition.msg"
  "msg/BusServoStates.msg"
  "msg/GetBusServoCmd.msg"
  "msg/GetPWMServoCmd.msg"
  "msg/ButtonState.msg"
  "msg/BuzzerState.msg"
  "msg/LedState.msg"
  "msg/OLEDState.msg"
  "msg/RGBState.msg"
  "msg/RGBStates.msg"
  "msg/Sbus.msg"
  "srv/GetBusServoState.srv"
  "srv/GetPWMServoState.srv"
  DEPENDENCIES std_msgs
)

ament_package()
