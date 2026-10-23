cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  
  "msg/BnoReading.msg"
  "msg/Encoder.msg"
  "msg/Lidar.msg"
  "msg/LimitSwitch.msg"
  "msg/Velocity.msg"
  "msg/BallScrew.msg"
  "msg/ClimbMotors.msg"
  "msg/Odom.msg"
  "msg/Irsensors.msg"
  "msg/ServoSpearhead.msg"
  "srv/ServiceCall.srv"
  "srv/KfsServicePickup.srv"
  "srv/PickSpearhead.srv"
  "srv/CornerAlignService.srv"
  "msg/IR.msg"
  "msg/ClimbJhonsonEnc.msg"
  "msg/GridData.msg"
  "msg/Grippers.msg"
  "msg/DepthIR.msg"
  "msg/ClimbIR.msg"
  "msg/KfsServiceLimitSwitch.msg"
  "msg/GantryVel.msg"
  "srv/Zone3Detect.srv"
  "msg/Jenc.msg"
  "msg/JmotorVelocity.msg"
  "msg/ReedSwitch.msg"
  "msg/SpearheadgripperTOF.msg"


)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
