cmake_minimum_required(VERSION 3.8)
project(syncai_common)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# MappingStatus.stamp is a builtin_interfaces/Time and NavigateToGoal carries
# geometry_msgs/PoseStamped (goal and feedback) -- the fields here of another
# package's type, so the DEPENDENCIES entry below is load-bearing for both.
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArtifactState.msg"
  "msg/IMUState.msg"
  "msg/MappingStatus.msg"
  "msg/MotorState.msg"
  "msg/MotorStates.msg"
  "msg/RobotBatteryStatus.msg"
  "msg/RobotLocalizationStatus.msg"
  "msg/RobotLowLevelMode.msg"
  "msg/RobotMode.msg"
  "msg/RobotNetworkStatus.msg"
  "msg/RobotPose.msg"
  "msg/RobotState.msg"
  "msg/RobotStatus.msg"
  "msg/WifiNetwork.msg"
  "msg/WifiStatus.msg"
  "srv/ConnectWifiNetwork.srv"
  "srv/GetMode.srv"
  "srv/ScanWifiNetworks.srv"
  "srv/SetMotionKey.srv"
  "srv/SetPolicyMode.srv"
  "srv/SetSpeedScale.srv"
  "srv/SwitchMode.srv"
  "srv/RestartMode.srv"
  "srv/ResetLIO.srv"
  "srv/SaveMaps.srv"
  "srv/ResetMapping.srv"
  "srv/StartMapping.srv"
  "srv/RefineMap.srv"
  "srv/SavePoses.srv"
  "srv/Relocalize.srv"
  "srv/IsValid.srv"
  "action/ExecuteTask.action"
  "action/NavigateToGoal.action"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
