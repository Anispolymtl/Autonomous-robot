cmake_minimum_required(VERSION 3.8)
project(limo_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# Generate interfaces (actions + services)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DoMission.action"
  "srv/GetScript.srv"
  "srv/SaveScript.srv"
  DEPENDENCIES action_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
