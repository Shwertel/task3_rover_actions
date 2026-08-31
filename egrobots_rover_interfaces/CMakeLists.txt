cmake_minimum_required(VERSION 3.8)
project(egrobots_rover_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)

# Interface generation needs an ament_cmake package: a pure ament_python one
# cannot run rosidl, which is why these definitions live apart from the node.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/MoveToGoal.action"
  DEPENDENCIES geometry_msgs
)

ament_package()
