cmake_minimum_required(VERSION 3.5)
project(robot_manipulator_manager)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

find_package(std_msgs REQUIRED)

find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(${PROJECT_NAME} REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


# ----- ros2 msg -----

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ManipulatorState.msg"
  DEPENDENCIES builtin_interfaces
)

# ----- setup python scripts -----

ament_python_install_package(${PROJECT_NAME}/)

install(PROGRAMS
  ${PROJECT_NAME}/robot_manipulator_manager.py
  DESTINATION bin
)

ament_package()


