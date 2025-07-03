cmake_minimum_required(VERSION 3.8)
project(rover_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rcl REQUIRED)
find_package(rclc REQUIRED)
find_package(std_msgs REQUIRED)

# Update this
add_executable(${PROJECT_NAME} src/main.cpp)

ament_target_dependencies(${PROJECT_NAME}
  rcl
  rclc
  std_msgs
  builtin_interfaces
  )

install(DIRECTORY src
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
