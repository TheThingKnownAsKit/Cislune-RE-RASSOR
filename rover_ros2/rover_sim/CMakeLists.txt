cmake_minimum_required(VERSION 3.8)
project(rover_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_program(GZ_EXECUTABLE gz)

install(DIRECTORY config models worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
