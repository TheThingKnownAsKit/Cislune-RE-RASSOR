cmake_minimum_required(VERSION 3.8)          # ≥ 3.8 required by ament_cmake
project(rover_firmware NONE)                 # “NONE” → no languages / no build

find_package(ament_cmake REQUIRED)           # still needed for ament index

find_program(PLATFORMIO_CMD platformio)
if(PLATFORMIO_CMD)
  add_custom_target(pio_build
    COMMAND ${PLATFORMIO_CMD} run -e teensy41         # compile firmware
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Building Teensy firmware with PlatformIO"
  )
else()
  message(WARNING "platformio not found – 'pio_build' target is disabled.")
endif()

install(
  FILES
    .pio/build/teensy41/firmware.hex            # path after a PlatformIO build
  DESTINATION
    share/${PROJECT_NAME}
  OPTIONAL                                     # ignores if file is missing
)

ament_package()
