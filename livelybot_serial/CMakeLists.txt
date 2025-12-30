cmake_minimum_required(VERSION 3.8)
project(livelybot_serial)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(livelybot_msg REQUIRED)
find_package(serial REQUIRED)

set(serial_SRCS
  src/lively_serial.cc
  src/hardware/motor.cc
)

find_library(SERIALPORT_LIBRARY serialport)
if(NOT SERIALPORT_LIBRARY)
  message(FATAL_ERROR "libserialport not found")
endif()

add_library(${PROJECT_NAME} SHARED ${serial_SRCS})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  std_msgs
  livelybot_msg
  serial
)
target_link_libraries(${PROJECT_NAME}
  rt
  pthread
  ${SERIALPORT_LIBRARY}
)

function(add_livelybot_serial_executable target source)
  add_executable(${target} ${source})
  target_include_directories(${target} PRIVATE include)
  ament_target_dependencies(${target}
    rclcpp
    std_msgs
    livelybot_msg
    serial
  )
  target_link_libraries(${target} ${PROJECT_NAME})
  install(TARGETS ${target}
    DESTINATION lib/${PROJECT_NAME}
  )
endfunction()

add_livelybot_serial_executable(lvbot_test src/serial_t.cpp)
add_livelybot_serial_executable(ik_run src/ik_run.cpp)
add_livelybot_serial_executable(robot_node_ src/robot_node.cpp)
add_livelybot_serial_executable(test_motor test/test_motor.cpp)
add_livelybot_serial_executable(test_reset_zero test/test_reset_zero.cpp)
add_livelybot_serial_executable(test_feedback test/test_feedback.cpp)
add_livelybot_serial_executable(test_motor_run test/test_motor_run.cpp)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(rclcpp std_msgs livelybot_msg serial)

ament_package()

