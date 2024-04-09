cmake_minimum_required(VERSION 3.5)
project(ros2_maxxii)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial REQUIRED)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

add_library(driver_interface include/${PROJECT_NAME}/driver_interface.cpp)
add_library(motor include/${PROJECT_NAME}/motor.cpp)
add_library(encoder include/${PROJECT_NAME}/encoder.cpp)

add_executable(maxxii_node src/maxxii_node.cpp)

target_link_libraries(maxxii_node 
  motor
  encoder
  driver_interface
  )

target_include_directories(maxxii_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies(maxxii_node 
  rclcpp 
  geometry_msgs
  sensor_msgs
  serial 
  std_msgs)

install(TARGETS maxxii_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(${PROJECT_NAME}_test 
    test/main.cpp
    test/encoder_unittest.cpp
    test/motor_unittest.cpp
    test/driver_interface_unittest.cpp
  )
  target_include_directories(${PROJECT_NAME}_test PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}_test 
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
  )
  target_link_libraries(${PROJECT_NAME}_test encoder)
  target_link_libraries(${PROJECT_NAME}_test motor)
  target_link_libraries(${PROJECT_NAME}_test driver_interface)
endif()


ament_package()
