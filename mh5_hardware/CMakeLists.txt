cmake_minimum_required(VERSION 3.8)
project(mh5_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(pluginlib REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include_directories(include)

add_library(mh5_hardware SHARED 
  src/dynamixel_bus.cpp
)

target_include_directories(mh5_hardware PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(mh5_hardware PUBLIC
  hardware_interface
  rclcpp
  rclcpp_lifecycle
  pluginlib
  dynamixel_sdk
)

pluginlib_export_plugin_description_file(
  hardware_interface
  mh5_hardware_plugin.xml
)

install(TARGETS mh5_hardware
  EXPORT export_mh5_hardware
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install (
  DIRECTORY config urdf launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_libraries(
  mh5_hardware
)

ament_export_targets(
  export_mh5_hardware
)

ament_package()
