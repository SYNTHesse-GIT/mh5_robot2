cmake_minimum_required(VERSION 3.8)
project(mh5_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(realtime_tools REQUIRED)

generate_parameter_library(diagnostic_message_broadcaster_parameters
  src/diagnostic_message_broadcaster_parameters.yaml
)

add_library(mh5_controllers SHARED 
  src/diagnostic_message_broadcaster.cpp
)
target_include_directories(mh5_controllers PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/mh5_controllers>
)
target_link_libraries(mh5_controllers PUBLIC
  diagnostic_message_broadcaster_parameters
)
ament_target_dependencies(mh5_controllers PUBLIC
  controller_interface
  diagnostic_msgs
  generate_parameter_library
  realtime_tools
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY include/
  DESTINATION include/mh5_controllers
)
install(
  TARGETS
    mh5_controllers
    diagnostic_message_broadcaster_parameters
  EXPORT export_mh5_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_mh5_controllers HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
