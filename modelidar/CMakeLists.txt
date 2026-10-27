cmake_minimum_required(VERSION 3.16)
project(modelidar LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# set the same behavior for windows as it is on linux
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Specify the required version of ros2_control
find_package(controller_manager 4.0.0)
# Handle the case where the required version is not found
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. "
  "Are you using the correct branch of the ros2_control_demos repository?")
endif()

# find dependencies
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(cxx-serial REQUIRED)

## COMPILE
add_library(
  modelidar
  SHARED
  hardware/modelidar_system.cpp
)

target_link_libraries(modelidar PRIVATE cxx-serial)

target_compile_features(modelidar PUBLIC cxx_std_17)
target_include_directories(modelidar PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/modelidar>
)
ament_target_dependencies(
  modelidar PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface modelidar.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/modelidar
)
install(
  DIRECTORY launch urdf meshes
  DESTINATION share/modelidar
)
install(
  DIRECTORY config udev
  DESTINATION share/modelidar
)
# Fake robot motion for RViz (launch/demo_launch.py): ros2 run modelidar state_publisher
install(
  PROGRAMS modelidar/mock_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME state_publisher
)
# Activates the board and the controllers once the board is there (launch/modelidar.launch.py)
install(
  PROGRAMS modelidar/board_supervisor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME board_supervisor
)


install(TARGETS modelidar
  EXPORT export_modelidar
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Serial protocol and wheel conversions, without hardware
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_modelidar_comms test/test_modelidar_comms.cpp)
  target_include_directories(test_modelidar_comms PRIVATE hardware/include)
  target_link_libraries(test_modelidar_comms cxx-serial)
  ament_target_dependencies(test_modelidar_comms hardware_interface)
endif()

## EXPORTS
ament_export_targets(export_modelidar HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()