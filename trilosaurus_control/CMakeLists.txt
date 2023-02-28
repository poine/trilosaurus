cmake_minimum_required(VERSION 3.8)
project(trilosaurus_control)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rsl REQUIRED)
find_package(pluginlib)
find_package(rosidl_default_generators REQUIRED)

# set(THIS_PACKAGE_INCLUDE_DEPENDS
#     controller_interface
#     geometry_msgs
#     hardware_interface
#     nav_msgs
#     pluginlib
#     rclcpp
#     rclcpp_lifecycle
#     rcpputils
#     realtime_tools
#     tf2
#     tf2_msgs
# )
# foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
#   find_package(${Dependency} REQUIRED)
# endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Report.msg"
)


find_package(diff_drive_controller REQUIRED)

include_directories(include)
# Create Cpp executable
add_executable(cpp_executable src/cpp_node.cpp)
ament_target_dependencies(cpp_executable rclcpp diff_drive_controller rsl)

# Create Cpp lib
add_library(trilosaurus_diff_drive_controller SHARED
  src/trilosaurus_diff_drive_controller.cpp
)
ament_target_dependencies(trilosaurus_diff_drive_controller rclcpp diff_drive_controller rsl)
rosidl_target_interfaces(trilosaurus_diff_drive_controller  ${PROJECT_NAME} "rosidl_typesupport_cpp")
#rosidl_get_typesupport_target(trilosaurus_diff_drive_controller  ${PROJECT_NAME} "rosidl_typesupport_cpp")
#target_link_libraries(trilosaurus_diff_drive_controller rosidl)
#instead of rosidl_target_interfaces()




# Install Cpp executables
install(TARGETS
  cpp_executable
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

# Install Python modules
#ament_python_install_package(${PROJECT_NAME})
# Install Python executables
install(PROGRAMS
  scripts/py_node.py
  scripts/send_test_twist.py
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS trilosaurus_diff_drive_controller
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
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

  find_package(ament_cmake_gmock REQUIRED)
  find_package(controller_manager REQUIRED)
  find_package(ros2_control_test_assets REQUIRED)

  ament_add_gmock(test_diff_drive_controller
    test/test_diff_drive_controller.cpp
    ENV config_file=${CMAKE_CURRENT_SOURCE_DIR}/test/config/test_diff_drive_controller.yaml)
  target_include_directories(test_diff_drive_controller PRIVATE include)
  target_link_libraries(test_diff_drive_controller trilosaurus_diff_drive_controller )

  ament_target_dependencies(test_diff_drive_controller
    geometry_msgs
    hardware_interface
    nav_msgs
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    tf2
    tf2_msgs
    rsl
    )

  ament_add_gmock(
    test_load_controller
    test/test_load_controller.cpp
  )
  target_include_directories(test_load_controller PRIVATE include)
  target_link_libraries(test_load_controller trilosaurus_diff_drive_controller )
  ament_target_dependencies(test_load_controller
    controller_manager
    ros2_control_test_assets
  )
  
endif()

pluginlib_export_plugin_description_file(controller_interface trilosaurus_diff_drive_controller.xml)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
