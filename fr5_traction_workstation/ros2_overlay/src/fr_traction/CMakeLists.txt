cmake_minimum_required(VERSION 3.8)
project(fr_traction)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TractionCommand.msg"
  "msg/TractionStatus.msg"
  "msg/TractionRecordSummary.msg"
  "msg/TractionHistory.msg"
  "srv/SetTargetForce.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs
)

add_library(fr_traction_core SHARED
  src/direction_correction.cpp
  src/traction_math.cpp
  src/traction_safety.cpp
  src/traction_state_machine.cpp
  src/traction_controller_core.cpp
)
target_include_directories(fr_traction_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(traction_controller_node src/traction_controller_node.cpp)
target_link_libraries(traction_controller_node fr_traction_core)
ament_target_dependencies(traction_controller_node
  geometry_msgs rclcpp std_msgs
)

add_executable(traction_manager_node src/traction_manager_node.cpp)
target_link_libraries(traction_manager_node fr_traction_core)
ament_target_dependencies(traction_manager_node
  builtin_interfaces controller_manager_msgs fairino_msgs geometry_msgs rclcpp sensor_msgs std_msgs std_srvs
)

rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(traction_controller_node "${cpp_typesupport_target}")
target_link_libraries(traction_manager_node "${cpp_typesupport_target}")

install(TARGETS fr_traction_core traction_controller_node traction_manager_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/check_traction_runtime.py
  scripts/fr5_direct_driver_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  # The ROS package schema URL is not reachable from the offline WSL build
  # image; package XML is checked locally with xmllint in the validation record.
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_xmllint)

  ament_add_gtest(test_traction_math test/test_traction_math.cpp)
  target_link_libraries(test_traction_math fr_traction_core)

  ament_add_gtest(test_traction_state_machine test/test_traction_state_machine.cpp)
  target_link_libraries(test_traction_state_machine fr_traction_core)

  ament_add_gtest(test_traction_controller_core test/test_traction_controller_core.cpp)
  target_link_libraries(test_traction_controller_core fr_traction_core)

  ament_add_gtest(test_traction_safety test/test_traction_safety.cpp)
  target_link_libraries(test_traction_safety fr_traction_core)

  ament_add_gtest(test_traction_synthetic_pipeline test/test_traction_synthetic_pipeline.cpp)
  target_link_libraries(test_traction_synthetic_pipeline fr_traction_core)

  ament_add_gtest(test_direction_correction test/test_direction_correction.cpp)
  target_link_libraries(test_direction_correction fr_traction_core)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(fr_traction_core)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
