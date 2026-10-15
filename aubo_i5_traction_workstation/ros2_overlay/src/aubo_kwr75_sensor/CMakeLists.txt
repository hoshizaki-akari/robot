cmake_minimum_required(VERSION 3.8)
project(aubo_kwr75_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Threads REQUIRED)

add_library(kwr75_protocol SHARED
  src/kwr75_protocol.cpp
)
target_include_directories(kwr75_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(kwr75_sensor_node src/kwr75_sensor_node.cpp)
target_link_libraries(kwr75_sensor_node kwr75_protocol Threads::Threads)
ament_target_dependencies(kwr75_sensor_node geometry_msgs rclcpp std_msgs)

install(TARGETS kwr75_protocol kwr75_sensor_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint ament_cmake_xmllint)

  ament_add_gtest(test_kwr75_protocol test/test_kwr75_protocol.cpp)
  target_link_libraries(test_kwr75_protocol kwr75_protocol)

  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(kwr75_protocol)
ament_package()
