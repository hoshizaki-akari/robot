cmake_minimum_required(VERSION 3.8)
project(aubo_v4_driver)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion)
endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(fr_traction REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
set(AUBO_V4_SDK_ROOT "$ENV{AUBO_V4_SDK_ROOT}" CACHE PATH "Root of the externally installed AUBOPE V4 legacy SDK")
if(NOT AUBO_V4_SDK_ROOT)
  message(FATAL_ERROR
    "AUBO_V4_SDK_ROOT is required. Point it to an external AUBOPE V4 SDK."
  )
endif()
find_path(AUBO_V4_INCLUDE_DIR NAMES aubo_driver/serviceinterface.h PATHS "${AUBO_V4_SDK_ROOT}/include" NO_DEFAULT_PATH)
find_library(AUBO_V4_CONTROLLER_LIBRARY NAMES auborobotcontroller PATHS "${AUBO_V4_SDK_ROOT}/lib/lib64/aubocontroller" NO_DEFAULT_PATH)
find_library(AUBO_V4_CONFIG_LIBRARY NAMES config PATHS "${AUBO_V4_SDK_ROOT}/lib/lib64/config" NO_DEFAULT_PATH)
find_library(AUBO_V4_LOG4CPLUS_LIBRARY NAMES log4cplus PATHS "${AUBO_V4_SDK_ROOT}/lib/lib64/log4cplus" NO_DEFAULT_PATH)
if(NOT AUBO_V4_INCLUDE_DIR OR NOT AUBO_V4_CONTROLLER_LIBRARY OR NOT AUBO_V4_CONFIG_LIBRARY OR NOT AUBO_V4_LOG4CPLUS_LIBRARY)
  message(FATAL_ERROR
    "AUBOPE V4 SDK is incomplete under AUBO_V4_SDK_ROOT='${AUBO_V4_SDK_ROOT}'."
  )
endif()
add_library(aubo_v4_state_validation SHARED src/state_validation.cpp)
target_include_directories(aubo_v4_state_validation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
add_executable(aubo_v4_state_node src/aubo_v4_state_node.cpp)
target_include_directories(aubo_v4_state_node PRIVATE "${AUBO_V4_INCLUDE_DIR}")
target_link_libraries(aubo_v4_state_node
  aubo_v4_state_validation
  "${AUBO_V4_CONTROLLER_LIBRARY}"
  "${AUBO_V4_CONFIG_LIBRARY}"
  "${AUBO_V4_LOG4CPLUS_LIBRARY}"
)
# The legacy controller binary records libprotobuf.so.9 as a transitive NEEDED
# dependency. The supplied audit bundle has no compatible protobuf binary, so
# permit an offline link while making the runtime gap explicit in documentation.
target_link_options(aubo_v4_state_node PRIVATE "-Wl,--allow-shlib-undefined")
set_target_properties(aubo_v4_state_node PROPERTIES BUILD_RPATH
  "${AUBO_V4_SDK_ROOT}/lib/lib64/aubocontroller;${AUBO_V4_SDK_ROOT}/lib/lib64/config;${AUBO_V4_SDK_ROOT}/lib/lib64/log4cplus"
)
ament_target_dependencies(aubo_v4_state_node
  controller_manager_msgs diagnostic_msgs fr_traction geometry_msgs rclcpp sensor_msgs std_msgs std_srvs
)
install(TARGETS aubo_v4_state_validation aubo_v4_state_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(FILES README.md stage_report.txt DESTINATION share/${PROJECT_NAME})
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  set(AMENT_LINT_AUTO_EXCLUDE ament_cmake_copyright ament_cmake_cpplint ament_cmake_xmllint)
  ament_add_gtest(test_state_validation test/test_state_validation.cpp)
  target_link_libraries(test_state_validation aubo_v4_state_validation)
  ament_lint_auto_find_test_dependencies()
endif()
ament_export_include_directories(include)
ament_export_libraries(aubo_v4_state_validation)
ament_package()
