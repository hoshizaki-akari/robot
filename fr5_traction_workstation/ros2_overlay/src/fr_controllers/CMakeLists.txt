cmake_minimum_required(VERSION 3.8)
project(fr_controllers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找所有需要的包
find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(fairino_msgs REQUIRED)
find_package(tf2_kdl REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)


# --- 构建控制器库 ---
# 1. 定义库的名称和源文件
add_library(${PROJECT_NAME} SHARED
  src/cartesian_velocity_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)


# 2. 指定头文件的包含目录
ament_target_dependencies(${PROJECT_NAME}
  PUBLIC
    controller_interface
    hardware_interface
    pluginlib
    rclcpp
    rclcpp_lifecycle
    std_msgs
    kdl_parser
    realtime_tools
    fairino_msgs
    tf2_kdl
    tf2_geometry_msgs
)

# 3. 链接依赖库
# target_link_libraries(${PROJECT_NAME}
#   ${kdl_parser_LIBRARIES}
# )

# --- 导出插件 ---
# 4. 使用pluginlib导出插件描述文件
pluginlib_export_plugin_description_file(controller_interface controller_plugins.xml)

# --- 安装 ---
# 5. 安装库文件到lib目录
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 6. 安装include目录下的所有头文件
install(DIRECTORY include/
  DESTINATION include
)

# 7. 安装插件描述XML文件
install(FILES controller_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# --- ament lint测试 ---
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
