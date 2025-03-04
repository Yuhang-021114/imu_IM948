cmake_minimum_required(VERSION 3.8)
project(imu_im948)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖项
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(serial REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(im948_CMD
  src/im948_CMD.cpp
)
target_include_directories(im948_CMD PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(im948_CMD rclcpp std_msgs sensor_msgs serial tf2 tf2_geometry_msgs)

# 添加可执行文件
add_executable(imu_node src/imuDo.cpp)

target_link_libraries(imu_node im948_CMD)

# 关联依赖项
ament_target_dependencies(imu_node rclcpp std_msgs sensor_msgs serial tf2 tf2_geometry_msgs)

# 安装可执行文件
install(TARGETS
  imu_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
