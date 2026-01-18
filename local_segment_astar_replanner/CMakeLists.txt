cmake_minimum_required(VERSION 3.26.3)
project(local_segment_astar_replanner)

# 关键：设置编译类型为Debug，生成调试符号
set(CMAKE_BUILD_TYPE Debug)  # 优先级最高，强制开启Debug模式
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")  # -g生成调试符号，-O0关闭优化
set(CMAKE_CXX_FLAGS_RELEASE "-g -O0")  # 防止误切换到Release模式，仍保留调试符号

# 查找依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system chrono)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(include)

include_directories(${Boost_INCLUDE_DIRS})
include_directories(./GridMap2D/include)
include_directories(./path_searching/include/path_searching)
include_directories(./manager/include)
add_subdirectory(GridMap2D)
add_subdirectory(path_searching)
add_subdirectory(manager)

# 编译节点
add_executable(local_segment_astar_replanner src/trajectory_publisher.cpp)
ament_target_dependencies(
  local_segment_astar_replanner
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  sensor_msgs
)

target_link_libraries(
  local_segment_astar_replanner
  path_searching
  local_map
  manager

)

# 安装节点
install(TARGETS
local_segment_astar_replanner
DESTINATION lib/${PROJECT_NAME})

ament_package()