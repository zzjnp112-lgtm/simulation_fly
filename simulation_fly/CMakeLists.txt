cmake_minimum_required(VERSION 3.8)
project(simulation_fly)

# Default to C++1
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(gz-transport QUIET)
find_package(quadrotor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# If the unversioned package is not available, try the system versioned package
if(NOT TARGET gz-transport::gz-transport)
  find_package(gz-transport13 QUIET)
  if(TARGET gz-transport13::gz-transport13)
    set(GZ_TRANSPORT_LIB gz-transport13::gz-transport13)
  else()
    # Try a required search so CMake will emit a clear error if nothing is found
    find_package(gz-transport REQUIRED)
    set(GZ_TRANSPORT_LIB gz-transport::gz-transport)
  endif()
else()
  set(GZ_TRANSPORT_LIB gz-transport::gz-transport)
endif()
find_package(nav_msgs REQUIRED)





# 包含头文件目录
include_directories(
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/src/include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${mavros_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${PROJECT_NAME}

)





# 添加可执行文件
add_executable(fly_simpx4 src/fly_simpx4.cpp)
add_executable(camer_bridge src/camer_bridge.cpp)
add_executable(time_bridge src/time_bridge.cpp)
add_executable(point_bridge src/point_bridge.cpp)


# 添加依赖
ament_target_dependencies(fly_simpx4
  rclcpp
  geometry_msgs
  mavros_msgs
  std_msgs

  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
 
  quadrotor_msgs
  nav_msgs

)

ament_target_dependencies(time_bridge
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)



ament_target_dependencies(point_bridge
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
  )


# 摄像头桥需要链接 rclcpp 和 消息包，否则会出现未定义引用的链接错误
ament_target_dependencies(camer_bridge
  rclcpp
  sensor_msgs
  std_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_srvs
)

target_link_libraries(time_bridge
  ${GZ_TRANSPORT_LIB}
)

target_link_libraries(camer_bridge
  ${GZ_TRANSPORT_LIB}
)


install(TARGETS
  camer_bridge
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS
  time_bridge
  DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS
  point_bridge
  DESTINATION lib/${PROJECT_NAME}
)


# 安装目标
install(TARGETS
  fly_simpx4
  DESTINATION lib/${PROJECT_NAME}
)



# 导出依赖
ament_export_dependencies(
  rclcpp
  geometry_msgs
  mavros_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  tf2_ros
  quadrotor_msgs
  nav_msgs



)

ament_export_include_directories(include)


ament_package()
