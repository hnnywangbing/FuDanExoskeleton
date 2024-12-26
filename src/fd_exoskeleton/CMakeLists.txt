cmake_minimum_required(VERSION 3.0.2)
project(fd_exoskeleton)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  message_generation
)

# 包含 libzmqpp 的头文件目录
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/local/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src 
)

# 查找 libzmq 库
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ REQUIRED libzmq)

set(CMAKE_LIBRARY_PATH "/usr/local/lib" ${CMAKE_LIBRARY_PATH})
find_library(libzmqpp zmqpp)
# 查找 libzmqpp 库
# find_library(libzmqpp zmqpp HINTS /usr/local/lib)

# 添加可执行文件
add_executable(bridge_node src/bridge_node.cpp)

add_message_files(
  FILES
  CmdMessage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)


# 链接 libzmqpp 库和 libzmq 库
target_link_libraries(bridge_node
  ${catkin_LIBRARIES}
  ${ZMQ_LIBRARIES}
  ${libzmqpp}
)

# 在target_link_libraries之后添加
set_target_properties(bridge_node PROPERTIES INSTALL_RPATH "/usr/lib/x86_64-linux-gnu")

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs sensor_msgs message_runtime
)

# 安装脚本
install(PROGRAMS
  scripts/listener.py
  scripts/talker.py
  scripts/Imu.py
  scripts/lida.py
  scripts/app.py
  scripts/realsense.py
  scripts/Imutest.py
  scripts/joint_state_control.py
  scripts/joint_state_listener.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# 安装启动文件
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装配置文件
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# 安装目标
install(TARGETS bridge_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


# catkin_install_python(PROGRAMS
#   scripts/lada.py
#   scripts/Imu.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
