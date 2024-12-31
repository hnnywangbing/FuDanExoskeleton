# FuDanExoskeleton
# 环境 ubuntu 20.04 ROS1 

# 通讯架构使用 swarm_ros_bridge 开源项目

# 通讯库文件 依赖库 libzmqpp  可源码安装
 sudo apt-get install libzmqpp-dev
 
# 消息中间件 依赖库 zmqpp  可源码安装
sudo apt-get install libzmq3-dev

# Build, check, and install the latest version of ZeroMQ
git clone git://github.com/zeromq/libzmq.git
cd libzmq
./autogen.sh 
./configure --with-libsodium && make
sudo make install
sudo ldconfig
cd ../
# Now install ZMQPP
git clone git://github.com/zeromq/zmqpp.git
cd zmqpp
make
make check
sudo make install
make installcheck

# 编译
catkin clean
catkin config --install
catkin build
source install/setup.bash

# 主控通信配置  send_topics 发送端topic 与 接收端 recv_topics IP 、数据类型、端口及topic名称要对应
config/main_topics.yaml
# 传感器模块通信配置
config/sensor_topics.yaml
# 外骨骼关机通信配置
config/joint_topics.yaml
# 启动关节点可灵活配置

# python 存放位置 scripts CMakeLists.txt install 添加安装python脚本

# 配置传感器端口：查看端口ls /dev/ttyUSB*
# 端口授权
sudo chmod 666 /dev/ttyUSB0 # Imu 注意 重新插拔端口会变 
sudo chmod 666 /dev/ttyUSB1 # lida

# 主控端节点启动
roslaunch fd_exoskeleton main_control.launch

# 电机控制端节点启动
roslaunch fd_exoskeleton joint_control.launch

# 传感器端节点启动
roslaunch fd_exoskeleton sensor_listener.launch


# 通信消息中间件 
src/bridge_node
# 自定义消息
msg
#宇树消息体
unitree_legged_msgs
# IMU  惯导 节点 、下蹲位置控制节点需订阅Imu跌倒检测执行
scripts/Imu.py
rosrun fd_exoskeleton Imu.py

# lida 椅子高度检测 节点 
scripts/lida.py
rosrun fd_exoskeleton lida.py

# realsense 楼梯检测 节点  需要启动 realsense-ros 节点 https://github.com/IntelRealSense/realsense-ros.git 使用 ros1-legacy 分支
scripts/realsense.py
rosrun fd_exoskeleton realsense.py

# app控制服务 节点 发布控制指令与mpc 通信
scripts/app.py
rosrun fd_exoskeleton app.py


