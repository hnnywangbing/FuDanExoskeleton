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

# 配置传感器端口：查看端口 ls /dev/ttyUSB*
# 端口授权
sudo chmod 666 /dev/ttyUSB0 # Imu 注意 重新插拔端口会变 
sudo chmod 666 /dev/ttyUSB1 # lida

# 主控端节点启动 1.中间件节点 bridge_node 2.realsense节点  3.app 服务节点
roslaunch fd_exoskeleton main_control.launch

# 电机控制端节点启动 1.中间件节点 2.启动跌倒位置控制服务节点   3.mpc力矩控制服务节点 joint_effort  4.助力模式节点 joint_effort 
roslaunch fd_exoskeleton joint_control.launch

# 传感器端节点启动 1.中间件节点  2.Imu 节点  3.坐下座椅检测 lida 服务节点
roslaunch fd_exoskeleton sensor_listener.launch

# 通信消息中间件 
src/bridge_node
# 自定义消息
msg
#宇树消息体
unitree_legged_msgs
# IMU  惯导 节点 、下蹲位置控制节点需订阅Imu跌倒检测执行 
# head_type == TYPE_IMU 四元数上传主控MPC   
# head_type == TYPE_IMU  head_type == TYPE_AHRS  跌倒判断
# topic /imu_state_sensor 对应mpc imu topic "/gazebo/model_states"  
# 把mpc imu  topic 修改为/imu_state_sensor 与仿真环境imu 区分
scripts/Imu.py  
rosrun fd_exoskeleton Imu.py

# lida 椅子高度检测 节点 
scripts/lida.py
rosrun fd_exoskeleton lida.py

# realsense 楼梯检测 节点  需要启动 realsense-ros 节点 https://github.com/IntelRealSense/realsense-ros.git 使用 ros1-legacy 分支
scripts/realsense.py
rosrun fd_exoskeleton realsense.py

# app控制服务 节点 发布控制指令与mpc 通信 
# handle_requests 接收app控制指令发送topic mpc需订阅对应指令进行步态控制，对应键盘控制，助力模式助力等级发送
# publish_updates 向app发送各个模块发送的健康信息，模块通讯、传感器是否异常等。

scripts/app.py
rosrun fd_exoskeleton app.py


