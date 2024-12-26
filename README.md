# FuDanExoskeleton

# 编译
catkin clean
catkin config --install
catkin build
source install/setup.bash

# 主控通信配置
config/main_topics.yaml
# 传感器模块通信配置
config/sensor_topics.yaml
# 外骨骼关机通信配置
config/joint_topics.yaml

# 通信消息中间件 
src/bridge_node

# 自定义消息
msg

# IMU  惯导 节点 、下蹲位置控制节点需订阅Imu跌倒检测执行
scripts/Imu.py
rosrun fd_exoskeleton Imu.py

# lida 椅子高度检测 节点 
scripts/lida.py
rosrun fd_exoskeleton lida.py

# realsense 楼梯检测 节点 
scripts/check_staircase.py
rosrun fd_exoskeleton check_staircase.py

# app控制服务 节点 发布控制指令与mpc 通信
scripts/app.py
rosrun fd_exoskeleton app.py


# 主控端节点启动
roslaunch fd_exoskeleton main_control.launch

# 电机控制端节点启动
roslaunch fd_exoskeleton joint_control.launch

# 传感器端节点启动
roslaunch fd_exoskeleton sensor_listener.launch