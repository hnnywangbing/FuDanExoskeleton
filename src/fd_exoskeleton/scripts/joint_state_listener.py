#!/usr/bin/env python3

import rospy
from std_msgs.msg import String
from unitree_legged_msgs.msg import MotorCmd
from sensor_msgs.msg import JointState
def callback(joints):
    rospy.loginfo(f'joints: {joints}')

def listener():
    # 在 ROS 中，节点名称唯一。如果启动两个同名节点，第一个会被关闭。
    # anonymous=True 标志意味着 rospy 将为节点选择一个唯一的名称，允许多个监听器同时运行。
    rospy.init_node('joint_state_listener', anonymous=True)
    # 订阅 'chatter_recv' 话题，接收 String 类型的消息，并调用回调函数
    rospy.Subscriber('/exo_humanoid_gazebo/L_hip_controller/command', MotorCmd, callback)

    # spin() 方法会保持 Python 进程运行，直到节点被停止
    rospy.spin()

if __name__ == '__main__':
    listener()
