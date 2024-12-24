#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

def talker():
    # 初始化节点
    rospy.init_node('joint_state_publisher', anonymous=True)
    
    # 创建一个发布者，发布到'/JointState'话题
    pub = rospy.Publisher('/JointState', JointState, queue_size=10)
    
    # 设置发布频率（例如，每秒10次）
    rate = rospy.Rate(10) 

    # 初始化JointState消息
    joint_state = JointState()
    joint_state.header.stamp = rospy.Time.now()
    joint_state.name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']
    joint_state.position =[1.1,2.1,3.1,4.4,5.4,6.4,7.4,8.4]  
    joint_state.velocity =[1.2,2.3,3.3,4.3,5.3,6.3,7.3,8.3]  
    joint_state.effort =[1.0,2.0,3.0,4.0,5.0,6.0,7.0,8.0]  

    # 模拟关节位置变化
    position_change = 0.1

    while not rospy.is_shutdown():
        # 更新时间戳
        joint_state.header.stamp = rospy.Time.now()
        
        # 更新关节位置（这里只是简单地增加一个固定值）
        joint_state.position = [(pos + position_change) % (2 * 3.14159265359) for pos in joint_state.position]
        
        # 发布JointState消息
        pub.publish(joint_state)
        
        # 打印关节位置（可选）
        # print(joint_state.position)
        
        # 按照设定的频率休眠
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
