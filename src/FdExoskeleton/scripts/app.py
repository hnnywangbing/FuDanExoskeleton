#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from FdExoskeleton.msg import CmdMessage
from std_msgs.msg import String
moveCmd = rospy.Publisher('moveCmd', String, queue_size=10)
modeCmd = rospy.Publisher('modeCmd', String, queue_size=10)


def callback(data):
    # 使用 f-string 格式化字符串
    # rospy.loginfo(f'cmd: {data.cmd} value{data.value}')
    paseCmdData()

def callbackCmdInfo(CmdMessage):
    # 使用 f-string 格式化字符串
    rospy.loginfo(f'cmd: {data.cmd}')
    # paseCmdData()

def paseCmdData(data):
    print("AppServer",data)
    moveCmd.publish(data)
    # if('Forward' in data.cmd){

    # } elif ('Left' in data.cmd){

    # }elif ('Right' in data.cmd){
        
    # }elif ('Backward' in data.cmd){
        
    # }elif ('Sitdown' in data.cmd){
        
    # }elif ('UpStairs' in data.cmd){
        
    # }elif ('DownStairs' in data.cmd){
        
    # }elif ('OnRoad' in data.cmd){
        
    # }
    

 

def listener():
    # 在 ROS 中，节点名称唯一。如果启动两个同名节点，第一个会被关闭。
    # anonymous=True 标志意味着 rospy 将为节点选择一个唯一的名称，允许多个监听器同时运行。
    rospy.init_node('AppServer', anonymous=True)    

    # 订阅 'chatter_recv' 话题，接收 String 类型的消息，并调用回调函数
    rospy.Subscriber('string_recv', String, callback)
    rospy.Subscriber('AppCmd', CmdMessage, callbackCmdInfo)

    # spin() 方法会保持 Python 进程运行，直到节点被停止
    rospy.spin()

if __name__ == '__main__':
    listener()
