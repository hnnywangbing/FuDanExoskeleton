#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import zmq

pub = rospy.Publisher('/appControl', String, queue_size=10)
rospy.init_node('AppServer', anonymous=True)
# 创建 ZeroMQ Context
context = zmq.Context()
# 创建一个 REP 套接字
socket = context.socket(zmq.REP)
# 绑定套接字到端口
socket.bind("tcp://*:5555")

while True:
    try:
        # 等待客户端请求
        message_bytes = socket.recv()  # 接收字节串消息
        message = message_bytes.decode('utf-8')  # 将字节串解码为字符串
        pub.publish(message)  # 发布字符串消息到ROS主题
        # print(f"Received: {message}")
        # 处理请求（这里只是简单地返回一个确认消息）
        socket.send_string(f"Server:{message}")  # 发送字符串响应
    except Exception as e:
        traceback.print_exc()
        break