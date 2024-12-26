#!/usr/bin/env python3
import rospy
import zmq
from std_msgs.msg import String
from fd_exoskeleton.msg import CmdMessage
import threading
import struct
import time
import traceback
from queue import Queue
# 消息队列
message_queue = Queue()
# 服务端处理客户端请求的函数
def handle_requests(context,rosPub):
    # 创建一个 REP socket
    socket = context.socket(zmq.REP)
    socket.bind("tcp://*:5555")  # 绑定到端口 5555，监听客户端请求
    while not rospy.is_shutdown():
        try:
            # 等待客户端请求
            message_bytes = socket.recv()  # 接收字节串消息
            message = message_bytes.decode('utf-8')  # 将字节串解码为字符串
            rosPub.publish(message)  # 发布字符串消息到ROS主题
            # print(f"Received: {message}")
            # 处理请求（这里只是简单地返回一个确认消息）
            socket.send_string(f"Server:{message}")  # 发送字符串响应
        except Exception as e:
            print(f"Failed to send message: {e}")
            traceback.print_exc()
            break

# 服务端推送消息给订阅客户端的函数
def publish_updates(context):
    # 创建一个 PUB socket
    publisher = context.socket(zmq.PUB)
    publisher.bind("tcp://*:5556")  # 绑定到端口 5556，用于发布消息
    print("publish_updates")
    while not rospy.is_shutdown():
        # 发布消息给所有订阅者
         # 从队列中获取消息 
        try:
            if not message_queue.empty():
                message = message_queue.get()
                # utf8_string = message.decode('utf-8')
                print(f"publishermessage:",message)
                # 发布消息给所有订阅者
                publisher.send_string(message)
        except Exception as e:
            traceback.print_exc()   

def callback(data):
     try:
        print("cmd:",data.cmd)
        if  data.cmd:
            if "Sitdown" in data.cmd:
                message_queue.put("NO_Chair")
            else :
                message_queue.put(data.cmd)
     except Exception as e:
            traceback.print_exc()

# 主函数
def main():
    # 在 ROS 中，节点名称唯一。如果启动两个同名节点，第一个会被关闭。
    # anonymous=True 标志意味着 rospy 将为节点选择一个唯一的名称，允许多个监听器同时运行。
    rospy.init_node('app_server_node', anonymous=True)
    rosPub = rospy.Publisher('AppCmd', String, queue_size=10) # mpc 接收 app端控制命令
    # 订阅 'chatter_recv' 话题，接收 String 类型的消息，并调用回调函数
    rospy.Subscriber('/cmd', CmdMessage, callback)

    context = zmq.Context()

    # 创建并启动处理请求的线程
    request_thread = threading.Thread(target=handle_requests, args=(context,rosPub))
    request_thread.start()

    # 创建并启动发布消息的线程
    publish_thread = threading.Thread(target=publish_updates, args=(context,))
    publish_thread.start()

    # 主线程可以继续执行其他任务，或者简单地等待子线程结束
    request_thread.join()
    publish_thread.join()
    # 使用 rospy.spin() 保持节点活跃，等待消息回调
    rospy.spin()

if __name__ == "__main__":
    main()
