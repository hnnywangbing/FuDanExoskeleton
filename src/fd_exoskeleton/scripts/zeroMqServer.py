#!/usr/bin/env python3
import rospy
import zmq
from std_msgs.msg import String
from fd_exoskeleton.msg import CmdMessage
 
receive_data(){
    while tr.is_alive() :
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
}



def callback(data):
    # 使用 f-string 格式化字符串
    if "Sitdown" in data.cmd:
        socket.send_string("NO_Chair")

  
if __name__ == '__main__':
    # 创建 ZeroMQ Context
    context = zmq.Context()
    # 创建一个 REP 套接字
    socket = context.socket(zmq.REP)
    # 绑定套接字到端口
    socket.bind("tcp://*:5555")
    # 在 ROS 中，节点名称唯一。如果启动两个同名节点，第一个会被关闭。
    # anonymous=True 标志意味着 rospy 将为节点选择一个唯一的名称，允许多个监听器同时运行。
    rospy.init_node('AppServer', anonymous=True)
    pub = rospy.Publisher('AppCmd', String, queue_size=10) # mpc 接收 app端控制命令
    # 订阅 'chatter_recv' 话题，接收 String 类型的消息，并调用回调函数
    rospy.Subscriber('/cmd_recv', CmdMessage, callback)
    tr = threading.Thread(target=receive_data)
    tr.start()
    while True:
        try:
            if tr.is_alive():
                time.sleep(1)
            else:
                break
        except(KeyboardInterrupt, SystemExit):
            break
    # spin() 方法会保持 Python 进程运行，直到节点被停止
    rospy.spin()
