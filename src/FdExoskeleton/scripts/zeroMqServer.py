#!/usr/bin/env python3

import zmq

# 创建 ZeroMQ Context
context = zmq.Context()

# 创建一个 REP 套接字
socket = context.socket(zmq.REP)
# 绑定套接字到端口
socket.bind("tcp://*:5555")

while True:
    # 等待客户端请求
    message = socket.recv()
    print(f"Received request: {message}")

    # 处理请求（这里只是简单地返回一个确认消息）
    socket.send(b"World")
