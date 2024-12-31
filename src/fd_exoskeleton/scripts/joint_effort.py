#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import time
import struct
import crcmod
import can
import math
import threading
import binascii
import rospy
from sensor_msgs.msg import JointState
from unitree_legged_msgs.msg import MotorCmd, MotorState

#先在命令行众配置 can0
# sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 restart-ms 1000 sample-point 0.8 dsample-point 0.75 berr-reporting on fd on
# sudo ip link set can0 up type can bitrate 1000000 dbitrate 5000000 restart-ms 1000 sample-point 0.8 dsample-point 0.75 berr-reporting on fd on

# 参考  https://zhuanlan.zhihu.com/p/11384593589

name = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7', 'joint8']

effort=[0]*8
position=[0]*8 
velocity=[0]*8

def calculate_crc(data):
    crc16 = crcmod.predefined.mkCrcFun('modbus')
    return crc16(data)


def parse_data(data):
    channels = []
    for i in range(3, len(data) - 2, 4):
        float_data = struct.unpack('>f', data[i:i + 4])[0]
        channels.append(float_data)
    return channels

def signed_int_to_32bit_hex(number):
    # 限制整数在32位有符号整数范围内
    # 对于正数，这什么也不做；对于负数，它利用补码表示法
    number = number & 0xFFFFFFFF

    # 使用格式化字符串将整数转换为32位宽度的16进制字符串
    # ':08x' 表示宽度为8，不足部分用0填充，小写16进制
    return f"{number:08x}"    

# 初始化的 CAN 消息发送函数
def send_initial_can_messages(bus):
    try:
        # 创建并发送初始化的 CAN 消息
        init_messages = [
            (0x1, [0x02, 0x49, 0x00]),
            (0x2, [0x02, 0x49, 0x00]),
            (0x3, [0x02, 0x49, 0x00]),
            (0x4, [0x02, 0x49, 0x00]),
            (0x1, [0x02, 0x30, 0x01]),
            (0x2, [0x02, 0x30, 0x01]),
            (0x3, [0x02, 0x30, 0x01]),
            (0x4, [0x02, 0x30, 0x01]),
        ]
        
        for arbitration_id, data in init_messages:
            message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
            bus.send(message)
            message=bus.recv()
            if message:
                print(f"Message received: {message}")
            time.sleep(0.3)
        
    except can.CanError:
       print("Failed to send initialization CAN messages")


# 获取关节信息
def send_state_can_messages(id,bus):
    try:
        # 创建并发送初始化的 CAN 消息
        init_messages =[(0x600+id, []),]
        for arbitration_id, data in init_messages:
            message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
            bus.send(message)
            message=bus.recv()
            if message:
                print(f"Message received: {message}")
                signed_value=get_position_value(message.data)
                print(f"signed_position_value: {signed_value}")
                return round(signed_value/10000,2)
        time.sleep(0.01)
    except can.CanError:
        print("Failed to send initialization CAN messages")  

# 获取速度信息
def send_speed_can_messages(id,bus):
    try:
        # 创建并发送初始化的 CAN 消息
        init_messages =[(0x0+id, [0x01,0x12,0x02]),]
        for arbitration_id, data in init_messages:
            message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
            bus.send(message)
            message=bus.recv()
            if message:
                print(f"Message received: {message}")
                signed_value=get_velocity_value(message.data)
                print(f"signed_velocity_value: {signed_value}")
                return signed_value
        time.sleep(0.01)
    except can.CanError:
        print("Failed to send initialization CAN messages")    

def get_position_value(data):
    extracted_bytes = data[8:12]
    reversed_bytes = extracted_bytes[::-1]
    hex_str = ''.join(f'{byte:02x}' for byte in reversed_bytes)               
    bytes_data = bytes.fromhex(hex_str)
    signed_value = struct.unpack('>i', bytes_data)[0]
    return signed_value

def get_velocity_value(data):
    extracted_bytes = data[2:6]
    reversed_bytes = extracted_bytes[::-1]
    hex_str = ''.join(f'{byte:02x}' for byte in reversed_bytes)                
    bytes_data = bytes.fromhex(hex_str)
    signed_value = struct.unpack('>i', bytes_data)[0]
    return signed_value



def listenerR(bus):
    print("启动了")
    # 订阅 'csv_output' 话题，消息类型为 Float64MultiArray
    rospy.Subscriber('/exo_humanoid_gazebo/R_calf_controller/command', MotorCmd, callbackR2, callback_args=bus)
    #rospy.Subscriber('/exo_humanoid_gazebo/R_hip2_controller/command', MotorCmd, callbackR3, callback_args=bus)
    #rospy.Subscriber('/exo_humanoid_gazebo/R_hip_controller/command', MotorCmd, callbackR4, callback_args=bus)
    rospy.Subscriber('/exo_humanoid_gazebo/R_toe_controller/command', MotorCmd, callbackR1,callback_args=bus)

def listenerL(bus1):
    rospy.Subscriber('/exo_humanoid_gazebo/L_calf_controller/command', MotorCmd, callbackL2, callback_args=bus1)
    #rospy.Subscriber('/exo_humanoid_gazebo/L_hip2_controller/command', MotorCmd, callbackL3, callback_args=bus1)
    #rospy.Subscriber('/exo_humanoid_gazebo/L_hip_controller/command', MotorCmd, callbackL4, callback_args=bus1)
    rospy.Subscriber('/exo_humanoid_gazebo/L_toe_controller/command', MotorCmd, callbackL1, callback_args=bus1)    

#
def talker(bus,bus1):
    # 调用发送函数
    motor_state = MotorState()
    R_calf_controller = rospy.Publisher('/exo_humanoid_gazebo/R_calf_controller/state', MotorState, queue_size=10)
    R_hip_controller = rospy.Publisher('/exo_humanoid_gazebo/R_hip_controller/state', MotorState, queue_size=10)
    R_hip2_controller = rospy.Publisher('/exo_humanoid_gazebo/R_hip2_controller/state', MotorState, queue_size=10)
    R_toe_controller = rospy.Publisher('/exo_humanoid_gazebo/R_toe_controller/state', MotorState, queue_size=10)
    L_calf_controller = rospy.Publisher('/exo_humanoid_gazebo/L_calf_controller/state', MotorState, queue_size=10)
    L_hip_controller = rospy.Publisher('/exo_humanoid_gazebo/L_hip_controller/state', MotorState, queue_size=10)
    L_hip2_controller = rospy.Publisher('/exo_humanoid_gazebo/L_hip2_controller/state', MotorState, queue_size=10)
    L_toe_controller = rospy.Publisher('/exo_humanoid_gazebo/L_toe_controller/state', MotorState, queue_size=10)
    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    request_data = bytes.fromhex('01 03 00 00 00 10 44 06')
    try:
        while True:
            ser.write(request_data)
            response = ser.read(37)
            if len(response) == 37:
                received_crc = response[-2:]
                calculated_crc = calculate_crc(response[:-2]).to_bytes(2, byteorder='little')
                if received_crc == calculated_crc:
                    channels = parse_data(response)
                    for i, channel in enumerate(channels):
                        print(f"通道{i + 1}：{channel}")
                        if i+1==1:
                            position_value=send_state_can_messages(3,bus)
                            speed_value=send_speed_can_messages(3,bus)
                            effort[1]=channel
                            motor_state.q=position_value
                            motor_state.dq=speed_value
                            motor_state.tauEst=channel
                            # 发布消息
                            R_hip2_controller.publish(motor_state)
                        elif i+1==2:
                            position_value=send_state_can_messages(2,bus1)
                            speed_value=send_speed_can_messages(2,bus1)
                            effort[6]=channel
                            motor_state.q=position_value
                            motor_state.dq=speed_value
                            motor_state.tauEst=channel
                            # 发布消息
                            L_calf_controller.publish(motor_state)
                        elif i+1==3:
                            position_value=send_state_can_messages(4,bus)
                            speed_value=send_speed_can_messages(4,bus)
                            effort[4]=channel
                            motor_state.q=position_value
                            motor_state.dq=speed_value
                            motor_state.tauEst=channel
                            # 发布消息
                            R_hip_controller.publish(motor_state)
                        elif i+1==4:
                            position_value=send_state_can_messages(4,bus1)
                            speed_value=send_speed_can_messages(4,bus1)
                            effort[0]=channel
                            motor_state.q=position_value
                            motor_state.dq=speed_value
                            motor_state.tauEst=channel
                            # 发布消息
                            L_hip_controller.publish(motor_state)
                        elif i+1==5:
                            position_value=send_state_can_messages(3,bus1)
                            speed_value=send_speed_can_messages(3,bus1)
                            effort[5]=channel
                            motor_state.q=position_value
                            motor_state.dq=speed_value
                            motor_state.tauEst=channel
                            # 发布消息
                            L_hip2_controller.publish(motor_state)
                        elif i+1==6:
                            position_value=send_state_can_messages(2,bus)
                            speed_value=send_speed_can_messages(2,bus)
                            effort[2]=channel
                            motor_state.q=position_value
                            motor_state.dq=speed_value
                            motor_state.tauEst=channel
                            # 发布消息
                            R_calf_controller.publish(motor_state)
                        elif i+1==7:
                            position_value=send_state_can_messages(1,bus)
                            speed_value=send_speed_can_messages(1,bus)
                            effort[3]=channel
                            motor_state.q=position_value
                            motor_state.dq=speed_value
                            motor_state.tauEst=channel
                            # 发布消息
                            R_toe_controller.publish(motor_state)
                        elif i+1==8:
                            position_value=send_state_can_messages(1,bus1)
                            speed_value=send_speed_can_messages(1,bus1)
                            effort[7]=channel
                            motor_state.q=position_value
                            motor_state.dq=speed_value
                            motor_state.tauEst=channel
                            # 发布消息
                            L_toe_controller.publish(motor_state)                   
                else:
                    print("CRC校验失败")
                    print(calculated_crc.hex().upper())
            time.sleep(0.1)
    except Exception as e:
        rospy.logerr(f"talker error:{e}")   

# 订阅者回调函数，接收来自 'csv_output' 的数据并根据需要发送 CAN 消息
def callbackR1(data, bus):
    rospy.loginfo(f"r1接收到的 9 维数组：{data.tau}")
    try:
        angles=data.q %360
        angle_in_radians = math.radians(angles)
        print(f"角度{angles}")
        # 计算正弦值
        sin_value = math.sin(angle_in_radians)
        print(f"余弦值{sin_value}")
        ele=0
        if(angles <180):
            ele = round((data.tau)*180*-11-2700*sin_value)
        else:
            ele = round((data.tau)*180 *-11-3000*sin_value)
        
        print(f"电流值{ele}")
        if abs(data.tau)<99999:
            ele_str = signed_int_to_32bit_hex(int(-data.tau))
            hex_number = int(ele_str, 16)
            temp = [0,0,0,0]    
            temp[0] = hex_number & 0xFF
            temp[1] = (hex_number >> 8) & 0xFF
            temp[2] = (hex_number >> 16) & 0xFF
            temp[3] = (hex_number >> 24) & 0xFF
            print(f"下发数据{temp}")
            send_can_message(bus, temp,0x401)
    except Exception as e:
        rospy.logerr(f"处理数据时出错：{e}")

def callbackR2(data, bus):
    rospy.loginfo(f"r2接收到的 9 维数组：{data.tau}")
    try:
        angles=data.q %360
        angle_in_radians = math.radians(angles)
        print(f"角度{angles}")
        # 计算正弦值
        sin_value = math.sin(angle_in_radians)
        print(f"余弦值{sin_value}")
        ele=0
        if(angles <180):
            ele = round((data.tau)*180*-10-1500*sin_value)
        else:
            ele = round((data.tau)*180*-10-1000*sin_value)
        ele_str = signed_int_to_32bit_hex(ele)
        print(f"电流值{data.tau}")
        if abs(data.tau)<99999:
            ele_str = signed_int_to_32bit_hex(int(-data.tau*0.3))
            hex_number = int(ele_str, 16)
            temp = [0,0,0,0]    
            temp[0] = hex_number & 0xFF
            temp[1] = (hex_number >> 8) & 0xFF
            temp[2] = (hex_number >> 16) & 0xFF
            temp[3] = (hex_number >> 24) & 0xFF
            print(f"下发数据{temp}")
            send_can_message(bus, temp,0x402)         
    except Exception as e:
        rospy.logerr(f"处理数据时出错：{e}")   

def callbackR3(data, bus):
    rospy.loginfo(f"r3接收到的 9 维数组：{data.tau}")
    try:
        angles=data.q %360
        angle_in_radians = math.radians(angles)
        print(f"角度{angles}")
        # 计算正弦值
        sin_value = math.sin(angle_in_radians)
        print(f"余弦值{sin_value}")
        ele=0
        if(angles <180):
            ele = round((data.tau)*180*-3-1000*sin_value)
        else:
            ele = round((data.tau)*180*-3-1000*sin_value)
        ele_str = signed_int_to_32bit_hex(ele)
        print(f"电流值{ele}")
        if abs(data.tau)<99999:
            ele_str = signed_int_to_32bit_hex(int(data.tau))
            hex_number = int(ele_str, 16)
            temp = [0,0,0,0]    
            temp[0] = hex_number & 0xFF
            temp[1] = (hex_number >> 8) & 0xFF
            temp[2] = (hex_number >> 16) & 0xFF
            temp[3] = (hex_number >> 24) & 0xFF
            print(f"下发数据{temp}")
            send_can_message(bus, temp,0x403)                
    except Exception as e:
        rospy.logerr(f"处理数据时出错：{e}")  

def callbackR4(data, bus):
    rospy.loginfo(f"r4接收到的 9 维数组：{data.tau}")
    try:
        angles=data.q %360
        angle_in_radians = math.radians(angles)
        print(f"角度{angles}")
        # 计算正弦值
        sin_value = math.sin(angle_in_radians)
        print(f"余弦值{sin_value}")
        ele=0
        if(angles <180):
            ele = round((data.tau)*180*-7-2000*sin_value)
        else:
            ele = round((data.tau)*180*-13-2000*sin_value)
        ele_str = signed_int_to_32bit_hex(ele)
        print(f"电流值{ele}")
        if abs(data.tau)<99999:
            ele_str = signed_int_to_32bit_hex(int(data.tau))
            hex_number = int(ele_str, 16)
            temp = [0,0,0,0]    
            temp[0] = hex_number & 0xFF
            temp[1] = (hex_number >> 8) & 0xFF
            temp[2] = (hex_number >> 16) & 0xFF
            temp[3] = (hex_number >> 24) & 0xFF
            print(f"下发数据{temp}")
            #send_can_message(bus, temp,0x404)               
    except Exception as e:
        rospy.logerr(f"处理数据时出错：{e}")                   

def callbackL1(data, bus1):
    rospy.loginfo(f"l1接收到的 9 维数组：{data.tau}")
    try:
        angles=data.q %360
        angle_in_radians = math.radians(angles)
        print(f"角度{angles}")
        # 计算正弦值
        sin_value = math.sin(angle_in_radians)
        print(f"余弦值{sin_value}")
        ele=0
        if(angles <180):
            ele = round((data.tau)*180*-11-3600*sin_value)
        else:
            ele = round((data.tau)*180 *-11-3600*sin_value)
        ele_str = signed_int_to_32bit_hex(ele)
        print(ele_str)
        if abs(data.tau)<99999:
            ele_str = signed_int_to_32bit_hex(int(-data.tau))
            hex_number = int(ele_str, 16)
            temp = [0,0,0,0]    
            temp[0] = hex_number & 0xFF
            temp[1] = (hex_number >> 8) & 0xFF
            temp[2] = (hex_number >> 16) & 0xFF
            temp[3] = (hex_number >> 24) & 0xFF
            print(f"下发数据{temp}")
            send_can_message(bus1, temp,0x401)             
    except Exception as e:
        rospy.logerr(f"处理数据时出错：{e}")

def callbackL2(data, bus1):
    rospy.loginfo(f"l2接收到的 9 维数组：{data.tau}")
    try:
        angles=data.q %360
        angle_in_radians = math.radians(angles)
        print(f"角度{angles}")
        # 计算正弦值
        sin_value = math.sin(angle_in_radians)
        print(f"余弦值{sin_value}")
        ele=0
        if(angles <180):
            ele = round((data.tau)*180*-12-2500*sin_value)
        else:
            ele = round((data.tau)*180 *-12-2400*sin_value)
        ele_str = signed_int_to_32bit_hex(ele)
        print(ele_str)
        if abs(data.tau)<99999:
            ele_str = signed_int_to_32bit_hex(int(-data.tau*0.3))
            hex_number = int(ele_str, 16)
            temp = [0,0,0,0]    
            temp[0] = hex_number & 0xFF
            temp[1] = (hex_number >> 8) & 0xFF
            temp[2] = (hex_number >> 16) & 0xFF
            temp[3] = (hex_number >> 24) & 0xFF
            print(f"下发数据{temp}")
            send_can_message(bus1, temp,0x402)               
    except Exception as e:
        rospy.logerr(f"处理数据时出错：{e}")

def callbackL3(data, bus1):
    rospy.loginfo(f"l3接收到的 9 维数组：{data.tau}")
    try:
        angles=data.q %360
        angle_in_radians = math.radians(angles)
        print(f"角度{angles}")
        # 计算正弦值
        sin_value = math.sin(angle_in_radians)
        print(f"余弦值{sin_value}")
        ele=0
        if(angles <180):
            ele = round((data.tau)*180*-12-3000*sin_value)
        else:
            ele = round((data.tau)*180 *-12-2400*sin_value)
        ele_str = signed_int_to_32bit_hex(ele)
        print(ele_str)
        if abs(data.tau)<99999:
            ele_str = signed_int_to_32bit_hex(int(data.tau))
            hex_number = int(ele_str, 16)
            temp = [0,0,0,0]    
            temp[0] = hex_number & 0xFF
            temp[1] = (hex_number >> 8) & 0xFF
            temp[2] = (hex_number >> 16) & 0xFF
            temp[3] = (hex_number >> 24) & 0xFF
            print(f"下发数据{temp}")
            send_can_message(bus1, temp,0x403)               
    except Exception as e:
        rospy.logerr(f"处理数据时出错：{e}")

def callbackL4(data, bus1):
    rospy.loginfo(f"l4接收到的 9 维数组：{data.tau}")
    try:
        angles=data.q %360
        angle_in_radians = math.radians(angles)
        print(f"角度{angles}")
        # 计算正弦值
        sin_value = math.sin(angle_in_radians)
        print(f"余弦值{sin_value}")
        ele=0
        if(angles <180):
            ele = round((data.tau)*180*-11-2300*sin_value)
        else:
            ele = round((data.tau)*180 *-12-2200*sin_value)
        ele_str = signed_int_to_32bit_hex(ele)
        print(ele_str)
        if abs(data.tau)<99999:
            ele_str = signed_int_to_32bit_hex(int(-data.tau))
            hex_number = int(ele_str, 16)
            temp = [0,0,0,0]    
            temp[0] = hex_number & 0xFF
            temp[1] = (hex_number >> 8) & 0xFF
            temp[2] = (hex_number >> 16) & 0xFF
            temp[3] = (hex_number >> 24) & 0xFF
            print(f"下发数据{temp}")
            #send_can_message(bus1, temp,0x404)               
    except Exception as e:
        rospy.logerr(f"处理数据时出错：{e}")        

# 发送 CAN 消息的函数（在主线程中运行）
def send_can_message(bus, message_data,canid):
    try:
        rospy.loginfo(f"canid: {canid}")
        # 创建 CAN 消息
        message = can.Message(arbitration_id=canid, data=message_data, is_extended_id=False)
        # 发送消息
        bus.send(message)
        rospy.loginfo(f"Message sent: {message}")
        message = bus.recv()  # 阻塞等待消息
        if message:
            print(f"Message received: {message}")
    except can.CanError:
        rospy.logerr("Message failed to send")


def main():
    rospy.init_node('csv_force_control', anonymous=True)
    # 设置 CAN 接口，假设使用的是 'can0' 接口
    bus = can.interface.Bus(channel='can1', interface='socketcan', fd=True)
    bus1 = can.interface.Bus(channel='can0', interface='socketcan', fd=True)

    send_initial_can_messages(bus)  # 在主线程中执行发送操作
    send_initial_can_messages(bus1)  # 在主线程中执行发送操作

     # 创建并启动接收线程
    listenerR_thread = threading.Thread(target=listenerR, args=(bus,))
    listenerR_thread.start()

    listenerL_thread = threading.Thread(target=listenerL, args=(bus1,))
    listenerL_thread.start()
    
    talker_thread = threading.Thread(target=talker, args=(bus,bus1,))
    talker_thread.start()

   

    
    

    rospy.spin()  # 保持节点活跃，等待消息回调
    



if __name__ == "__main__":
    main()
