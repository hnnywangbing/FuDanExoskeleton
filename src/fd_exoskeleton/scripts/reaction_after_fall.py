import serial
import time
import struct
import crcmod
import can
import threading
import binascii
from sensor_msgs.msg import JointState
from unitree_legged_msgs.msg import MotorCmd, MotorState
from Imu import roll_deg,pitch_deg

#先在命令行众配置 can0
# sudo ip link set can1 up type can bitrate 1000000 dbitrate 5000000 restart-ms 1000 sample-point 0.8 dsample-point 0.75 berr-reporting on fd on
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
        ]
        
        for arbitration_id, data in init_messages:
            message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
            bus.send(message)
            time.sleep(0.3)
        
    except can.CanError:
       print("Failed to send initialization CAN messages")


# 获取关节信息
def send_state_can_messages(bus):
    try:
        # 创建并发送初始化的 CAN 消息
        init_messages = [
            (0x601, []),
            (0x602, []),
            (0x603, []),
            (0x604, []),
        ]
        
        for arbitration_id, data in init_messages:
            print(arbitration_id)
            print(data)
            message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
            bus.send(message)
            print(f"Sent initialization message: {message}")
            time.sleep(0.3)
        
    except can.CanError:
        print("Failed to send initialization CAN messages")  

# 获取速度信息
def send_speed_can_messages(bus):
    try:
        # 创建并发送初始化的 CAN 消息
        init_messages = [
            (0x1, [0x01,0x12,0x02]),
            (0x2, [0x01,0x12,0x02]),
            (0x3, [0x01,0x12,0x02]),
            (0x4, [0x01,0x12,0x02]),
        ]
        
        for arbitration_id, data in init_messages:
            print(arbitration_id)
            print(data)
            message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
            bus.send(message)
            print(f"Sent initialization message: {message}")
            time.sleep(0.3)
        
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


# 接收 CAN 消息的线程函数
def receive_can_message(bus):
    while True:
        # 接收 CAN 消息
        message = bus.recv()  # 阻塞等待消息
        if message:
            print(f"Message received: {message}")
            # 访问消息的各个属性
            arbitration_id=hex(message.arbitration_id)
            hex_str = binascii.hexlify(message.data).decode('utf-8')
            print(f"message: {hex_str}")
            if message.channel== "can1" and arbitration_id=="0x701":
                signed_value=get_position_value(message.data)
                position[3]=round(signed_value/10000,2)
            if message.channel== "can1" and arbitration_id=="0x702":
                signed_value=get_position_value(message.data)
                position[2]=round(signed_value/10000,2)
            if message.channel== "can1" and arbitration_id=="0x703":
                signed_value=get_position_value(message.data)
                position[1]=round(signed_value/10000,2)
            if message.channel== "can1" and arbitration_id=="0x704":
                signed_value=get_position_value(message.data)
                position[0]=round(signed_value/10000,2) 
            if message.channel== "can0" and arbitration_id=="0x701":
                signed_value=get_position_value(message.data)
                position[7]=round(signed_value/10000,2)
            if message.channel== "can0" and arbitration_id=="0x702":
                signed_value=get_position_value(message.data)
                position[6]=round(signed_value/10000,2)
            if message.channel== "can0" and arbitration_id=="0x703":
                signed_value=get_position_value(message.data)
                position[5]=round(signed_value/10000,2)
            if message.channel== "can0" and arbitration_id=="0x704":
                signed_value=get_position_value(message.data)
                position[4]=round(signed_value/10000,2)                
            if message.channel== "can1" and hex_str=="0x101" and hex_str.startswith("0112"):
                signed_value=get_velocity_value(message.data)
                position[3]=round(signed_value*0.02,2)
            if message.channel== "can1" and arbitration_id=="0x102" and hex_str.startswith("0112"):
                signed_value=get_velocity_value(message.data)
                position[2]=round(signed_value*0.02,2)
            if message.channel== "can1" and arbitration_id=="0x103" and hex_str.startswith("0112"):
                signed_value=get_velocity_value(message.data)
                position[1]=round(signed_value*0.02,2)
            if message.channel== "can1" and arbitration_id=="0x104" and hex_str.startswith("0112"):
                signed_value=get_velocity_value(message.data)
                position[0]=round(signed_value*0.02,2)
            if message.channel== "can0" and arbitration_id=="0x101" and hex_str.startswith("0112"):
                signed_value=get_velocity_value(message.data)
                position[7]=round(signed_value*0.02,2)
            if message.channel== "can0" and arbitration_id=="0x102" and hex_str.startswith("0112"):
                signed_value=get_velocity_value(message.data)
                position[6]=round(signed_value*0.02,2)
            if message.channel== "can0" and arbitration_id=="0x103" and hex_str.startswith("0112"):
                signed_value=get_velocity_value(message.data)
                position[5]=round(signed_value*0.02,2)
            if message.channel== "can0" and arbitration_id=="0x104" and hex_str.startswith("0112"):
                signed_value=get_velocity_value(message.data)
                position[4]=round(signed_value*0.02,2)

def main():
    # 设置 CAN 接口，假设使用的是 'can0' 接口
    bus = can.interface.Bus(channel='can1', interface='socketcan', fd=True)
    
    # 创建并启动接收线程
    receive_thread = threading.Thread(target=receive_can_message, args=(bus,))
    receive_thread.daemon = True  # 设置为守护线程，主程序结束时自动退出
    receive_thread.start()

    # 调用发送函数
    send_initial_can_messages(bus)  # 在主线程中执行发送操作

    ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
    request_data = bytes.fromhex('01 03 00 00 00 10 44 06')
    while True:
        send_state_can_messages(bus)
        send_speed_can_messages(bus)
        ser.write(request_data)
        response = ser.read(37)

        if len(response) == 37:
            received_crc = response[-2:]
            calculated_crc = calculate_crc(response[:-2]).to_bytes(2, byteorder='little')
            if received_crc == calculated_crc:
                channels = parse_data(response)
                for i, channel in enumerate(channels):
                    if i+1==1:
                        effort[1]=channel
                    elif i+1==2:
                        effort[6]=channel
                    elif i+1==3:
                        effort[4]=channel
                    elif i+1==4:
                        effort[0]=channel
                    elif i+1==5:
                        effort[5]=channel
                    elif i+1==6:
                        effort[2]=channel
                    elif i+1==7:
                        effort[3]=channel
                    elif i+1==8:
                        effort[7]=channel                   
            else:
                print("CRC校验失败")
                print(calculated_crc.hex().upper())
        time.sleep(0.5)
        break
    roll_deg  
    pitch_deg

if __name__ == "__main__":
    main()
