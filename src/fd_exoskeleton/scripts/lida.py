#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import serial
import time
import threading
import keyboard
from datetime import datetime
import rospy
import traceback
from fd_exoskeleton.msg import CmdMessage
import numpy as np
def convert_rostime_to_int64(dt):
      # 将 datetime 对象转换为 rospy.Time 对象
    rostime = rospy.Time.from_sec(float(dt.strftime('%s.%f')))
    # 然后将 rospy.Time 对象转换为 int64
    total_nanoseconds = (rostime.secs * 1000000000) + rostime.nsecs
    return total_nanoseconds
  
def calculate_crc32(data):
    crc = 0xFFFFFFFF
    polynomial = 0x04C11DB7
    for byte in data:
        crc ^= byte << 24
        for _ in range(8):
            if crc & 0x80000000:
                crc = (crc << 1) ^ polynomial
            else:
                crc <<= 1
            crc &= 0xFFFFFFFF
    return crc

def send_distance_command(serial_port,publisher):
    # pub = rospy.Publisher('CmdMessage', CmdMessage, queue_size=10)
    # rospy.init_node('lada_node', anonymous=True)
    # rate = rospy.Rate(10)  # 100hz
    distance_command = bytes.fromhex("F5 29 00 00 00 00 00 00 00 00 CB FA AB E7") 
    serial_port.write(distance_command)
    distance_response = serial_port.read(520)
    calculated_crc1 = calculate_crc32(distance_response[:-4])
    received_crc1 = int.from_bytes(distance_response[-4:][::-1], byteorder='big')
    if calculated_crc1 == received_crc1:
        data_array = []
        for i in range(4, 256, 4):
            value = int.from_bytes(distance_response[i:i+4][::-1], byteorder='big') / 10
            data_array.append(value)
        # print("处理后的数据:", data_array,' len:',len(data_array))
        # 计算 data_array 最小20个数的平均值
        # 使用 NumPy 的 partition 函数获取最小的20个数
        twenty_smallest_values = np.partition(data_array, 19)[:20]
        # 保留两位小数
        
        # 计算这20个最小值的平均值
        average_value = np.mean(twenty_smallest_values)
        average_value = np.round(average_value, 2)
        print("平均值:", average_value)
        current_datetime = datetime.now()
        msg = CmdMessage()
        msg.time = convert_rostime_to_int64(current_datetime)  
        msg.cmd = "lida"
        msg.value = average_value
        publisher.publish(msg)
    else:
        print("距离数据接收失败，CRC不匹配")

def main():

    publisher = rospy.Publisher('cmd', CmdMessage, queue_size=10)
    rospy.init_node('lida_node', anonymous=True)
    rate = rospy.Rate(20)  # 50hz 
    # 设置串口参数，确保端口名正确
    serial_port = serial.Serial('/dev/ttyUSB1', baudrate=115200, timeout=1)
    # 上电
    power_on_command = bytes.fromhex("F5 40 01 00 00 00 00 00 00 00 9C D7 D6 91")
    serial_port.write(power_on_command)
    power_on_response = serial_port.read(8)
    power_on_response_hex = power_on_response.hex().upper()
    calculated_crc = calculate_crc32(power_on_response[:-4])
    received_crc = int.from_bytes(power_on_response[-4:][::-1], byteorder='big')
    while not rospy.is_shutdown():
        if calculated_crc == received_crc:
            try:
              send_distance_command(serial_port,publisher)
            except Exception as e:
                traceback.print_exc()
                break
        else:
            print("上电失败")
    
    # 关闭串口
    serial_port.close()

if __name__ == "__main__":
    main()
