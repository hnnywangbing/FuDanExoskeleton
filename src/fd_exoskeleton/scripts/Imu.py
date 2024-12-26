#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import serial  # 导入模块
import threading
import struct
import time
import math
from fd_exoskeleton.msg import CmdMessage
import traceback
from serial import EIGHTBITS, PARITY_NONE, STOPBITS_ONE
from datetime import datetime

# 固定的串口参数
PORT = '/dev/ttyUSB0'  # 串口端口
BAUDRATE = 921600      # 波特率
TIMEOUT = 20           # 超时时间

PI = 3.141592653589793
DEG_TO_RAD = 0.017453292519943295
FRAME_HEAD = str('fc')
FRAME_END = str('fd')
TYPE_IMU = str('40')
TYPE_AHRS = str('41')
TYPE_INSGPS = str('42')
TYPE_GEODETIC_POS = str('5c')
TYPE_GROUND = str('f0')
TYPE_SYS_STATE = str('50')
TYPE_BODY_ACCELERATION = str('62')
TYPE_ACCELERATION = str('61')
TYPE_MSG_BODY_VEL = str('60')
IMU_LEN = str('38')  # //56
AHRS_LEN = str('30')  # //48
INSGPS_LEN = str('48')  # //72
GEODETIC_POS_LEN = str('20')  # //32
SYS_STATE_LEN = str('64')  # // 100
BODY_ACCELERATION_LEN = str('10') #// 16
ACCELERATION_LEN = str('0c')  # 12

isrun = True

# 将 datetime 对象转换为 int64 的函数
def convert_rostime_to_int64(dt):
    rostime = rospy.Time.from_sec(float(dt.strftime('%s.%f')))
    total_nanoseconds = (rostime.secs * 1000000000) + rostime.nsecs
    return total_nanoseconds

# 接收数据线程
def receive_data(publisher):
    try:
        serial_ = serial.Serial(port=PORT, baudrate=BAUDRATE, bytesize=EIGHTBITS, parity=PARITY_NONE,
                                stopbits=STOPBITS_ONE, timeout=TIMEOUT)
        print("baud rates = " + str(serial_.baudrate))
    except:
        print("error: unable to open port.")
        exit(1)

    # 循环读取数据
    while serial_.isOpen():
        if not threading.main_thread().is_alive():
            print('done')
            break

        check_head = serial_.read().hex()
        # 校验帧头
        if check_head != FRAME_HEAD:
            continue
        head_type = serial_.read().hex()
        # 校验数据类型
        if head_type not in [TYPE_IMU, TYPE_AHRS, TYPE_INSGPS, TYPE_GEODETIC_POS, TYPE_SYS_STATE, TYPE_GROUND,
                             TYPE_BODY_ACCELERATION, TYPE_ACCELERATION, TYPE_MSG_BODY_VEL]:
            continue
        check_len = serial_.read().hex()
        # 校验数据类型的长度
        if head_type == TYPE_IMU and check_len != IMU_LEN:
            continue
        elif head_type == TYPE_AHRS and check_len != AHRS_LEN:
            continue
        elif head_type == TYPE_INSGPS and check_len != INSGPS_LEN:
            continue
        elif head_type == TYPE_GEODETIC_POS and check_len != GEODETIC_POS_LEN:
            continue
        elif head_type == TYPE_SYS_STATE and check_len != SYS_STATE_LEN:
            continue
        elif head_type == TYPE_BODY_ACCELERATION and check_len != BODY_ACCELERATION_LEN:
            continue
        elif head_type == TYPE_ACCELERATION and check_len != ACCELERATION_LEN:
            continue
        elif head_type == TYPE_MSG_BODY_VEL and check_len != ACCELERATION_LEN:
            continue

        # 读取并解析IMU数据
        if head_type == TYPE_IMU:
            data_s = serial_.read(int(IMU_LEN, 16))
            IMU_DATA = struct.unpack('12f ii', data_s[0:56])
            # 解析IMU数据（此处省略打印语句）

        # 读取并解析AHRS数据
        elif head_type == TYPE_AHRS:
            data_s = serial_.read(int(AHRS_LEN, 16))
            AHRS_DATA = struct.unpack('10f ii', data_s[0:48])
            roll_deg = abs(AHRS_DATA[3] * (180 / math.pi))
            pitch_deg = abs(AHRS_DATA[4] * (180 / math.pi))

            # 判断是否摔倒
            if roll_deg > 30:
                current_datetime = datetime.now()
                msg = CmdMessage()
                msg.time = convert_rostime_to_int64(current_datetime)
                msg.cmd = "fall-roll"
                msg.value = roll_deg
                publisher.publish(msg)
                print("Roll(rad) : ", roll_deg)
                print("Pitch(rad) : ", pitch_deg)
            elif pitch_deg > 30:
                current_datetime = datetime.now()
                msg = CmdMessage()
                msg.time = convert_rostime_to_int64(current_datetime)
                msg.cmd = "fall-Pitch"
                msg.value = pitch_deg
                publisher.publish(msg)
                print("Roll(rad) : ", roll_deg)
                print("Pitch(rad) : ", pitch_deg)

        # 其他数据类型的解析（省略）

# 寻找串口
# def find_serial():
#     port_list = list(serial.tools.list_ports.comports())
#     for port in port_list:
#         if port.device == PORT:
#             return True
#     return False

# def open_port():
#     if find_serial():
#         print("find this port : " + PORT)
#     else:
#         print("error: unable to find this port : " + PORT)
#         exit(1)

def main():
    publisher = rospy.Publisher('/cmd', CmdMessage, queue_size=10)
    rospy.init_node('imu_node', anonymous=True)
    # open_port()  # 打开串口
    tr = threading.Thread(target=receive_data, args=(publisher,))
    tr.daemon = True  # 将线程设为守护线程
    tr.start()

    rospy.spin()  # 保持节点活跃，等待消息回调

if __name__ == "__main__":
    main()
