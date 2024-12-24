# #!/usr/bin/env python3
# import serial
# import time
# import threading
# import keyboard
# from datetime import datetime
# import rospy
# from swarm_ros_bridge.msg import CmdMessage
# def calculate_crc32(data):
#     crc = 0xFFFFFFFF
#     polynomial = 0x04C11DB7
#     for byte in data:
#         crc ^= byte << 24
#         for _ in range(8):
#             if crc & 0x80000000:
#                 crc = (crc << 1) ^ polynomial
#             else:
#                 crc <<= 1
#             crc &= 0xFFFFFFFF
#     return crc

# def send_distance_command(serial_port):
#     # pub = rospy.Publisher('CmdMessage', CmdMessage, queue_size=10)
#     # rospy.init_node('lada_node', anonymous=True)
#     # rate = rospy.Rate(10)  # 100hz
#     distance_command = bytes.fromhex("F5 29 00 00 00 00 00 00 00 00 CB FA AB E7") 
#     while not stop_event.is_set():
#         serial_port.write(distance_command)
#         distance_response = serial_port.read(520)
#         calculated_crc1 = calculate_crc32(distance_response[:-4])
#         received_crc1 = int.from_bytes(distance_response[-4:][::-1], byteorder='big')

#         if calculated_crc1 == received_crc1:
#             data_array = []
#             for i in range(4, 256, 4):
#                 value = int.from_bytes(distance_response[i:i+4][::-1], byteorder='big') / 10
#                 data_array.append(value)
#             print("处理后的数据:", data_array)
#             # 计算 data_array 的平均值
#             average_value = sum(data_array) / len(data_array)

#             # msg = CmdMessage()
#             # msg.time = datetime.now()
#             # msg.cmd = "lida"
#             # msg.value = average_value
#             # pub.publish(msg)
#         else:
#             print("距离数据接收失败，CRC不匹配")
#         time.sleep(0.05)  # 间隔50ms

# def main():
#     global stop_event
#     stop_event = threading.Event()
#     # 设置串口参数，确保端口名正确
#     serial_port = serial.Serial('/dev/ttyUSB0', baudrate=115200, timeout=1)
#     # 上电
#     power_on_command = bytes.fromhex("F5 40 01 00 00 00 00 00 00 00 9C D7 D6 91")
#     serial_port.write(power_on_command)
#     power_on_response = serial_port.read(8)
#     power_on_response_hex = power_on_response.hex().upper()
#     calculated_crc = calculate_crc32(power_on_response[:-4])
#     received_crc = int.from_bytes(power_on_response[-4:][::-1], byteorder='big')
#     if calculated_crc == received_crc:
#         print("上电成功")
#         print("上电返回数据:", power_on_response_hex)
#         # 启动发送距离命令的线程
#         distance_thread = threading.Thread(target=send_distance_command, args=(serial_port,))
#         distance_thread.start()

#         # 等待空格键被按下
#         keyboard.wait('space')
#         stop_event.set()  # 设置停止事件
#         distance_thread.join()  # 等待线程结束
#     else:
#         print("上电失败")
    
#     # 关闭串口
#     serial_port.close()

# if __name__ == "__main__":
#     main()
