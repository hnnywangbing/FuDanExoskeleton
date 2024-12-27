#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from datetime import datetime
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from fd_exoskeleton.msg import CmdMessage
class DepthProcessingNode:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('realsense_node', anonymous=True)
        self.publisher = rospy.Publisher('/cmd_step_height', CmdMessage, queue_size=10)
        # 创建订阅者
        self.subscription = rospy.Subscriber(
            '/camera/depth/image_rect_raw',  # 请根据您的实际情况修改话题名称
            Image,
            self.depth_image_callback,
            queue_size=10)
        
        # 创建CvBridge对象，用于转换ROS图像消息到OpenCV图像
        self.bridge = CvBridge()

    def convert_rostime_to_int64(self,dt):
        # 将 datetime 对象转换为 rospy.Time 对象
        rostime = rospy.Time.from_sec(float(dt.strftime('%s.%f')))
        # 然后将 rospy.Time 对象转换为 int64
        total_nanoseconds = (rostime.secs * 1000000000) + rostime.nsecs
        return total_nanoseconds

    def depth_image_callback(self, msg):
        # 使用cv_bridge将ROS图像消息转换为OpenCV图像
        try:
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge Error: {e}")
            return
        
        # 获取图像的分辨率
        height, width = depth_image.shape

        # 过滤1米以内的深度数据
        depth_array = np.array(depth_image, dtype=np.float32)  # 将深度转换为米
        valid_depths = depth_array[depth_array < 1000]  # 过滤深度大于1米的像素

        # 寻找台阶边缘并计算台阶高度
        step_heights = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if depth_array[y, x] < 1000:
                    # 计算当前像素与周围像素的深度差
                    depth_diff = np.abs(depth_array[y, x] - depth_array[y - 1, x])
                    # 如果深度差大于某个阈值（例如2毫米），则可能是一个台阶边缘
                    if depth_diff > 2:
                        step_heights.append(depth_diff)

        # 计算平均台阶高度
        average_step_height = np.mean(step_heights) if step_heights else 0

        current_datetime = datetime.now()
        msg = CmdMessage()
        msg.time = self.convert_rostime_to_int64(current_datetime)  
        msg.cmd = "step_height"
        msg.value = average_step_height
        self.publisher.publish(msg)
        rospy.loginfo(f'Average Step Height: {average_step_height:.2f} mm')

    def run(self):
        # 保持节点持续运行
        rospy.spin()

if __name__ == '__main__':
    try:
        # 创建节点对象并运行
        node = DepthProcessingNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
