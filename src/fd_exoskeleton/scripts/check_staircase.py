import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DepthProcessingNode(Node):
    def __init__(self):
        super().__init__('depth_processing_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/depth/image_rect_raw',  # 请根据您的实际情况修改话题名称
            self.depth_image_callback,
            10)
        self.bridge = CvBridge()

    def depth_image_callback(self, msg):
        # 使用cv_bridge将ROS图像消息转换为OpenCV图像
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        
        # 获取图像的分辨率
        height, width = depth_image.shape

        # 过滤1米以内的深度数据
        depth_array = np.array(depth_image, dtype=np.float32) / 1000.0  # 将深度转换为米
        valid_depths = depth_array[depth_array < 1.0]  # 过滤深度大于1米的像素

        # 寻找台阶边缘并计算台阶高度
        step_heights = []
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if depth_array[y, x] < 1.0:
                    # 计算当前像素与周围像素的深度差
                    depth_diff = np.abs(depth_array[y, x] - depth_array[y - 1, x])
                    # 如果深度差大于某个阈值（例如0.05米），则可能是一个台阶边缘
                    if depth_diff > 0.002:
                        step_heights.append(depth_diff)

        # 计算平均台阶高度
        average_step_height = np.mean(step_heights) if step_heights else 0

        self.get_logger().info(f'Average Step Height: {average_step_height * 1000:.2f} mm')

def main(args=None):
    rclpy.init(args=args)
    depth_processing_node = DepthProcessingNode()
    rclpy.spin(depth_processing_node)
    depth_processing_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
