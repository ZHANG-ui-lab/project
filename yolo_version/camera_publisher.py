import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(0)  # 默认相机
        self.timer = self.create_timer(0.03, self.timer_callback)  # 30Hz 帧率

    def timer_callback(self):
        ret, frame = self.cap.read()
        if ret:
            img_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            self.publisher_.publish(img_msg)
            self.get_logger().info('Publishing camera frame')

def main(args=None):
    rclpy.init(args=args)
    camera_pub = CameraPublisher()
    rclpy.spin(camera_pub)
    camera_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()