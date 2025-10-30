import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO  

class YoloV8Detector(Node):
    def __init__(self):
        super().__init__('yolo_v8_detector')
        # 订阅相机图像话题
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10  # 队列长度
        )
        self.bridge = CvBridge()  # 用于 ROS 图像与 OpenCV 图像转换
        
        # 加载 YOLOv8 模型
        self.model = YOLO('/home/zmy/ros2_yolo_ws/src/yolo_vision/yolo_vision/mine_box.pt')  
        
        # 指定设备（CPU ）
        
        self.get_logger().info('YOLOv8 模型加载完成，开始接收图像...')

    def image_callback(self, msg):
        try:
            # 将 ROS Image 消息转换为 OpenCV 格式（BGR8）
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'图像转换失败: {str(e)}')
            return
        
        # YOLOv8 推理（返回结果对象）
        results = self.model(cv_image)  # 默认为检测任务，若为分割模型会自动处理
        
        # 可视化结果（在图像上绘制检测框、类别、置信度）
        # 对于检测模型：results[0].plot() 直接返回带标注的图像
        # 对于分割模型：会同时绘制分割掩码
        annotated_image = results[0].plot()
        
        # 显示结果图像
        cv2.imshow('YOLOv8 Detection Results', annotated_image)
        cv2.waitKey(1)  # 刷新窗口
        
        # 打印检测到的目标数量
        self.get_logger().info(f'检测到 {len(results[0].boxes)} 个目标')

def main(args=None):
    rclpy.init(args=args)
    detector = YoloV8Detector()
    try:
        rclpy.spin(detector)  # 持续运行节点
    except KeyboardInterrupt:
        detector.get_logger().info('用户中断，退出节点')
    finally:
        detector.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()  # 关闭所有 OpenCV 窗口

if __name__ == '__main__':
    main()