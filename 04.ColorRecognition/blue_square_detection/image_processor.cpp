#include "rclcpp/rclcpp.hpp"         // ROS 2 C++ 的核心库，用于创建节点、发布/订阅等
#include "sensor_msgs/msg/image.hpp" // ROS 2 的图像消息类型
#include <cv_bridge/cv_bridge.h>     // 用于在 ROS 图像消息和 OpenCV 图像之间进行转换
#include <opencv2/opencv.hpp>        // OpenCV 库，用于图像处理

// 定义一个名为 ImageProcessor 的类，它继承自 rclcpp::Node
class ImageProcessor : public rclcpp::Node {
public:
    // 初始化成员变量 image_received_ 为 false
    ImageProcessor() : Node("image_processor"), image_received_(false) {
        // 创建一个图像订阅者
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_topic", 
            10,           
            // 使用 lambda 表达式作为回调函数，当收到消息时自动执行
            [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                if (!image_received_) {
                    try {
                        cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
                        
                        // 显示原始图像
                        cv::namedWindow("Original", cv::WINDOW_NORMAL); // 创建一个可调整大小的窗口
                        cv::resizeWindow("Original", 640, 480);         // 将窗口大小初始化为 640x480
                        cv::imshow("Original", image);                  // 在窗口中显示图像

                        // HSV 空间更适合进行颜色分割
                        cv::Mat hsv_image;
                        cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

                        // 定义蓝色在 HSV 空间中的范围
                        cv::Scalar lower_blue = cv::Scalar(96, 119, 73);
                        cv::Scalar upper_blue = cv::Scalar(104, 255, 255);

                        // 根据颜色范围创建掩码 (Mask)
                        cv::Mat mask;
                        cv::inRange(hsv_image, lower_blue, upper_blue, mask);

                        // 形态学操作，优化掩码
                        // 去除小的白色噪点，并填充白色区域内的小黑洞
                        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
                        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel); // 先闭运算（填充）
                        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);  // 再开运算（去噪）

                        // 显示处理后的掩码图像
                        cv::namedWindow("Processed", cv::WINDOW_NORMAL);
                        cv::resizeWindow("Processed", 640, 480);
                        cv::imshow("Processed", mask);
                        
                        RCLCPP_INFO(this->get_logger(), "Received and processed the first image.");
                        RCLCPP_INFO(this->get_logger(), "Press any key in the image window to exit.");

                        cv::waitKey(0);
                        
                        // 关闭所有 OpenCV 创建的窗口
                        cv::destroyAllWindows();

                        image_received_ = true;
                        
                        subscription_.reset();

                        RCLCPP_INFO(this->get_logger(), "Unsubscribed from image topic.");

                    } catch (cv_bridge::Exception &e) {
                        // 如果在 cv_bridge 转换过程中发生错误，则捕获异常并打印错误日志
                        RCLCPP_ERROR(this->get_logger(), "CV_Bridge exception: %s", e.what());
                    }
                }
            });

        // 打印一条日志，表示节点已成功启动并正在等待图像
        RCLCPP_INFO(this->get_logger(), "Subscription created. Waiting for the first image on 'image_topic'...");
    }

private:
    // 声明一个订阅者的智能指针，用于接收图像消息
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    
    // 用于记录是否已经接收并处理过一张图像，防止重复处理
    bool image_received_;
};

// 主函数
int main(int argc, char **argv) {
    // 初始化 ROS 2
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<ImageProcessor>();
    
    // 运行节点，使其保持活跃状态，等待并处理回调函数
    rclcpp::spin(node);
    
    rclcpp::shutdown();
    
    return 0;
}