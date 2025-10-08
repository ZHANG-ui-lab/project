#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <chrono> // 头文件

using namespace std::chrono_literals;

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
        
        // 在构造函数中一次性读取图像
        // 请确保图像路径正确
        image_ = cv::imread("/home/zmy/ros2_ws/image.jpg");
        if (image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load image!");
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Image loaded successfully. Starting to publish...");

        // 创建一个定时器，每 1 秒（1000ms）触发一次回调
        timer_ = this->create_wall_timer(
            1000ms, // 时间间隔，这里是 1 秒
            std::bind(&ImagePublisher::timer_callback, this)
        );
    }

private:
    // 定时器回调函数，负责周期性地发布图像
    void timer_callback() {
        if (image_.empty()) {
            return; // 如果图像未加载，则不执行任何操作
        }
        
        // 将预先加载好的 OpenCV 图像转换为 ROS 消息并发布
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Publishing image...");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat image_; // 将图像作为类成员变量，避免在回调中重复读取
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}