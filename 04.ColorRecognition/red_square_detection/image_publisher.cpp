#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

class ImagePublisher : public rclcpp::Node {
public:
    ImagePublisher() : Node("image_publisher") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_topic", 10);
        
        image_path_ = "/home/zmy/my_ros2_ws/src/color_detection/image.jpg";
        image_ = cv::imread(image_path_);
        
        if (image_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Error: Could not read image '%s'", image_path_.c_str());
            rclcpp::shutdown();
            return;
        }
        
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&ImagePublisher::publish_image, this));
    }

private:
    void publish_image() {
        sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_).toImageMsg();
        publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Publishing image");
    }

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string image_path_;
    cv::Mat image_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImagePublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}