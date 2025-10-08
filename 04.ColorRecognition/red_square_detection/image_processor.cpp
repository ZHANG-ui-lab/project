#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>

// 优化 is_square 函数，使其更健壮
bool is_square(const std::vector<cv::Point> &contour, double tolerance = 0.3) {
    std::vector<cv::Point> approx;
    // 使用一个更固定的精度值进行轮廓逼近，提高稳定性
    double epsilon = 0.03 * cv::arcLength(contour, true);
    cv::approxPolyDP(contour, approx, epsilon, true);
    
    // 必须是凸四边形
    if (approx.size() != 4 || !cv::isContourConvex(approx)) {
        return false;
    }
    
    // 计算四条边的长度
    std::vector<double> sides;
    for (int i = 0; i < 4; ++i) {
        sides.push_back(cv::norm(approx[i] - approx[(i+1)%4]));
    }
    double avg_side = std::accumulate(sides.begin(), sides.end(), 0.0) / 4.0;
    
    // 检查四条边是否大致相等
    for (double side : sides) {
        if (std::abs(side - avg_side) / avg_side > tolerance) {
            return false;
        }
    }
    
    // 计算两条对角线的长度
    double diag1 = cv::norm(approx[0] - approx[2]);
    double diag2 = cv::norm(approx[1] - approx[3]);
    
    // 检查两条对角线是否大致相等
    if (std::abs(diag1 - diag2) / std::max(diag1, diag2) > tolerance) {
        return false;
    }
    
    return true;
}

class ImageProcessor : public rclcpp::Node {
public:
    ImageProcessor() : Node("image_processor"), processed_(false) {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_topic",
            10,
            std::bind(&ImageProcessor::image_callback, this, std::placeholders::_1));
    }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        if (processed_) {
            return;
        }
        
        try {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::Mat hsv, mask_red, result_img = image.clone();
            
            cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
            
            // HSV颜色范围保持不变，因为Red Mask效果不错
            cv::Scalar lower_red1 = cv::Scalar(0, 120, 80);
            cv::Scalar upper_red1 = cv::Scalar(10, 255, 255);
            cv::Scalar lower_red2 = cv::Scalar(170, 120, 80);
            cv::Scalar upper_red2 = cv::Scalar(180, 255, 255);
            
            cv::Mat mask_red1, mask_red2;
            cv::inRange(hsv, lower_red1, upper_red1, mask_red1);
            cv::inRange(hsv, lower_red2, upper_red2, mask_red2);
            mask_red = mask_red1 + mask_red2;
            
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
            cv::morphologyEx(mask_red, mask_red, cv::MORPH_CLOSE, kernel, cv::Point(-1, -1), 3);
            cv::morphologyEx(mask_red, mask_red, cv::MORPH_OPEN, kernel, cv::Point(-1, -1), 2);
            
            std::vector<std::vector<cv::Point>> contours;
            std::vector<cv::Vec4i> hierarchy;
            cv::findContours(mask_red, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            
            bool found_square = false; // 添加一个标志位
            
            for (const auto &cnt : contours) {
                double area = cv::contourArea(cnt);
                
                // 降低面积阈值，以检测更小的正方形
                if (area < 300) {
                    continue;
                }

                // 打印轮廓信息到终端
                RCLCPP_INFO(this->get_logger(), "Found contour with area: %.2f", area);
                
                // 使用放宽的容差进行正方形检测
                if (is_square(cnt, 0.3)) {
                    found_square = true; // 标记找到正方形
                    
                    // 获取轮廓的多边形逼近
                    std::vector<cv::Point> approx;
                    cv::approxPolyDP(cnt, approx, 0.03 * cv::arcLength(cnt, true), true);
                    
                    // 绘制轮廓
                    cv::drawContours(result_img, std::vector<std::vector<cv::Point>>{approx}, -1, cv::Scalar(0, 255, 0), 2);
                    
                    // 计算并绘制中心点
                    cv::Moments M = cv::moments(approx);
                    if (M.m00 > 0) { // 防止除以零
                        int cx = static_cast<int>(M.m10 / M.m00);
                        int cy = static_cast<int>(M.m01 / M.m00);
                        cv::circle(result_img, cv::Point(cx, cy), 5, cv::Scalar(255, 0, 0), -1);
                        cv::putText(result_img, "Red Square", cv::Point(cx - 50, cy - 10),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
                    }
                }
            }
            
            // 显示图像
            cv::namedWindow("Original Image", cv::WINDOW_NORMAL);
            cv::resizeWindow("Original Image", 640, 480);
            cv::imshow("Original Image", image);
            
            cv::namedWindow("Red Mask", cv::WINDOW_NORMAL);
            cv::resizeWindow("Red Mask", 640, 480);
            cv::imshow("Red Mask", mask_red);
            
            cv::namedWindow("Red Square Detection", cv::WINDOW_NORMAL);
            cv::resizeWindow("Red Square Detection", 640, 480);
            cv::imshow("Red Square Detection", result_img);
            
            // [调试] 如果没找到正方形，打印信息
            if (!found_square) {
                 RCLCPP_INFO(this->get_logger(), "No red square detected.");
            }
            
            cv::waitKey(0);
            cv::destroyAllWindows();
            processed_ = true;
            rclcpp::shutdown();
        } catch (cv_bridge::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "CV_Bridge exception: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    bool processed_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageProcessor>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}