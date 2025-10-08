#include <opencv2/opencv.hpp>
#include <iostream>

// 回调函数，滑动条值改变时调用
void nothing(int x, void* userdata) {}

int main() {
    // 读取图像
    std::string image_path = "/home/zmy/ros2_ws/image.jpg";
    cv::Mat image = cv::imread(image_path);

    if (image.empty()) {
        std::cerr << "错误：无法读取图像文件 '" << image_path << "'。" << std::endl;
        return -1;
    }

    // 创建窗口
    cv::namedWindow("HSV Color Picker", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("Original Image", cv::WINDOW_NORMAL); // 设置为可调整大小
    cv::namedWindow("HSV Color Picker (Adjust until blue is white)", cv::WINDOW_NORMAL); // 设置为可调整大小

    // 调整窗口初始大小
    cv::resizeWindow("Original Image", 640, 480);
    cv::resizeWindow("HSV Color Picker (Adjust until blue is white)", 640, 480);

    // 创建滑动条来调整 HSV 的范围
    int h_min = 0, h_max = 179;
    int s_min = 0, s_max = 255;
    int v_min = 0, v_max = 255;

    cv::createTrackbar("H Min", "HSV Color Picker", &h_min, 179, nothing, NULL);
    cv::createTrackbar("H Max", "HSV Color Picker", &h_max, 179, nothing, NULL);
    cv::createTrackbar("S Min", "HSV Color Picker", &s_min, 255, nothing, NULL);
    cv::createTrackbar("S Max", "HSV Color Picker", &s_max, 255, nothing, NULL);
    cv::createTrackbar("V Min", "HSV Color Picker", &v_min, 255, nothing, NULL);
    cv::createTrackbar("V Max", "HSV Color Picker", &v_max, 255, nothing, NULL);

    // 将图像转换为 HSV
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

    while (true) {
        // 4. 获取当前滑动条的值
        h_min = cv::getTrackbarPos("H Min", "HSV Color Picker");
        h_max = cv::getTrackbarPos("H Max", "HSV Color Picker");
        s_min = cv::getTrackbarPos("S Min", "HSV Color Picker");
        s_max = cv::getTrackbarPos("S Max", "HSV Color Picker");
        v_min = cv::getTrackbarPos("V Min", "HSV Color Picker");
        v_max = cv::getTrackbarPos("V Max", "HSV Color Picker");

        // 定义当前的颜色范围
        cv::Scalar lower_bound(h_min, s_min, v_min);
        cv::Scalar upper_bound(h_max, s_max, v_max);

        // 根据范围创建掩码
        cv::Mat mask;
        cv::inRange(hsv_image, lower_bound, upper_bound, mask);

        // 进行形态学操作，让掩码更干净
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);

        // 显示原图和掩码（窗口大小已调整）
        cv::imshow("Original Image", image);
        cv::imshow("HSV Color Picker (Adjust until blue is white)", mask);

        // 等待 1 毫秒，检查键盘输入
        int key = cv::waitKey(1) & 0xFF;
        // 如果按下 's' 键，则保存当前的掩码并退出
        if (key == 's') {
            cv::imwrite("perfect_mask.png", mask);
            std::cout << "成功！完美的掩码已保存为 'perfect_mask.png'" << std::endl;
            std::cout << "当前最佳 HSV 范围: H[" << h_min << ", " << h_max << "], S[" 
                      << s_min << ", " << s_max << "], V[" << v_min << ", " << v_max << "]" << std::endl;
            break;
        }
        // 如果按下 'q' 键，则直接退出
        else if (key == 'q') {
            break;
        }
    }

    // 关闭所有窗口
    cv::destroyAllWindows();

    return 0;
}