#pragma once
#include <opencv2/opencv.hpp>

enum class BallColor { ORANGE, BLUE, NONE };

struct BallResult {
    BallColor color{BallColor::NONE};
    float cx{0.0f};    // 球心x（归一化[-1,1]，0=图像中心）
    float cy{0.0f};    // 球心y
    float radius{0.0f};// 球半径（像素）
    bool  found{false};
};

class BallDetector {
public:
    BallDetector() = default;

    // 检测指定颜色的球
    BallResult detect(const cv::Mat& frame, BallColor target);

private:
    // 橙色HSV范围
    cv::Scalar orange_low_{5, 120, 120};
    cv::Scalar orange_high_{20, 255, 255};

    // 蓝色HSV范围
    cv::Scalar blue_low_{95, 80, 80};
    cv::Scalar blue_high_{125, 255, 255};

    BallResult find_ball(const cv::Mat& frame, 
                         const cv::Scalar& low,
                         const cv::Scalar& high,
                         BallColor color);
};
