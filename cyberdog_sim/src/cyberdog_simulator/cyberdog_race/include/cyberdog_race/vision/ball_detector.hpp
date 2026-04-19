#pragma once
#include <opencv2/opencv.hpp>

enum class BallColor { ORANGE, BLUE, NONE };

struct BallResult {
    BallColor color{BallColor::NONE};
    float cx{0.0f};      // 球心x（归一化[-1,1]，0=图像中心）
    float cy{0.0f};      // 球心y
    float radius{0.0f};  // 球半径（像素，面积等效半径）
    float dist_m{0.0f};  // 估算距离（米）
    bool  found{false};
};

class BallDetector {
public:
    BallDetector() = default;
    BallResult detect(const cv::Mat& frame, BallColor target);

private:
    cv::Scalar orange_low_{5, 80, 80};
    cv::Scalar orange_high_{25, 255, 255};
    cv::Scalar blue_low_{95, 80, 80};
    cv::Scalar blue_high_{125, 255, 255};

    // RGB_camera: fov=1.3962rad, width=640 → fx=320/tan(0.6981)≈402px
    static constexpr float FOCAL_LEN   = 402.0f;
    static constexpr float BALL_RADIUS = 0.10f;  // 真实半径10cm

    float dist_filtered_{0.0f};

    BallResult find_ball(const cv::Mat& frame,
                         const cv::Scalar& low,
                         const cv::Scalar& high,
                         BallColor color);
};
