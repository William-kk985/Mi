#pragma once
#include <opencv2/opencv.hpp>

enum class BallColor { ORANGE, BLUE, WHITE, NONE };

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
    void reset_filter() { dist_filtered_ = 0.0f; }  // 切换赛段时重置

private:
    // (2026-08-14 折中: S≥130近处球反光边缘漏检; 放宽靠圆度过滤防误检)
    cv::Scalar orange_low_{6, 90, 80};
    cv::Scalar orange_high_{22, 255, 255};
    cv::Scalar blue_low_{95, 80, 80};
    cv::Scalar blue_high_{125, 255, 255};
    cv::Scalar white_low_{0, 0, 180};
    cv::Scalar white_high_{180, 40, 255};  // 完全同 Stage4 find_football

    // RGB_camera: fov=1.3962rad, width=640 → fx=320/tan(0.6981)≈402px
    static constexpr float FOCAL_LEN   = 402.0f;
    static constexpr float BALL_RADIUS = 0.10f;  // 真实半径10cm

    float dist_filtered_{0.0f};

    BallResult find_ball(const cv::Mat& frame,
                         const cv::Scalar& low,
                         const cv::Scalar& high,
                         BallColor color);
};
