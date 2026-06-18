#pragma once
#include <opencv2/opencv.hpp>

struct Stage4Result {
    // 限高杆（灰色横杆，蹲下通过）
    bool  limbar_found{false};
    float limbar_cx{0.0f};
    float limbar_dist{0.0f};

    // 可乐瓶（黑色圆柱，撞倒）
    bool  coke_found{false};
    float coke_cx{0.0f};
    float coke_dist{0.0f};

    // 橙色球（形状优先+颜色判断）
    bool  ball_found{false};
    bool  ball_is_orange{false};
    float ball_cx{0.0f};
    float ball_dist{0.0f};

    // 足球（白色圆球）
    bool  football_found{false};
    float football_cx{0.0f};
    float football_dist{0.0f};

    // 障碍物（蓝色方块，绕行）
    bool  obstacle_found{false};
    float obstacle_cx{0.0f};
    float obstacle_dist{0.0f};
};

class Stage4Detector {
public:
    Stage4Detector() = default;
    Stage4Result detect(const cv::Mat& frame);

private:
    static constexpr float FOCAL_LEN   = 402.0f;
    static constexpr float BALL_RADIUS = 0.10f;

    static constexpr int ORANGE_H_LOW  = 5;
    static constexpr int ORANGE_H_HIGH = 25;
    static constexpr int ORANGE_S_MIN  = 100;
    static constexpr int ORANGE_V_MIN  = 100;

    struct CircleResult {
        bool  found{false};
        float cx{0.0f};
        float dist{0.0f};
        float radius{0.0f};
        cv::Point2f center;
    };

    std::vector<CircleResult> find_all_circles(const cv::Mat& gray, int min_r=10, int max_r=100);
    bool is_orange(const cv::Mat& hsv, const cv::Point2f& center, float radius);
    CircleResult find_limbar(const cv::Mat& hsv);    // 限高杆：灰色横向矩形
    CircleResult find_coke(const cv::Mat& hsv);      // 可乐瓶：黑色圆柱
    CircleResult find_football(const cv::Mat& hsv);  // 足球：白色圆球
    CircleResult find_obstacle(const cv::Mat& hsv);  // 障碍物：蓝色方块
};
