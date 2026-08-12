#pragma once
#include <opencv2/opencv.hpp>

struct LaneResult {
    float offset{0.0f};
    float yaw{0.0f};
    float curvature{0.0f};
    float lane_width{0.0f};
    bool  valid{false};
    bool  both_sides{false};
};

enum class LaneMode {
    STRICT,   // stage1：严格过滤，直道为主
    RELAXED   // stage3：宽松过滤，弯道为主
};

class LaneDetector {
public:
    LaneDetector() = default;

    LaneResult detect(const cv::Mat& frame);
    void set_mode(LaneMode mode) { mode_ = mode; }

    std::vector<cv::Point> last_left_;
    std::vector<cv::Point> last_right_;

private:
    cv::Scalar hsv_low_{15, 80, 80};
    cv::Scalar hsv_high_{35, 255, 255};

    float alpha_{0.4f};
    float last_offset_{0.0f};

    float lane_width_{180.0f};
    bool  width_calibrated_{false};

    float last_valid_left_x_{-1.0f};
    float last_valid_right_x_{-1.0f};

    float last_lane_slope_{0.0f};
    float last_lane_width_{0.0f};
    bool  has_history_{false};

    LaneMode mode_{LaneMode::STRICT};

    // STRICT 模式参数
    static constexpr float LATERAL_THRESH_STRICT     = 30.0f;
    static constexpr float CONTINUITY_THRESH_STRICT  = 0.30f;

    // RELAXED 模式参数（弯道更宽松）
    static constexpr float LATERAL_THRESH_RELAXED    = 50.0f;
    static constexpr float CONTINUITY_THRESH_RELAXED = 0.55f;

    void scan_edges(const cv::Mat& binary,
                    std::vector<cv::Point>& left,
                    std::vector<cv::Point>& right);
    void filter_lateral(std::vector<cv::Point>& pts);
    void filter_continuity(std::vector<cv::Point>& pts, float& last_valid_x, int img_width);
    void remove_small_blobs(cv::Mat& mask, int min_area);  // 连通域面积过滤 (2026-08-12 增强)
    float calc_offset(const std::vector<cv::Point>& left,
                      const std::vector<cv::Point>& right,
                      int img_width);
    float calc_curvature(const std::vector<cv::Point>& edge);
};
