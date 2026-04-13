#pragma once
#include <opencv2/opencv.hpp>

struct LaneResult {
    float offset{0.0f};   // 中心偏差，正=偏右，单位：归一化[-1,1]
    float yaw{0.0f};      // 方向偏差（rad）
    float curvature{0.0f};// 弯曲程度，用于判断是否在弯道
    float lane_width{0.0f};// 当前估算赛道宽度（像素），0=未校准
    bool  valid{false};
};

class LaneDetector {
public:
    LaneDetector() = default;

    // 处理一帧图像，返回巡线结果
    LaneResult detect(const cv::Mat& frame);

    // debug用：上一帧的边界点
    std::vector<cv::Point> last_left_;
    std::vector<cv::Point> last_right_;

private:
    // HSV黄色阈值
    cv::Scalar hsv_low_{15, 80, 80};
    cv::Scalar hsv_high_{35, 255, 255};

    // 低通滤波系数（0~1，越小越平滑，越大越跟手）
    float alpha_{0.4f};
    float last_offset_{0.0f};

    // 自适应赛道宽度
    float lane_width_{120.0f};   // 默认初始宽度（像素），双边出现后自动校准
    bool  width_calibrated_{false};

    // 连续性检验：上一帧有效边界x坐标，-1表示未初始化
    float last_valid_left_x_{-1.0f};
    float last_valid_right_x_{-1.0f};

    // 历史车道方向（斜率）和宽度，用于单边预测
    float last_lane_slope_{0.0f};  // 车道方向斜率（dx/dy）
    float last_lane_width_{0.0f};  // 上一帧有效车道宽度
    bool  has_history_{false};     // 是否有历史数据

    // 横向干扰过滤阈值（相邻行x坐标最大允许跳变，像素）
    static constexpr float LATERAL_THRESH     = 30.0f;
    // 连续性检验阈值（与上一帧x坐标差，相对图像宽度比例）
    static constexpr float CONTINUITY_THRESH  = 0.30f;

    void scan_edges(const cv::Mat& binary,
                    std::vector<cv::Point>& left,
                    std::vector<cv::Point>& right);
    void filter_lateral(std::vector<cv::Point>& pts);
    void filter_continuity(std::vector<cv::Point>& pts, float& last_valid_x, int img_width);
    float calc_offset(const std::vector<cv::Point>& left,
                      const std::vector<cv::Point>& right,
                      int img_width);
    float calc_curvature(const std::vector<cv::Point>& edge);
};
