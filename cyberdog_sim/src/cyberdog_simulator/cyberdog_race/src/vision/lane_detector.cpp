#include "cyberdog_race/vision/lane_detector.hpp"
#include <numeric>

LaneResult LaneDetector::detect(const cv::Mat& frame) {
    LaneResult result;
    if (frame.empty()) return result;

    // 1. HSV颜色分割
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_low_, hsv_high_, mask);

    // 2. 逐行扫描
    std::vector<cv::Point> left_pts, right_pts;
    scan_edges(mask, left_pts, right_pts);

    // 3. 思路1：过滤横向干扰（斜率突变的点）
    filter_lateral(left_pts);
    filter_lateral(right_pts);

    // 4. 思路3：连续性检验（与上一帧位置差异过大的点丢弃）
    filter_continuity(left_pts,  last_valid_left_x_,  frame.cols);
    filter_continuity(right_pts, last_valid_right_x_, frame.cols);

    last_left_  = left_pts;
    last_right_ = right_pts;

    if (left_pts.empty() && right_pts.empty()) return result;

    // 5. 计算中心偏差
    result.offset = calc_offset(left_pts, right_pts, frame.cols);

    // 6. 计算弯曲程度
    if (!left_pts.empty())
        result.curvature = calc_curvature(left_pts);
    else if (!right_pts.empty())
        result.curvature = calc_curvature(right_pts);

    // 单边且未校准：等双边出现后再开始控制
    if (!width_calibrated_ && (left_pts.empty() || right_pts.empty()))
        return result;

    result.valid = true;
    result.lane_width = lane_width_;

    // 低通滤波
    result.offset = alpha_ * result.offset + (1.0f - alpha_) * last_offset_;
    last_offset_  = result.offset;

    return result;
}

void LaneDetector::scan_edges(const cv::Mat& binary,
                               std::vector<cv::Point>& left,
                               std::vector<cv::Point>& right) {
    int rows = binary.rows;
    int cols = binary.cols;

    for (int r = rows - 1; r > rows / 2; r -= 4) {
        const uchar* row = binary.ptr<uchar>(r);

        // 左黄线外边缘（黑→黄跳变）
        for (int c = 1; c < cols / 2; c++) {
            if (row[c] > 127 && row[c-1] <= 127) {
                left.emplace_back(c, r);
                break;
            }
        }

        // 右黄线外边缘（黑→黄跳变，从右往左）
        for (int c = cols - 2; c >= cols / 2; c--) {
            if (row[c] > 127 && row[c+1] <= 127) {
                right.emplace_back(c, r);
                break;
            }
        }
    }
}

// 思路1：过滤横向干扰
// 正常赛道边界：相邻行x坐标变化缓慢（斜率小）
// 横向边界线：x坐标在某行突然大幅跳变
void LaneDetector::filter_lateral(std::vector<cv::Point>& pts) {
    if (pts.size() < 3) return;
    std::vector<cv::Point> filtered;
    filtered.push_back(pts[0]);
    for (size_t i = 1; i + 1 < pts.size(); i++) {
        // 计算前后两段的x变化量
        float dx_prev = std::abs(pts[i].x - pts[i-1].x);
        float dx_next = std::abs(pts[i+1].x - pts[i].x);
        // 如果某点与前后点的x差都很大，认为是横向干扰，丢弃
        if (dx_prev > LATERAL_THRESH && dx_next > LATERAL_THRESH) continue;
        filtered.push_back(pts[i]);
    }
    if (!pts.empty()) filtered.push_back(pts.back());
    pts = filtered;
}

// 思路3：连续性检验
// 与上一帧有效位置差异超过阈值的点丢弃
void LaneDetector::filter_continuity(std::vector<cv::Point>& pts,
                                      float& last_valid_x,
                                      int img_width) {
    if (pts.empty()) return;

    // 用底部几个点的均值代表当前帧位置
    int n = std::min((int)pts.size(), 5);
    float cur_x = 0;
    for (int i = 0; i < n; i++) cur_x += pts[i].x;
    cur_x /= n;

    if (last_valid_x >= 0) {
        float jump = std::abs(cur_x - last_valid_x);
        if (jump > img_width * CONTINUITY_THRESH) {
            // 跳变太大，整组点丢弃，保持上一帧状态
            pts.clear();
            return;
        }
    }
    last_valid_x = cur_x;
}

float LaneDetector::calc_offset(const std::vector<cv::Point>& left,
                                  const std::vector<cv::Point>& right,
                                  int img_width) {
    float center = img_width / 2.0f;

    auto avg_x = [](const std::vector<cv::Point>& pts, int take) -> float {
        float sum = 0;
        int n = std::min((int)pts.size(), take);
        for (int i = 0; i < n; i++) sum += pts[i].x;
        return sum / n;
    };
    int take   = std::max(3, (int)std::min(left.size(), right.size()) / 3);
    int take_l = std::max(3, (int)left.size()  / 3);
    int take_r = std::max(3, (int)right.size() / 3);

    float road_center;
    bool both_sides = !left.empty() && !right.empty();
    bool left_only = !left.empty() && right.empty();
    bool right_only = left.empty() && !right.empty();
    
    if (both_sides) {
        float lx = avg_x(left,  take);
        float rx = avg_x(right, take);
        road_center = (lx + rx) / 2.0f;

        float w = rx - lx;
        // 宽度合理性检查：0.05~0.60，超过60%认为是横线干扰，拒绝更新
        if (w > img_width * 0.05f && w < img_width * 0.60f) {
            if (!width_calibrated_) {
                lane_width_ = w;
                width_calibrated_ = true;
                last_lane_width_ = w;
                has_history_ = true;
            } else {
                lane_width_ = 0.3f * w + 0.7f * lane_width_;
                last_lane_width_ = lane_width_;
            }
        }
        // 更新历史数据
        last_lane_slope_ = 0.0f; // 双边时斜率设为0
        last_lane_width_ = lane_width_;
        has_history_ = true;
        
    } else if (left_only) {
        float lx = avg_x(left, take_l);
        if (width_calibrated_) {
            road_center = lx + lane_width_ / 2.0f;
        } else if (has_history_) {
            // 使用历史车道宽度
            road_center = lx + last_lane_width_ / 2.0f;
        } else {
            road_center = lx + img_width * 0.25f;
        }
    } else if (right_only) {
        float rx = avg_x(right, take_r);
        if (width_calibrated_) {
            road_center = rx - lane_width_ / 2.0f;
        } else if (has_history_) {
            // 使用历史车道宽度
            road_center = rx - last_lane_width_ / 2.0f;
        } else {
            road_center = rx - img_width * 0.25f;
        }
    } else {
        // 两边都空，使用上一帧的偏移（如果有历史）
        if (has_history_) {
            return last_offset_; // 使用上一帧的偏移
        }
        return 0.0f;
    }

    return (road_center - center) / center;
}

float LaneDetector::calc_curvature(const std::vector<cv::Point>& edge) {
    if (edge.size() < 5) return 0.0f;

    float sum_slope = 0.0f, sum_sq = 0.0f;
    int n = 0;
    for (size_t i = 4; i < edge.size(); i += 4) {
        float dy = edge[i].x - edge[i-4].x;
        float dx = edge[i].y - edge[i-4].y;
        float slope = (dx != 0) ? dy / dx : 255.0f;
        sum_slope += slope;
        sum_sq += slope * slope;
        n++;
    }
    if (n < 2) return 0.0f;
    float mean = sum_slope / n;
    return std::sqrt(sum_sq / n - mean * mean);
}
