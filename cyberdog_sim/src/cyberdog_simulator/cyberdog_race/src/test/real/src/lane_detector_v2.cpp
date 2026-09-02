#include "lane_detector_v2.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>

void LaneDetector::set_mode(LaneMode mode) {
    if (mode_ == mode) return;
    mode_ = mode;

    // Stage1 与 Stage3 的视角、边界形态不同，不能沿用上一赛段的线位置和宽度。
    last_left_.clear();
    last_right_.clear();
    last_offset_ = 0.0f;
    lane_width_ = 180.0f;
    width_calibrated_ = false;
    last_valid_left_x_ = -1.0f;
    last_valid_right_x_ = -1.0f;
    last_lane_slope_ = 0.0f;
    last_lane_width_ = 0.0f;
    has_history_ = false;
    left_jump_counter_ = 0;
    right_jump_counter_ = 0;
    relaxed_lost_frames_ = 0;
}

LaneResult LaneDetector::detect(const cv::Mat& frame) {
    LaneResult result;
    if (frame.empty()) return result;

    // 1. HSV颜色分割
    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, hsv_low_, hsv_high_, mask);

    // 2. Stage1 保留原来的双边扫描；Stage3 从黄色连通轨迹中选赛道边界。
    // Stage3 实拍中常常只看见一侧曲线，且远处存在横向黄色线，不能再从
    // 图像左右边缘简单取每行遇到的第一段黄色。
    std::vector<cv::Point> left_pts, right_pts;
    if (mode_ == LaneMode::RELAXED)
        scan_stage3_tracks(mask, frame, left_pts, right_pts);
    else
        scan_edges(mask, left_pts, right_pts);

    // 2.5 五点滑动平均: 边线x序列平滑, 抑制阈值边界逐帧抖动
    auto smooth5 = [](std::vector<cv::Point>& pts) {
        if (pts.size() < 5) return;
        std::vector<int> xs(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) xs[i] = pts[i].x;
        for (size_t i = 2; i + 2 < pts.size(); ++i)
            pts[i].x = (xs[i - 2] + xs[i - 1] + xs[i] + xs[i + 1] + xs[i + 2]) / 5;
    };
    smooth5(left_pts);
    smooth5(right_pts);

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

    // 6. 计算弯曲程度（先算，后续valid再决定是否被上层信任）
    if (!left_pts.empty())
        result.curvature = calc_curvature(left_pts);
    else if (!right_pts.empty())
        result.curvature = calc_curvature(right_pts);

    result.valid = true;
    result.lane_width = lane_width_;
    result.both_sides = !left_pts.empty() && !right_pts.empty();

    // 7. 单线近端横向位置 (沿线趋势跟踪用)
    //    取近端(最大y)1/3点平均x, 归一化[-1,1], 避免远端透视干扰
    if (!result.both_sides) {
        const auto& pts = !left_pts.empty() ? left_pts : right_pts;
        std::vector<cv::Point> near_pts = pts;
        std::sort(near_pts.begin(), near_pts.end(),
                  [](const cv::Point& a, const cv::Point& b) { return a.y > b.y; });
        const size_t n = std::max<size_t>(4, near_pts.size() / 3);
        double sx = 0.0;
        for (size_t i = 0; i < n && i < near_pts.size(); ++i) sx += near_pts[i].x;
        const float avg_x = static_cast<float>(sx / static_cast<double>(n));
        result.line_x = (avg_x - frame.cols / 2.0f) / (frame.cols / 2.0f);
    }

    // 低通滤波
    result.offset = alpha_ * result.offset + (1.0f - alpha_) * last_offset_;
    last_offset_  = result.offset;

    return result;
}

void LaneDetector::scan_stage3_tracks(const cv::Mat& binary, const cv::Mat& frame,
                                      std::vector<cv::Point>& left,
                                      std::vector<cv::Point>& right) {
    const int rows = binary.rows;
    const int cols = binary.cols;
    if (rows < 40 || cols < 40) return;

    // 只看地面区域；开运算去反光碎点，闭运算连接胶带皱褶造成的小断口。
    // ROI 用下1/2，看更远（弯道提前可见）
    const int roi_y = static_cast<int>(rows / 2);
    cv::Mat roi = binary(cv::Rect(0, roi_y, cols, rows - roi_y)).clone();
    cv::morphologyEx(
        roi, roi, cv::MORPH_OPEN,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
    // 闭运算9x9（15x15会把远处两条线粘住，不稳定）
    cv::morphologyEx(
        roi, roi, cv::MORPH_CLOSE,
        cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(9, 9)));

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(roi, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    struct Track {
        std::vector<cv::Point> points;       // 黄带中心(展示/连续性用)
        std::vector<cv::Point> inner_left;   // 带右缘x1 = 左线内侧
        std::vector<cv::Point> inner_right;  // 带左缘x0 = 右线内侧
        float near_x{0.0f};
        float score{0.0f};
    };
    std::vector<Track> tracks;

    for (const auto& contour : contours) {
        if (cv::contourArea(contour) < cols * rows * 0.0007f) continue;
        cv::Rect box = cv::boundingRect(contour);
        const int global_bottom = box.y + box.height + roi_y;
        const float vertical_span = static_cast<float>(box.height) / rows;
        const float horizontal_span = static_cast<float>(box.width) / cols;

        // 横向干扰通常很宽但纵向只有几像素；有效曲线必须在地面 ROI 内
        // 跨越足够多的行，并延伸到画面较近处。
        if (vertical_span < 0.10f) continue;
        if (global_bottom < static_cast<int>(rows * 0.64f)) continue;
        if (horizontal_span > 0.88f && vertical_span < 0.22f) continue;

        cv::Mat component = cv::Mat::zeros(roi.size(), CV_8UC1);
        std::vector<std::vector<cv::Point>> one{contour};
        cv::drawContours(component, one, 0, cv::Scalar(255), cv::FILLED);

        Track track;
        // 从近到远按行取黄色带中心。只保留连续的下半段轨迹，避免同色墙面
        // 或远处横线通过反光偶然和目标连成一个轮廓。
        int gap_rows = 0;
        int last_x = -1;
        for (int y = component.rows - 1; y >= 0; y -= 4) {
            const uchar* row = component.ptr<uchar>(y);
            // ── 逐行跟踪取段: 选离上一行中心最近的黄段 ──
            //   旧版取"最长段"在段长度接近时会互相切换→边缘跳变→不稳定;
            //   最近段=连续性优先, 碎片离轨迹远自然被忽略(八邻域巡线同思路)
            int best0 = -1, best1 = -1, best_dist = 0x7fffffff, best_len = 0;
            for (int x = 0; x < cols; ) {
                if (!row[x]) { ++x; continue; }
                const int s = x;
                while (x < cols && row[x]) ++x;
                const int len = x - s;
                const int cx = (s + x - 1) / 2;
                if (last_x < 0) {
                    // 首行: 选最长段
                    if (len > best_len) { best_len = len; best0 = s; best1 = x - 1; }
                } else {
                    const int d = std::abs(cx - last_x);
                    if (d < best_dist) { best_dist = d; best0 = s; best1 = x - 1; }
                }
            }
            if (best0 < 0) {
                if (!track.points.empty() && ++gap_rows > 4) break;
                continue;
            }
            // ── gap续接保护: 恢复点离gap前太远→继续算gap, 防远距硬接成折线 ──
            if (gap_rows > 0) {
                const int cx = (best0 + best1) / 2;
                if (std::abs(cx - last_x) > cols * 0.06f) {
                    if (++gap_rows > 4) break;
                    continue;
                }
            }
            gap_rows = 0;
            int x0 = best0, x1 = best1;
            // ── 边缘精修: 在端点邻域内找B通道突变最大处 ──
            //   黄线B低/地面B高, 阈值二值化边缘在灰度渐渡处会抖,
            //   梯度最大点=真正的"黄色突变处", 逐帧稳定
            {
                const uchar* brow = frame.ptr<uchar>(y + roi_y);
                auto refine = [&](int xe, int dir) -> int {
                    int best = xe, bg = 0;
                    for (int d = dir; std::abs(d) <= 8; d += dir) {
                        const int xi = xe + d;
                        if (xi < 1 || xi >= cols - 1) break;
                        const int g = std::abs(int(brow[3 * (xi + 1)]) - int(brow[3 * (xi - 1)]));
                        if (g > bg) { bg = g; best = xi; }
                    }
                    // 限幅±4px: 只做微调, 防搜到带外杂物/反光边缘→跳变折线
                    return std::max(xe - 4, std::min(xe + 4, best));
                };
                x1 = refine(x1, +1);   // 左线内侧=带右缘, 往右搜
                x0 = refine(x0, -1);   // 右线内侧=带左缘, 往左搜
            }
            const int x = (x0 + x1) / 2;
            if (last_x >= 0 && std::abs(x - last_x) > cols * 0.18f) break;
            track.points.emplace_back(x, y + roi_y);
            // 内侧边缘: 赛道线宽粗, 外侧受光照/视角影响不稳,
            //   左线用带右缘x1(靠赛道中), 右线用带左缘x0, 归属确定后二选一
            track.inner_left.emplace_back(x1, y + roi_y);
            track.inner_right.emplace_back(x0, y + roi_y);
            last_x = x;
        }
        if (track.points.size() < 6) continue;

        const int near_count = std::min<int>(5, track.points.size());
        for (int i = 0; i < near_count; ++i) track.near_x += track.points[i].x;
        track.near_x /= near_count;

        const float bottom_score = static_cast<float>(global_bottom) / rows;
        track.score = vertical_span * 3.0f + bottom_score
                    + std::min(0.5f, static_cast<float>(track.points.size()) / 80.0f);
        tracks.push_back(std::move(track));
    }

    std::sort(tracks.begin(), tracks.end(),
              [](const Track& a, const Track& b) { return a.score > b.score; });
    if (tracks.empty()) {
        ++relaxed_lost_frames_;
        return;
    }

    // 最多保留两条互相分离的赛道边界。图二只有一条时仍然输出有效单边线。
    std::vector<Track> selected;
    selected.push_back(tracks.front());
    for (size_t i = 1; i < tracks.size() && selected.size() < 2; ++i) {
        if (std::abs(tracks[i].near_x - selected.front().near_x) > cols * 0.18f)
            selected.push_back(tracks[i]);
    }
    std::sort(selected.begin(), selected.end(),
              [](const Track& a, const Track& b) { return a.near_x < b.near_x; });

    if (selected.size() == 2) {
        left  = std::move(selected[0].inner_left);   // 左线取内侧(带右缘)
        right = std::move(selected[1].inner_right);  // 右线取内侧(带左缘)
    } else {
        // 单边归属优先参考上一帧，其次看曲线近端处于画面哪一侧。
        const float x = selected[0].near_x;
        bool is_left = x < cols * 0.5f;
        if (last_valid_left_x_ >= 0.0f || last_valid_right_x_ >= 0.0f) {
            const float dl = last_valid_left_x_ >= 0.0f
                           ? std::abs(x - last_valid_left_x_) : static_cast<float>(cols);
            const float dr = last_valid_right_x_ >= 0.0f
                           ? std::abs(x - last_valid_right_x_) : static_cast<float>(cols);
            is_left = dl <= dr;
        }
        if (is_left) left = std::move(selected[0].inner_left);
        else right = std::move(selected[0].inner_right);
    }
    relaxed_lost_frames_ = 0;
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
    float thresh = (mode_ == LaneMode::RELAXED) ? LATERAL_THRESH_RELAXED : LATERAL_THRESH_STRICT;
    std::vector<cv::Point> filtered;
    filtered.push_back(pts[0]);
    for (size_t i = 1; i + 1 < pts.size(); i++) {
        float dx_prev = std::abs(pts[i].x - pts[i-1].x);
        float dx_next = std::abs(pts[i+1].x - pts[i].x);
        if (dx_prev > thresh && dx_next > thresh) continue;
        filtered.push_back(pts[i]);
    }
    if (!pts.empty()) filtered.push_back(pts.back());
    pts = filtered;
}

// 思路3：连续性检验（带滞回+连续帧投票，避免单帧跳变直接清空）
// 进入丢线：连续 kJumpFramesEnter 帧超阈值才清空；
// 恢复：任一帧回落到阈值内即复位计数器（经验：车道跳变噪声多为1帧尖峰）
void LaneDetector::filter_continuity(std::vector<cv::Point>& pts,
                                      float& last_valid_x,
                                      int img_width) {
    if (pts.empty()) return;
    float thresh_enter = (mode_ == LaneMode::RELAXED) ? CONTINUITY_THRESH_RELAXED
                                                        : CONTINUITY_THRESH_STRICT;
    // 退出阈值比进入阈值低 25%，形成滞回窗口，避免边界抖动
    float thresh_exit  = thresh_enter * 0.75f;

    int n = std::min((int)pts.size(), 5);
    float cur_x = 0;
    for (int i = 0; i < n; i++) cur_x += pts[i].x;
    cur_x /= n;

    if (last_valid_x >= 0) {
        float jump = std::abs(cur_x - last_valid_x);
        if (jump > img_width * thresh_enter) {
            // 超阈值：只记录计数，不立即清空（除非连续多帧）
            // 判断是左线还是右线：last_valid_x < img_width/2 视为左线
            int& counter = (last_valid_x < img_width * 0.5f)
                           ? left_jump_counter_ : right_jump_counter_;
            counter++;
            if (counter >= kJumpFramesEnter) {
                // 连续多帧跳变：确认丢线，清空当前帧点但保留 last_valid_x 作为后续参考
                pts.clear();
                counter = kJumpFramesEnter;  // 饱和，避免溢出
                return;
            }
            // 未达连续阈值：降级处理——把当前检测值向 last_valid_x 拉回 (1-α)*old + α*new
            // α 与跳变程度成反比：跳越大，越不信任当前值，保持历史
            float alpha = std::max(0.05f, 1.0f - jump / (img_width * 0.5f));
            float clamped = (1.0f - alpha) * last_valid_x + alpha * cur_x;
            for (auto& p : pts) p.x = static_cast<int>(clamped);
            cur_x = clamped;
        } else if (jump < img_width * thresh_exit) {
            // 回落到退出阈值内，清零计数器
            if (last_valid_x < img_width * 0.5f) left_jump_counter_  = 0;
            else                                 right_jump_counter_ = 0;
        }
    }
    last_valid_x = cur_x;
}

float LaneDetector::calc_offset(const std::vector<cv::Point>& left,
                                  const std::vector<cv::Point>& right,
                                  int img_width) {
    float center = img_width / 2.0f;

    auto avg_near_x = [](const std::vector<cv::Point>& pts, int take) -> float {
        float sum = 0;
        int n = std::min((int)pts.size(), take);
        if (n <= 0) return 0.0f;
        for (int i = 0; i < n; i++) sum += pts[i].x;
        return sum / n;
    };
    auto avg_lookahead_x = [](const std::vector<cv::Point>& pts) -> float {
        if (pts.empty()) return 0.0f;
        // Stage3 轨迹按近到远排列。取轨迹中段而不是画面最底部，才能提前
        // 看见图二这种右弯；窗口平均降低胶带皱褶带来的像素抖动。
        const int n = static_cast<int>(pts.size());
        const int mid = std::min(n - 1, static_cast<int>(n * 0.45f));
        const int begin = std::max(0, mid - 2);
        const int end = std::min(n, mid + 3);
        float sum = 0.0f;
        for (int i = begin; i < end; ++i) sum += pts[i].x;
        return sum / std::max(1, end - begin);
    };
    auto sample_x = [&](const std::vector<cv::Point>& pts, int take) -> float {
        return mode_ == LaneMode::RELAXED ? avg_lookahead_x(pts)
                                           : avg_near_x(pts, take);
    };
    int take   = std::max(3, (int)std::min(left.size(), right.size()) / 3);
    int take_l = std::max(3, (int)left.size()  / 3);
    int take_r = std::max(3, (int)right.size() / 3);

    float road_center;
    bool both_sides = !left.empty() && !right.empty();
    bool left_only = !left.empty() && right.empty();
    bool right_only = left.empty() && !right.empty();
    
    if (both_sides) {
        float lx = sample_x(left,  take);
        float rx = sample_x(right, take);
        road_center = (lx + rx) / 2.0f;

        float w = rx - lx;
        const float min_width = mode_ == LaneMode::RELAXED ? 0.15f : 0.05f;
        const float max_width = mode_ == LaneMode::RELAXED ? 0.95f : 0.60f;
        // Stage3 低机位近场双边可占画面约80%，不能沿用 Stage1 的60%上限。
        if (w > img_width * min_width && w < img_width * max_width) {
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
        float lx = sample_x(left, take_l);
        if (width_calibrated_) {
            road_center = lx + lane_width_ / 2.0f;
        } else if (has_history_) {
            // 使用历史车道宽度
            road_center = lx + last_lane_width_ / 2.0f;
        } else {
            // Stage3 入口可能首帧就是单边。以约半个赛道宽度推算中心，
            // 后续一旦看到双边会自动用实测宽度校准。
            road_center = lx + img_width * (mode_ == LaneMode::RELAXED ? 0.40f : 0.25f);
        }
    } else if (right_only) {
        float rx = sample_x(right, take_r);
        if (width_calibrated_) {
            road_center = rx - lane_width_ / 2.0f;
        } else if (has_history_) {
            // 使用历史车道宽度
            road_center = rx - last_lane_width_ / 2.0f;
        } else {
            road_center = rx - img_width * (mode_ == LaneMode::RELAXED ? 0.40f : 0.25f);
        }
    } else {
        // 两边都空，使用上一帧的偏移（如果有历史）
        if (has_history_) {
            return last_offset_; // 使用上一帧的偏移
        }
        return 0.0f;
    }

    if (center < 1.0f) center = 1.0f;  // 除零兜底（空帧路径，detect上游已拦但保守）
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
    // 方差 = E[X²] - (E[X])²，浮点舍入可能使结果微负 → 必须加 std::max(0, ...)，
    // 否则 sqrt(负数) 产生 NaN，后续所有 cv>thresh 判定恒false，弯道无法识别。
    float var = sum_sq / n - mean * mean;
    return std::sqrt(std::max(0.0f, var));
}
