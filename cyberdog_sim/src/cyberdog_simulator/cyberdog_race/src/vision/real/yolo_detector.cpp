#include "cyberdog_race/vision/real/yolo_detector.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

namespace {
bool send_all(int fd, const void* data, size_t n) {
    const auto* p = static_cast<const uint8_t*>(data);
    size_t off = 0;
    while (off < n) {
        ssize_t w = ::send(fd, p + off, n - off, MSG_NOSIGNAL);
        if (w <= 0) return false;
        off += static_cast<size_t>(w);
    }
    return true;
}
}  // namespace

// ═══════════════════════════════════════════════════════════
// detect：本地 CV（限高杆/障碍/橙球/分隔线）+ 远程 YOLO（可乐/足球）
// ═══════════════════════════════════════════════════════════
YoloResult YoloDetector::detect(const cv::Mat& frame) {
    YoloResult result;
    if (frame.empty()) return result;

    cv::Mat hsv, gray;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // ── 橙色球：形状优先 + 颜色判断 ──
    for (auto& c : find_all_circles(gray)) {
        if (!c.found) continue;
        if (!is_orange(hsv, c.center, c.radius)) continue;
        result.ball_found = true;
        result.ball_is_orange = true;
        result.ball_cx    = c.cx;
        result.ball_dist  = c.dist;
        result.ball_center = c.center;
        result.ball_radius = c.radius;
        break;  // 取最大的球
    }

    // ── 限高杆：红色横杆 ──
    auto limbar = find_limbar(hsv);
    result.limbar_found = limbar.found;
    result.limbar_cx    = limbar.cx;
    result.limbar_dist  = limbar.dist;
    result.limbar_box   = limbar.box;

    // ── 可乐/足球：远程 YOLO（先收上次结果，再发新帧） ──
    Target coke, fb;
    remote_.poll(coke, fb);
    result.coke_found = coke.found;
    result.coke_cx    = coke.cx;
    result.coke_dist  = coke.dist;
    result.coke_box   = coke.box;
    result.coke_conf  = coke.conf;
    result.football_found = fb.found;
    result.football_cx    = fb.cx;
    result.football_dist  = fb.dist;
    result.football_box   = fb.box;
    result.football_conf  = fb.conf;
    remote_.send_frame(frame);   // 发下一帧（不阻塞）

    // ── 障碍物：淡蓝色方块 ──
    auto obstacle = find_obstacle(hsv);
    result.obstacle_found = obstacle.found;
    result.obstacle_cx    = obstacle.cx;
    result.obstacle_dist  = obstacle.dist;
    result.obstacle_box   = obstacle.box;

    // ── 区域分隔黄线：实/虚判断 ──
    auto divider = find_divider_line(hsv);
    result.divider_found     = divider.found;
    result.divider_cx        = divider.cx;
    result.divider_dist      = divider.dist;
    result.divider_is_dashed = divider.is_dashed;

    return result;
}

// ═══════════════════════════════════════════════════════════
// 本地 CV（真机专用）
// ═══════════════════════════════════════════════════════════

// 找所有圆形物体（不管颜色），按半径从大到小
std::vector<YoloDetector::CircleResult> YoloDetector::find_all_circles(
    const cv::Mat& gray, int min_r, int max_r) {
    std::vector<CircleResult> results;
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(9, 9), 2);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1,
                     gray.rows / 8, 100, 25, min_r, max_r);

    for (auto& c : circles) {
        CircleResult r;
        r.found  = true;
        r.center = cv::Point2f(c[0], c[1]);
        r.radius = c[2];
        r.cx     = (c[0] - gray.cols / 2.0f) / (gray.cols / 2.0f);
        if (r.radius > 1.0f) r.dist = (BALL_RADIUS * FOCAL_LEN) / r.radius;
        results.push_back(r);
    }
    std::sort(results.begin(), results.end(),
              [](const CircleResult& a, const CircleResult& b) { return a.radius > b.radius; });
    return results;
}

// 判断圆形区域是否为橙色（gc02m1 低饱和相机阈值放宽）
bool YoloDetector::is_orange(const cv::Mat& hsv, const cv::Point2f& center, float radius) {
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    cv::circle(mask, center, static_cast<int>(radius * 0.8f), 255, -1);
    cv::Scalar mean = cv::mean(hsv, mask);
    return (mean[0] >= ORANGE_H_LOW && mean[0] <= ORANGE_H_HIGH
         && mean[1] >= ORANGE_S_MIN && mean[2] >= ORANGE_V_MIN);
}

// 限高杆：红色横向矩形（宽高比>2.5）
//   实物暗红；红色HSV跨两端（低H+高H）；整片暗区不是横杆（高度上限）
YoloDetector::CircleResult YoloDetector::find_limbar(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask_red_low, mask_red_high, mask;
    cv::inRange(hsv, cv::Scalar(0, 60, 100), cv::Scalar(12, 255, 190), mask_red_high);
    cv::inRange(hsv, cv::Scalar(160, 25, 55), cv::Scalar(180, 185, 255), mask_red_low);
    cv::bitwise_or(mask_red_low, mask_red_high, mask);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(51, 15)));
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1, -1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 1500) continue;
        cv::Rect bbox = cv::boundingRect(c);
        float ratio = static_cast<float>(bbox.width) / bbox.height;
        if (bbox.y > hsv.rows * 0.78f || bbox.width < hsv.cols * 0.20f) continue;
        if (bbox.height > hsv.rows * 0.50f) continue;   // 高度上限防暗带误报
        if (ratio > 2.5f) {
            result.found = true;
            result.cx  = (bbox.x + bbox.width / 2.0f - hsv.cols / 2.0f) / (hsv.cols / 2.0f);
            result.box = bbox;
            if (bbox.height > 1) result.dist = (0.10f * FOCAL_LEN) / bbox.height;
            return result;
        }
    }

    // 暗色横杆兜底（白平衡偏移导致红色变暗紫/灰）
    cv::Mat dark;
    cv::inRange(hsv, cv::Scalar(0, 0, 35), cv::Scalar(180, 120, 175), dark);
    cv::morphologyEx(dark, dark, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(41, 11)));
    cv::dilate(dark, dark, cv::Mat(), cv::Point(-1, -1), 1);
    contours.clear();
    cv::findContours(dark, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& c : contours) {
        cv::Rect bbox = cv::boundingRect(c);
        double area = cv::contourArea(c);
        if (area < hsv.cols * hsv.rows * 0.02) continue;
        if (bbox.y > hsv.rows * 0.65f || bbox.width < hsv.cols * 0.40f) continue;
        float ratio = static_cast<float>(bbox.width) / std::max(1, bbox.height);
        if (ratio < 3.0f || bbox.height > hsv.rows * 0.35f) continue;
        result.found = true;
        result.cx  = (bbox.x + bbox.width / 2.0f - hsv.cols / 2.0f) / (hsv.cols / 2.0f);
        result.dist = (0.10f * FOCAL_LEN) / std::max(1, bbox.height);
        return result;
    }

    // 行覆盖率兜底（横杆与背景粘连时）
    for (int y = 0; y < static_cast<int>(hsv.rows * 0.65f); ++y) {
        if (cv::countNonZero(dark.row(y)) < hsv.cols * 0.35f) continue;
        int y2 = y;
        while (y2 < static_cast<int>(hsv.rows * 0.65f) &&
               cv::countNonZero(dark.row(y2)) >= hsv.cols * 0.35f) ++y2;
        if (y2 - y < hsv.rows * 0.02f || y2 - y > hsv.rows * 0.35f) { y = y2; continue; }
        std::vector<cv::Point> pts;
        cv::findNonZero(dark.rowRange(y, y2), pts);
        if (pts.empty()) { y = y2; continue; }
        cv::Rect bbox = cv::boundingRect(pts);
        bbox.y += y;
        if (bbox.width >= hsv.cols * 0.35f) {
            result.found = true;
            result.cx  = (bbox.x + bbox.width / 2.0f - hsv.cols / 2.0f) / (hsv.cols / 2.0f);
            result.box = bbox;
            result.dist = (0.10f * FOCAL_LEN) / std::max(1, bbox.height);
            return result;
        }
        y = y2;
    }
    return result;
}

// 可乐瓶：蓝黑瓶身（CV 回退，真机主要走远程 YOLO）
YoloDetector::CircleResult YoloDetector::find_coke(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(95, 20, 55), cv::Scalar(115, 60, 190), mask);
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return result;

    double max_area = 0; size_t max_idx = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        double a = cv::contourArea(contours[i]);
        if (a > max_area) { max_area = a; max_idx = i; }
    }
    if (max_area < 500) return result;
    cv::Rect bbox = cv::boundingRect(contours[max_idx]);
    if (bbox.height < bbox.width * 1.5f) return result;
    if (bbox.width < 30 || bbox.width > 350) return result;
    const double fill = max_area / (static_cast<double>(bbox.width) * bbox.height);
    if (fill < 0.40) return result;

    result.found = true;
    result.cx  = (bbox.x + bbox.width / 2.0f - hsv.cols / 2.0f) / (hsv.cols / 2.0f);
    result.box = bbox;
    if (bbox.width > 1) result.dist = (0.10f * FOCAL_LEN) / bbox.width;
    return result;
}

// 足球：黑白球体（白色圆 + 圆内黑斑比例过滤纯白墙；CV 回退）
YoloDetector::CircleResult YoloDetector::find_football(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 140), cv::Scalar(180, 60, 255), mask);
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 300) continue;
        double perimeter = cv::arcLength(c, true);
        double circularity = 4 * M_PI * area / (perimeter * perimeter);
        if (circularity < 0.6f) continue;

        cv::Point2f center; float radius;
        cv::minEnclosingCircle(c, center, radius);

        // 黑白相间：圆内黑色(V<90)比例>1.5%（亮光下黑花纹被冲淡，阈值放低）
        cv::Mat black_mask, circ, black_in_circ;
        cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 90), black_mask);
        cv::circle(circ, center, static_cast<int>(radius * 0.85f), 255, -1);
        cv::bitwise_and(black_mask, circ, black_in_circ);
        const double black_cnt = cv::countNonZero(black_in_circ);
        const double circ_px = CV_PI * (radius * 0.85f) * (radius * 0.85f);
        if (circ_px > 1.0 && black_cnt / circ_px < 0.015) continue;

        float r = std::sqrt(static_cast<float>(area) / M_PI);
        result.found = true;
        result.cx  = (center.x - hsv.cols / 2.0f) / (hsv.cols / 2.0f);
        result.box = cv::boundingRect(c);
        if (r > 1.0f) result.dist = (BALL_RADIUS * FOCAL_LEN) / r;
        return result;
    }
    return result;
}

// 障碍物：淡蓝色方块（圆度低；只认地面上的，过滤海报/招牌）
YoloDetector::CircleResult YoloDetector::find_obstacle(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(95, 100, 150), cv::Scalar(112, 200, 255), mask);
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1, -1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1, -1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    cv::Rect best_box;
    bool best_found = false;
    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 500) continue;
        cv::Rect bbox = cv::boundingRect(c);
        if ((bbox.y + bbox.height / 2.0) < hsv.rows * 0.30f) continue;
        if (bbox.width < hsv.cols * 0.08f || bbox.height < hsv.rows * 0.08f) continue;
        double perimeter = cv::arcLength(c, true);
        double circularity = 4 * M_PI * area / (perimeter * perimeter);
        if (circularity < 0.90f && area > best_area) {
            best_area = area; best_box = bbox; best_found = true;
        }
    }
    if (best_found) {
        result.found = true;
        result.box = best_box;
        result.cx = (best_box.x + best_box.width / 2.0f - hsv.cols / 2.0f) / (hsv.cols / 2.0f);
        if (best_box.width > 1) result.dist = (0.20f * FOCAL_LEN) / best_box.width;
    }
    return result;
}

// 分隔黄线：地面 ROI 水平扫描跳变计数判实虚
//   实线=每行跳变≤2（一段黄线入+出）；虚线=≥4（多段交替）
YoloDetector::DividerResult YoloDetector::find_divider_line(const cv::Mat& hsv) {
    DividerResult result;
    if (hsv.empty()) return result;

    const int W = hsv.cols, H = hsv.rows;
    const int roi_top = static_cast<int>(H * 0.45f);
    const int roi_bottom = static_cast<int>(H * 0.80f);
    const int roi_h = roi_bottom - roi_top;
    if (roi_h < 20) return result;

    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(14, 70, 70), cv::Scalar(42, 255, 255), mask);
    cv::Mat ker_open  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat ker_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 2));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  ker_open);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, ker_close);

    cv::Mat roi_mask = mask(cv::Rect(0, roi_top, W, roi_h)).clone();
    const int yellow_count = cv::countNonZero(roi_mask);
    if (yellow_count < W * roi_h * 0.005f) return result;

    static const float kScanRatios[3] = {0.25f, 0.50f, 0.75f};
    int total_transitions = 0, rows_used = 0;
    for (float r : kScanRatios) {
        int y = roi_top + static_cast<int>(roi_h * r);
        if (y < 0 || y >= H) continue;
        const uchar* row = mask.ptr<uchar>(y);
        int transitions = 0;
        uchar last = 0;
        const int x0 = static_cast<int>(W * 0.05f);
        const int x1 = static_cast<int>(W * 0.95f);
        for (int x = x0; x <= x1; ++x) {
            uchar cur = (row[x] > 0) ? 1 : 0;
            if (x == x0) { last = cur; continue; }
            if (cur != last) { ++transitions; last = cur; }
        }
        total_transitions += transitions;
        ++rows_used;
    }
    if (rows_used == 0) return result;

    float avg_trans = static_cast<float>(total_transitions) / rows_used;
    result.is_dashed = (avg_trans >= 4.0f);
    bool is_solid = (avg_trans <= 2.5f && total_transitions > 0);
    if (!result.is_dashed && !is_solid) result.is_dashed = (avg_trans >= 3.0f);

    long long sum_x = 0, sum_y = 0;
    int count = 0;
    const int x0c = static_cast<int>(W * 0.05f);
    const int x1c = static_cast<int>(W * 0.95f);
    for (int y = 0; y < roi_h; ++y) {
        const uchar* rp = roi_mask.ptr<uchar>(y);
        for (int x = x0c; x <= x1c; ++x) {
            if (rp[x]) { sum_x += x; sum_y += (y + roi_top); ++count; }
        }
    }
    if (count < 30) return result;
    result.found = true;

    float avg_px = static_cast<float>(sum_x) / count;
    float avg_py = static_cast<float>(sum_y) / count;
    result.cx = (avg_px - W * 0.5f) / (W * 0.5f);
    const float y_ratio = (avg_py - static_cast<float>(roi_top)) / roi_h;
    result.dist = 2.5f - std::max(0.0f, std::min(1.0f, y_ratio)) * 2.2f;
    if (result.dist < 0.05f) result.dist = 0.05f;
    return result;
}

// ═══════════════════════════════════════════════════════════
// RemoteYolo — unix socket 远程推理客户端（真机专用）
//   send_frame: 无未决时发半分辨率 JPEG（懒连接）
//   poll: 有未决时非阻塞收结果；超时 8s 断线重连（推理两模型最慢~4.5s）
// ═══════════════════════════════════════════════════════════
YoloDetector::RemoteYolo::~RemoteYolo() {
    if (fd_ >= 0) ::close(fd_);
}

bool YoloDetector::RemoteYolo::send_frame(const cv::Mat& frame) {
    if (frame.empty()) return false;

    // 懒连接（非阻塞 fd）
    if (fd_ < 0) {
        fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (fd_ < 0) return false;
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", kSockPath);
        if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            ::close(fd_); fd_ = -1;
            return false;
        }
        const int fl = ::fcntl(fd_, F_GETFL, 0);
        ::fcntl(fd_, F_SETFL, fl | O_NONBLOCK);
        pending_ = false;
        rx_len_  = 0;
    }

    if (pending_) return true;   // 有未决请求，本轮不重发（异步节奏）

    cv::Mat small;
    cv::resize(frame, small, cv::Size(), 0.5, 0.5);
    std::vector<uchar> jpg;
    cv::imencode(".jpg", small, jpg, {cv::IMWRITE_JPEG_QUALITY, 80});
    const uint32_t n = static_cast<uint32_t>(jpg.size());
    const uint8_t hdr[4] = {
        static_cast<uint8_t>(n & 0xFF), static_cast<uint8_t>((n >> 8) & 0xFF),
        static_cast<uint8_t>((n >> 16) & 0xFF), static_cast<uint8_t>((n >> 24) & 0xFF) };
    if (!send_all(fd_, hdr, 4) || !send_all(fd_, jpg.data(), jpg.size())) {
        ::close(fd_); fd_ = -1;
        return false;
    }
    pending_ = true;
    sent_at_ = std::chrono::steady_clock::now();
    rx_len_  = 0;
    return true;
}

bool YoloDetector::RemoteYolo::poll(Target& coke, Target& football) {
    if (fd_ >= 0 && pending_) {
        const ssize_t r = ::recv(fd_, rx_buf_ + rx_len_,
                                 sizeof(rx_buf_) - 1 - rx_len_, 0);
        if (r > 0) {
            rx_len_ += static_cast<int>(r);
            rx_buf_[rx_len_] = '\0';
            if (std::strchr(rx_buf_, '\n') != nullptr) {
                pending_ = false;
                rx_len_  = 0;
                int cf = 0, ff = 0;
                float ccx = 0.0f, cd = 0.0f, fcx = 0.0f, fdist = 0.0f;
                int cbx = 0, cby = 0, cbw = 0, cbh = 0, fbx = 0, fby = 0, fbw = 0, fbh = 0;
                float csc = 0.0f, fsc = 0.0f;
                if (std::sscanf(rx_buf_, "coke %d %f %f %d %d %d %d %f;fb %d %f %f %d %d %d %d %f",
                                &cf, &ccx, &cd, &cbx, &cby, &cbw, &cbh, &csc,
                                &ff, &fcx, &fdist, &fbx, &fby, &fbw, &fbh, &fsc) == 16) {
                    coke_cache_.found = (cf != 0);
                    coke_cache_.cx = ccx; coke_cache_.dist = cd;
                    coke_cache_.box = cv::Rect(cbx, cby, cbw, cbh);
                    coke_cache_.conf = csc;
                    fb_cache_.found = (ff != 0);
                    fb_cache_.cx = fcx; fb_cache_.dist = fdist;
                    fb_cache_.box = cv::Rect(fbx, fby, fbw, fbh);
                    fb_cache_.conf = fsc;
                }
            }
        } else if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
            ::close(fd_); fd_ = -1;
            pending_ = false; rx_len_ = 0;
            return false;
        }
        // 未决超时 8s → 断线重连（结果在超时前返回，避免可乐/足球恒为 0）
        if (pending_ &&
            std::chrono::steady_clock::now() - sent_at_ > std::chrono::milliseconds(8000)) {
            ::close(fd_); fd_ = -1;
            pending_ = false; rx_len_ = 0;
            return false;
        }
    }
    coke = coke_cache_;
    football = fb_cache_;
    return true;
}
