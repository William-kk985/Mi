#include "cyberdog_race/vision/virtual/stage4_detector.hpp"
#include <cmath>

Stage4Result Stage4Detector::detect(const cv::Mat& frame) {
    Stage4Result result;
    if (frame.empty()) return result;

    cv::Mat hsv, gray;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // ── 橙色球：形状优先 + 颜色判断 ──────────────────────
    auto circles = find_all_circles(gray);
    for (auto& c : circles) {
        if (!c.found) continue;
        result.ball_found = true;
        result.ball_cx    = c.cx;
        result.ball_dist  = c.dist;
        // 判断是否橙色
        result.ball_is_orange = is_orange(hsv, c.center, c.radius);
        break;  // 取最大的球
    }

    // ── 限高杆：灰色横向矩形 ──────────────────────────────
    auto limbar = find_limbar(hsv);
    result.limbar_found = limbar.found;
    result.limbar_cx    = limbar.cx;
    result.limbar_dist  = limbar.dist;

    // ── 可乐瓶：黑色圆柱 ──────────────────────────────────
    auto coke = find_coke(hsv);
    result.coke_found = coke.found;
    result.coke_cx    = coke.cx;
    result.coke_dist  = coke.dist;

    // ── 足球：白色圆球 ────────────────────────────────────
    auto football = find_football(hsv);
    result.football_found = football.found;
    result.football_cx    = football.cx;
    result.football_dist  = football.dist;

    // ── 障碍物：蓝色方块 ──────────────────────────────────
    auto obstacle = find_obstacle(hsv);
    result.obstacle_found = obstacle.found;
    result.obstacle_cx    = obstacle.cx;
    result.obstacle_dist  = obstacle.dist;

    return result;
}

// 形状优先：找所有圆形物体（不管颜色）
std::vector<Stage4Detector::CircleResult> Stage4Detector::find_all_circles(
    const cv::Mat& gray, int min_r, int max_r) {

    std::vector<CircleResult> results;
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(9, 9), 2);

    std::vector<cv::Vec3f> circles;
    cv::HoughCircles(blurred, circles, cv::HOUGH_GRADIENT, 1,
                     gray.rows / 8,   // 圆心最小间距
                     100, 30,         // Canny阈值, 累加器阈值
                     min_r, max_r);

    for (auto& c : circles) {
        CircleResult r;
        r.found  = true;
        r.center = cv::Point2f(c[0], c[1]);
        r.radius = c[2];
        r.cx     = (c[0] - gray.cols / 2.0f) / (gray.cols / 2.0f);
        if (r.radius > 1.0f) {
            r.dist = (BALL_RADIUS * FOCAL_LEN) / r.radius;
        }
        results.push_back(r);
    }

    // 按半径从大到小排序
    std::sort(results.begin(), results.end(),
              [](const CircleResult& a, const CircleResult& b) {
                  return a.radius > b.radius;
              });
    return results;
}

// 判断圆形区域是否为橙色
bool Stage4Detector::is_orange(const cv::Mat& hsv,
                                const cv::Point2f& center, float radius) {
    // 取圆形区域内的像素
    cv::Mat mask = cv::Mat::zeros(hsv.size(), CV_8UC1);
    cv::circle(mask, center, static_cast<int>(radius * 0.8f), 255, -1);

    cv::Scalar mean = cv::mean(hsv, mask);
    float h = mean[0], s = mean[1], v = mean[2];

    return (h >= ORANGE_H_LOW && h <= ORANGE_H_HIGH
         && s >= ORANGE_S_MIN
         && v >= ORANGE_V_MIN);
}

// 限高杆：灰色横向矩形（S<30, V:80-180，宽高比>3）
Stage4Detector::CircleResult Stage4Detector::find_limbar(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 80), cv::Scalar(180, 30, 180), mask);
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 500) continue;
        cv::Rect bbox = cv::boundingRect(c);
        float ratio = (float)bbox.width / bbox.height;
        // 横杆：宽度远大于高度
        if (ratio > 3.0f) {
            result.found = true;
            result.cx    = (bbox.x + bbox.width/2.0f - hsv.cols/2.0f) / (hsv.cols/2.0f);
            // 用高度估算距离（限高杆截面10cm）
            if (bbox.height > 1) result.dist = (0.10f * FOCAL_LEN) / bbox.height;
            return result;
        }
    }
    return result;
}

// 可乐瓶：黑色圆柱（V<60，高宽比>1.5）
Stage4Detector::CircleResult Stage4Detector::find_coke(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 60), mask);
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return result;

    double max_area = 0;
    size_t max_idx = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double a = cv::contourArea(contours[i]);
        if (a > max_area) { max_area = a; max_idx = i; }
    }
    if (max_area < 500) return result;  // 面积过滤

    cv::Rect bbox = cv::boundingRect(contours[max_idx]);
    // 圆柱：高度 > 宽度 * 1.5
    if (bbox.height < bbox.width * 1.5f) return result;

    result.found = true;
    result.cx    = (bbox.x + bbox.width/2.0f - hsv.cols/2.0f) / (hsv.cols/2.0f);
    if (bbox.width > 1) result.dist = (0.10f * FOCAL_LEN) / bbox.width;
    return result;
}

// 足球：白色圆球（S<40, V>180，圆度>0.8）
Stage4Detector::CircleResult Stage4Detector::find_football(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(0, 0, 180), cv::Scalar(180, 40, 255), mask);
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return result;

    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 200) continue;
        // 圆度检测，过滤墙壁等非圆形白色区域
        double perimeter = cv::arcLength(c, true);
        double circularity = 4 * M_PI * area / (perimeter * perimeter);
        if (circularity < 0.7f) continue;  // 不够圆跳过

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(c, center, radius);
        float r = std::sqrt(static_cast<float>(area) / M_PI);

        result.found = true;
        result.cx    = (center.x - hsv.cols/2.0f) / (hsv.cols/2.0f);
        if (r > 1.0f) result.dist = (BALL_RADIUS * FOCAL_LEN) / r;
        return result;
    }
    return result;
}

// 障碍物：蓝色方块（圆度低）
Stage4Detector::CircleResult Stage4Detector::find_obstacle(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(95, 80, 80), cv::Scalar(125, 255, 255), mask);
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 500) continue;
        double perimeter = cv::arcLength(c, true);
        double circularity = 4 * M_PI * area / (perimeter * perimeter);
        // 方块圆度 < 0.85
        if (circularity < 0.85f) {
            cv::Rect bbox = cv::boundingRect(c);
            result.found = true;
            result.cx    = (bbox.x + bbox.width/2.0f - hsv.cols/2.0f) / (hsv.cols/2.0f);
            if (bbox.width > 1) result.dist = (0.20f * FOCAL_LEN) / bbox.width;
            return result;
        }
    }
    return result;
}
