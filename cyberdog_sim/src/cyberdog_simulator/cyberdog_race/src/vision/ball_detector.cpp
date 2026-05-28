#include "cyberdog_race/vision/ball_detector.hpp"
#include <cmath>

BallResult BallDetector::detect(const cv::Mat& frame, BallColor target) {
    if (frame.empty()) return {};
    switch (target) {
        case BallColor::ORANGE: return find_ball(frame, orange_low_, orange_high_, BallColor::ORANGE);
        case BallColor::BLUE:   return find_ball(frame, blue_low_,   blue_high_,   BallColor::BLUE);
        case BallColor::WHITE:  return find_ball(frame, white_low_,  white_high_,  BallColor::WHITE);
        default: return {};
    }
}

BallResult BallDetector::find_ball(const cv::Mat& frame,
                                    const cv::Scalar& low,
                                    const cv::Scalar& high,
                                    BallColor color) {
    BallResult result;

    cv::Mat hsv, mask;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, low, high, mask);

    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return result;

    size_t max_idx = 0;
    double max_area = 0;

    if (color == BallColor::WHITE) {
        // 白球：遍历所有轮廓，找面积最大且满足圆度的（同 Stage4 find_football）
        for (size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if (area < 200) continue;
            double perimeter = cv::arcLength(contours[i], true);
            double circularity = 4 * M_PI * area / (perimeter * perimeter);
            if (circularity < 0.7f) continue;
            if (area > max_area) { max_area = area; max_idx = i; }
        }
        if (max_area < 200) return result;
    } else {
        for (size_t i = 0; i < contours.size(); i++) {
            double a = cv::contourArea(contours[i]);
            if (a > max_area) { max_area = a; max_idx = i; }
        }
        if (max_area < 100) return result;
    }

    // 面积等效半径（比外接圆稳定，遮挡时外接圆偏大）
    float r_area = std::sqrt(static_cast<float>(max_area) / M_PI);

    cv::Point2f center;
    float r_enc;
    cv::minEnclosingCircle(contours[max_idx], center, r_enc);

    // 近距离时面积半径偏小（边缘漏检），用外接圆半径更准
    // 远距离时外接圆偏大（噪点），用面积半径更准
    // 取两者较大值，近距离时外接圆主导，远距离时面积主导
    float radius = std::max(r_area, r_enc * 0.85f);

    result.found  = true;
    result.color  = color;
    result.cx     = (center.x - frame.cols / 2.0f) / (frame.cols / 2.0f);
    result.cy     = (center.y - frame.rows / 2.0f) / (frame.rows / 2.0f);
    result.radius = radius;

    // 针孔模型估距，用修正后的半径
    if (radius > 1.0f) {
        float raw = (BALL_RADIUS * FOCAL_LEN) / radius;
        dist_filtered_ = 0.6f * dist_filtered_ + 0.4f * raw;
        result.dist_m = dist_filtered_;
    }

    return result;
}
