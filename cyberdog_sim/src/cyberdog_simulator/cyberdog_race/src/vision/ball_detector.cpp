#include "cyberdog_race/vision/ball_detector.hpp"

BallResult BallDetector::detect(const cv::Mat& frame, BallColor target) {
    if (frame.empty()) return {};

    switch (target) {
        case BallColor::ORANGE:
            return find_ball(frame, orange_low_, orange_high_, BallColor::ORANGE);
        case BallColor::BLUE:
            return find_ball(frame, blue_low_, blue_high_, BallColor::BLUE);
        default:
            return {};
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

    // 形态学去噪
    cv::erode(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    // 找最大轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    if (contours.empty()) return result;

    // 取面积最大的轮廓
    size_t max_idx = 0;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); i++) {
        double area = cv::contourArea(contours[i]);
        if (area > max_area) { max_area = area; max_idx = i; }
    }

    if (max_area < 100) return result; // 太小忽略

    // 计算外接圆
    cv::Point2f center;
    float radius;
    cv::minEnclosingCircle(contours[max_idx], center, radius);

    result.found  = true;
    result.color  = color;
    result.cx     = (center.x - frame.cols / 2.0f) / (frame.cols / 2.0f);
    result.cy     = (center.y - frame.rows / 2.0f) / (frame.rows / 2.0f);
    result.radius = radius;

    return result;
}
