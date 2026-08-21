#include "cyberdog_race/vision/virtual/stage4_detector.hpp"
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <vector>
#include <fcntl.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

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
        // 判断是否橙色
        result.ball_is_orange = is_orange(hsv, c.center, c.radius);
        if (!result.ball_is_orange) continue;
        result.ball_found = true;
        result.ball_cx    = c.cx;
        result.ball_dist  = c.dist;
        result.ball_center = c.center;
        result.ball_radius = c.radius;
        break;  // 取最大的球
    }

    // ── 限高杆：红色横向矩形（横杆） ────────────────────────
    auto limbar = find_limbar(hsv);
    result.limbar_found = limbar.found;
    result.limbar_cx    = limbar.cx;
    result.limbar_dist  = limbar.dist;
    result.limbar_box   = limbar.box;

    // ── 可乐/足球: (2026-08-21 接入ONNX模型: 异步发帧给python推理服务, 模型优先/传统CV兜底) ──
    //   remote_detect_async 非阻塞: 无未决请求时发半分辨率JPEG, 结果走最近缓存;
    //   服务不在/连接失败 → 返回false → 纯CV兜底 (2026-08-18 之前: 纯传统CV识别)
    CircleResult coke = find_coke(hsv);
    CircleResult football = find_football(hsv);
    CircleResult rcoke, rfb;
    if (remote_detect_async(frame, rcoke, rfb)) {
        if (rcoke.found) coke = rcoke;            // 模型结果优先(已过服务端阈值)
        if (rfb.found)  football = rfb;
    }
    result.coke_found = coke.found;
    result.coke_cx    = coke.cx;
    result.coke_dist  = coke.dist;
    result.coke_box   = coke.box;
    result.coke_conf  = coke.conf;
    result.football_found = football.found;
    result.football_cx    = football.cx;
    result.football_dist  = football.dist;
    result.football_box   = football.box;
    result.football_conf  = football.conf;

    // ── 障碍物：淡蓝色方块 ──────────────────────────────────
    auto obstacle = find_obstacle(hsv);
    result.obstacle_found = obstacle.found;
    result.obstacle_cx    = obstacle.cx;
    result.obstacle_dist  = obstacle.dist;
    result.obstacle_box   = obstacle.box;   // (2026-08-18 可视化画框用)

    // ── 区域分隔黄线：实/虚判断 ──────────────────────────
    auto divider = find_divider_line(hsv);
    result.divider_found     = divider.found;
    result.divider_cx        = divider.cx;
    result.divider_dist      = divider.dist;
    result.divider_is_dashed = divider.is_dashed;

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
                     100, 25,         // Canny阈值, 累加器阈值 (2026-08-18 30→25 低饱和图更好找)
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

// 限高杆：红色横向矩形（宽高比>3）(2026-08-18 用户: 实物是暗红色(深红, V低),
//   阈值 S≥35 V≥30 H[0,15]+[160,180]; 2026-08-18 H上限20→15: 画面里暗黄色(H~20-35)被误检进去)
//   红色HSV跨两端：H[0,15] S[35,255] V[30,255]  +  H[160,180] S[35,255] V[30,255]
Stage4Detector::CircleResult Stage4Detector::find_limbar(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask_red_low, mask_red_high, mask;
    // (2026-08-20 官方赛场贴边定稿: H主峰178-179 S中位143 V中位198亮红;
    //  去V上限(176会漏75%亮红杆); 去H低段[0,5]→橙球(H中位10 p5=0)深红边缘被误判限高杆)
    cv::inRange(hsv, cv::Scalar(160, 25, 55), cv::Scalar(180, 185, 255), mask_red_low);
    cv::bitwise_or(mask_red_low, mask_red_low, mask);
    // (2026-08-21 用户: 红色连起来大块才认, 小块不要; 闭运算核31x9→51x15连得更开, 面积下限500→1500)
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE,
                     cv::getStructuringElement(cv::MORPH_RECT, cv::Size(51, 15)));
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 1);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 1500) continue;   // (2026-08-21 500→1500: 小块红色噪声不要)
        cv::Rect bbox = cv::boundingRect(c);
        float ratio = (float)bbox.width / bbox.height;
        if (bbox.y > hsv.rows * 0.78f || bbox.width < hsv.cols * 0.20f) continue;
        // (2026-08-18 收紧: 横杆高度不得超过画面1/3, 防整片红色区域误报)
        if (bbox.height > hsv.rows * 0.30f) continue;
        // 横杆：宽度远大于高度
        if (ratio > 3.0f) {
            result.found = true;
            result.cx    = (bbox.x + bbox.width/2.0f - hsv.cols/2.0f) / (hsv.cols/2.0f);
            result.box   = bbox;
            // 用高度估算距离（限高杆截面10cm）
            if (bbox.height > 1) result.dist = (0.10f * FOCAL_LEN) / bbox.height;
            return result;
        }
    }

    // Camera white-balance can shift the red bar toward dark purple/gray.
    // Use a constrained luminance fallback for a broad horizontal bar.
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
        result.cx = (bbox.x + bbox.width / 2.0f - hsv.cols / 2.0f) / (hsv.cols / 2.0f);
        result.dist = (0.10f * FOCAL_LEN) / std::max(1, bbox.height);
        return result;
    }
    // Row-coverage fallback handles a bar merged with the background contour.
    for (int y = 0; y < static_cast<int>(hsv.rows * 0.65f); ++y) {
        const int count = cv::countNonZero(dark.row(y));
        if (count < hsv.cols * 0.35f) continue;
        int y2 = y;
        while (y2 < static_cast<int>(hsv.rows * 0.65f) &&
               cv::countNonZero(dark.row(y2)) >= hsv.cols * 0.35f) ++y2;
        if (y2 - y < hsv.rows * 0.02f) { y = y2; continue; }
        // (2026-08-18 收紧: 整片暗区(高度>1/3画面)不是横杆, 跳过——
        //  这是lim=1 d=0.06m全程误报的直接来源: 暗带无高度上限→670px高整片被当横杆)
        if (y2 - y > hsv.rows * 0.35f) { y = y2; continue; }
        cv::Mat band = dark.rowRange(y, y2);
        std::vector<cv::Point> pts;
        cv::findNonZero(band, pts);
        if (pts.empty()) { y = y2; continue; }
        cv::Rect bbox = cv::boundingRect(pts);
        bbox.y += y;
        if (bbox.width >= hsv.cols * 0.35f) {
            result.found = true;
            result.cx = (bbox.x + bbox.width / 2.0f - hsv.cols / 2.0f) / (hsv.cols / 2.0f);
            result.box = bbox;
            result.dist = (0.10f * FOCAL_LEN) / std::max(1, bbox.height);
            return result;
        }
        y = y2;
    }
    return result;
}

// 可乐瓶：黑色立着的类长方体（V<70，高>宽*1.2）
// (2026-08-18 传统识别: 黑色瓶身直立; V<60→<70 兼容暗环境)
Stage4Detector::CircleResult Stage4Detector::find_coke(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    // (2026-08-20 用户贴边框选定稿: 可乐本体 H中位103/S中位37/V中位75 蓝黑瓶身;
    //  H[95,115] S[20,60] V[55,190], 之前按'黑色'抓全错)
    cv::inRange(hsv, cv::Scalar(95, 20, 55), cv::Scalar(115, 60, 190), mask);
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
    // (2026-08-20 用户贴边定稿: 可乐178x358 高宽比2.0; 放宽防后段变大丢失)
    //   高宽比≥1.5
    if (bbox.height < bbox.width * 1.5f) return result;
    //   宽度30~350px (后段走近可乐轮廓宽超300)
    if (bbox.width < 30 || bbox.width > 350) return result;
    //   外接矩形填充率≥0.40
    const double fill = max_area / (static_cast<double>(bbox.width) * bbox.height);
    if (fill < 0.40) return result;

    result.found = true;
    result.cx    = (bbox.x + bbox.width/2.0f - hsv.cols/2.0f) / (hsv.cols/2.0f);
    result.box   = bbox;   // (2026-08-18 可视化画框)
    if (bbox.width > 1) result.dist = (0.10f * FOCAL_LEN) / bbox.width;
    return result;
}

// 足球：黑白球体（白色圆 + 圆内黑色花纹→黑白相间, 过滤纯白墙/板）
// (2026-08-18 传统识别: 白圆基础上加圆内黑斑比例判断)
Stage4Detector::CircleResult Stage4Detector::find_football(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    // (2026-08-20 用户贴边框选: 足球白+蓝花纹 白区S中位9/V254, 蓝纹S p95=54;
    //  S上限30→60覆盖蓝纹防轮廓碎; V>140保持)
    cv::inRange(hsv, cv::Scalar(0, 0, 140), cv::Scalar(180, 60, 255), mask);
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) return result;

    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 300) continue;
        // 圆度检测，过滤墙壁等非圆形白色区域
        double perimeter = cv::arcLength(c, true);
        double circularity = 4 * M_PI * area / (perimeter * perimeter);
        if (circularity < 0.6f) continue;  // (2026-08-20 0.7→0.6: 手机视频足球+白网连体圆度0.65)

        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(c, center, radius);

        // (2026-08-18 黑白相间: 圆内黑色(V<90)像素比例>4% → 黑白球体, 过滤纯白墙/板)
        // (2026-08-20 4%→1.5%: 亮光场景黑花纹被冲淡, 实测0.013)
        cv::Mat black_mask;
        cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 90), black_mask);
        cv::Mat circ = cv::Mat::zeros(hsv.size(), CV_8UC1);
        cv::circle(circ, center, static_cast<int>(radius * 0.85f), 255, -1);
        cv::Mat black_in_circ;
        cv::bitwise_and(black_mask, circ, black_in_circ);
        const double black_cnt = cv::countNonZero(black_in_circ);
        const double circ_px = CV_PI * (radius * 0.85f) * (radius * 0.85f);
        if (circ_px > 1.0 && black_cnt / circ_px < 0.015) continue;

        float r = std::sqrt(static_cast<float>(area) / M_PI);

        result.found = true;
        result.cx    = (center.x - hsv.cols/2.0f) / (hsv.cols/2.0f);
        result.box   = cv::boundingRect(c);   // (2026-08-18 可视化)
        if (r > 1.0f) result.dist = (BALL_RADIUS * FOCAL_LEN) / r;
        return result;
    }
    return result;
}

// 障碍物：淡蓝色方块（圆度低）
// (2026-08-21 用户: 阈值放开一点, 淡蓝色方形也要; 原 H[99,106] S[135,200] V[200,255] 太紧)
Stage4Detector::CircleResult Stage4Detector::find_obstacle(const cv::Mat& hsv) {
    CircleResult result;
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(95, 100, 150), cv::Scalar(112, 200, 255), mask);
    cv::erode(mask,  mask, cv::Mat(), cv::Point(-1,-1), 2);
    cv::dilate(mask, mask, cv::Mat(), cv::Point(-1,-1), 2);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    double best_area = 0.0;
    cv::Rect best_box;
    bool best_found = false;
    for (auto& c : contours) {
        double area = cv::contourArea(c);
        if (area < 500) continue;
        cv::Rect bbox = cv::boundingRect(c);
        // (2026-08-20 视频验证: 墙上海报/招牌误报→只认地面上的障碍物)
        // (2026-08-20 修复: 用轮廓中心y判断, 大障碍物顶部超过0.30线也会被顶点判断误杀)
        if ((bbox.y + bbox.height / 2.0) < hsv.rows * 0.30f) continue;
        if (bbox.width < hsv.cols * 0.08f || bbox.height < hsv.rows * 0.08f) continue;
        double perimeter = cv::arcLength(c, true);
        double circularity = 4 * M_PI * area / (perimeter * perimeter);
        // 方块圆度 < 0.85
        if (circularity < 0.90f && area > best_area) {
            best_area = area;
            best_box = bbox;
            best_found = true;
        }
    }
    if (best_found) {
        result.found = true;
        result.box = best_box;   // (2026-08-18 可视化画框用)
        result.cx = (best_box.x + best_box.width / 2.0f - hsv.cols / 2.0f) / (hsv.cols / 2.0f);
        if (best_box.width > 1) result.dist = (0.20f * FOCAL_LEN) / best_box.width;
    }
    return result;
}

// ═══════════════════════════════════════════════════════════
// 模型加载（路径为空则不加载，回退颜色检测）
// ═══════════════════════════════════════════════════════════
void Stage4Detector::set_coke_model(const std::string& path, int target_class_id) {
    coke_model_path_       = path;
    coke_target_class_id_  = target_class_id;
    coke_model_loaded_     = false;
    if (path.empty()) return;

    try {
        coke_net_ = cv::dnn::readNetFromONNX(path);
        coke_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        coke_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        coke_model_loaded_ = true;
    } catch (const cv::Exception&) {
        coke_model_loaded_ = false;
    }
}

void Stage4Detector::set_football_model(const std::string& path, int target_class_id) {
    football_model_path_       = path;
    football_target_class_id_  = target_class_id;
    football_model_loaded_     = false;
    if (path.empty()) return;

    try {
        football_net_ = cv::dnn::readNetFromONNX(path);
        football_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
        football_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
        football_model_loaded_ = true;
    } catch (const cv::Exception&) {
        football_model_loaded_ = false;
    }
}

// ═══════════════════════════════════════════════════════════
// 通用 YOLO ONNX 推理（YOLOv5/v8 自适应后处理）
// 输出格式自动适配：
//   v5: [1, N, 5+nc] 有 obj → row = [cx, cy, w, h, obj, cls_scores...]
//   v8: [1, 4+nc, N] 无 obj  → row = [cx, cy, w, h, cls_scores...]
// ═══════════════════════════════════════════════════════════
Stage4Detector::CircleResult Stage4Detector::detect_yolo(
    const cv::dnn::Net& net, const cv::Mat& frame_bgr,
    int input_size, float conf_thresh, float nms_thresh,
    int target_class_id, float real_size) {

    CircleResult result;
    if (frame_bgr.empty() || input_size <= 0) return result;

    // 1. 预处理：resize + 归一化到 [0,1]
    cv::Mat blob = cv::dnn::blobFromImage(
        frame_bgr, 1.0 / 255.0,
        cv::Size(input_size, input_size),
        cv::Scalar(), true, false, CV_32F);
    const_cast<cv::dnn::Net&>(net).setInput(blob);

    // 2. 前向推理
    std::vector<cv::Mat> outputs;
    const_cast<cv::dnn::Net&>(net).forward(outputs, net.getUnconnectedOutLayersNames());
    if (outputs.empty()) return result;
    cv::Mat out = outputs[0];

    // 3. 输出形状统一为 2D [N, C]
    cv::Mat det;
    if (out.dims == 3) {
        int d1 = out.size[1], d2 = out.size[2];
        if (d1 < d2) {
            cv::Mat tmp = out.reshape(1, d1);
            cv::transpose(tmp, det);
        } else {
            det = out.reshape(1, d1);
        }
    } else {
        det = out;
    }
    if (det.cols < 5) return result;

    // 4. 解析检测框
    const bool has_obj = (det.cols >= 6);
    std::vector<cv::Rect> boxes;
    std::vector<float>    confs;
    const float sx = static_cast<float>(frame_bgr.cols) / input_size;
    const float sy = static_cast<float>(frame_bgr.rows) / input_size;

    for (int i = 0; i < det.rows; ++i) {
        const float* row = det.ptr<float>(i);
        float obj = has_obj ? row[4] : 1.0f;

        float max_score = 0.0f; int max_id = -1;
        const int cls_offset = has_obj ? 5 : 4;
        for (int c = cls_offset; c < det.cols; ++c) {
            float s = row[c] * obj;
            if (s > max_score) { max_score = s; max_id = c - cls_offset; }
        }
        if (max_id != target_class_id || max_score < conf_thresh) continue;

        float cx = row[0] * sx, cy = row[1] * sy;
        float w  = row[2] * sx, h  = row[3] * sy;
        boxes.emplace_back(cv::Rect(
            static_cast<int>(cx - w / 2), static_cast<int>(cy - h / 2),
            static_cast<int>(w),           static_cast<int>(h)));
        confs.push_back(max_score);
    }
    if (boxes.empty()) return result;

    // 5. NMS 去重
    std::vector<int> idx;
    cv::dnn::NMSBoxes(boxes, confs, conf_thresh, nms_thresh, idx);
    if (idx.empty()) return result;

    // 6. 取最高置信度框
    int best = idx[0];
    for (int i : idx) if (confs[i] > confs[best]) best = i;

    const cv::Rect& b = boxes[best];
    result.found  = true;
    result.center = cv::Point2f(b.x + b.width / 2.0f, b.y + b.height / 2.0f);
    result.radius = b.width / 2.0f;
    result.cx     = (result.center.x - frame_bgr.cols / 2.0f) / (frame_bgr.cols / 2.0f);
    if (b.height > 1) {
        result.dist = (real_size * FOCAL_LEN) / b.height;
    }
    return result;
}

// ── 可乐模型推理（标签 "c"，单类模型 target_class_id=0） ──
Stage4Detector::CircleResult Stage4Detector::find_coke_by_model(const cv::Mat& frame_bgr) {
    if (!coke_model_loaded_) return CircleResult{};
    return detect_yolo(coke_net_, frame_bgr,
                       kCokeInputSize, kCokeConfThresh, kCokeNmsThresh,
                       coke_target_class_id_, kCokeRealHeight);
}

// ── 足球模型推理（标签 "soccer"，单类模型 target_class_id=0） ──
Stage4Detector::CircleResult Stage4Detector::find_football_by_model(const cv::Mat& frame_bgr) {
    if (!football_model_loaded_) return CircleResult{};
    return detect_yolo(football_net_, frame_bgr,
                       kFootballInputSize, kFootballConfThresh, kFootballNmsThresh,
                       football_target_class_id_, kFootballRealSize);
}

// ═══════════════════════════════════════════════════════════
// NX本地python推理 (2026-08-17): unix socket → s4_detect_server.py
//   请求=4字节LE长度+JPEG; 响应="coke f cx dist x y w h conf;fb ..."
//   异步: 推理640两模型≈750ms, 阻塞会饿死control_loop(303断供→狗失稳)
//   → 发帧不阻塞, 下一轮detect非阻塞读回; 结果用最近缓存
// ═══════════════════════════════════════════════════════════
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

bool Stage4Detector::remote_detect_async(const cv::Mat& frame,
                                         CircleResult& coke, CircleResult& football) {
    if (frame.empty()) return false;

    // ── 懒连接(非阻塞fd) ──
    if (remote_fd_ < 0) {
        remote_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
        if (remote_fd_ < 0) return false;
        sockaddr_un addr{};
        addr.sun_family = AF_UNIX;
        std::snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", kRemoteSockPath);
        if (::connect(remote_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
            ::close(remote_fd_); remote_fd_ = -1;
            return false;
        }
        const int fl = ::fcntl(remote_fd_, F_GETFL, 0);
        ::fcntl(remote_fd_, F_SETFL, fl | O_NONBLOCK);
        remote_pending_ = false;
        remote_rx_len_ = 0;
    }

    if (remote_pending_) {
        // ── 有未决请求: 非阻塞收结果 ──
        const ssize_t r = ::recv(remote_fd_, remote_rx_buf_ + remote_rx_len_,
                                 sizeof(remote_rx_buf_) - 1 - remote_rx_len_, 0);
        if (r > 0) {
            remote_rx_len_ += static_cast<int>(r);
            remote_rx_buf_[remote_rx_len_] = '\0';
            if (std::strchr(remote_rx_buf_, '\n') != nullptr) {
                remote_pending_ = false;
                remote_rx_len_ = 0;
                // 解析 "coke f cx dist x y w h conf;fb ..."
                int cf = 0, ff = 0;
                float ccx = 0.0f, cd = 0.0f, fcx = 0.0f, fd = 0.0f;
                int cbx = 0, cby = 0, cbw = 0, cbh = 0, fbx = 0, fby = 0, fbw = 0, fbh = 0;
                float csc = 0.0f, fsc = 0.0f;
                if (std::sscanf(remote_rx_buf_, "coke %d %f %f %d %d %d %d %f;fb %d %f %f %d %d %d %d %f",
                                &cf, &ccx, &cd, &cbx, &cby, &cbw, &cbh, &csc,
                                &ff, &fcx, &fd, &fbx, &fby, &fbw, &fbh, &fsc) == 16) {
                    remote_coke_cache_.found = (cf != 0);
                    remote_coke_cache_.cx = ccx; remote_coke_cache_.dist = cd;
                    remote_coke_cache_.box = cv::Rect(cbx, cby, cbw, cbh);
                    remote_coke_cache_.conf = csc;
                    remote_fb_cache_.found = (ff != 0);
                    remote_fb_cache_.cx = fcx; remote_fb_cache_.dist = fd;
                    remote_fb_cache_.box = cv::Rect(fbx, fby, fbw, fbh);
                    remote_fb_cache_.conf = fsc;
                }
            }
        } else if (r == 0 || (r < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
            ::close(remote_fd_); remote_fd_ = -1;
            remote_pending_ = false; remote_rx_len_ = 0;
            return false;
        }
        // 未决超时 (2026-08-18 3s→8s: NX python推理两模型串行实测3.2~4.5s,
        //  3s超时→C++断线重连→服务端算完sendall时连接已断→结果永远丢弃,
        //  可乐/足球全程coke=0 fb=0的直接根因; 8s覆盖最慢4.5s留足余量) → 断线重连
        if (remote_pending_ &&
            std::chrono::steady_clock::now() - remote_sent_at_ > std::chrono::milliseconds(8000)) {
            ::close(remote_fd_); remote_fd_ = -1;
            remote_pending_ = false; remote_rx_len_ = 0;
            return false;
        }
    } else {
        // ── 无未决请求: 半分辨率JPEG发帧 ──
        cv::Mat small;
        cv::resize(frame, small, cv::Size(), 0.5, 0.5);
        std::vector<uchar> jpg;
        cv::imencode(".jpg", small, jpg, {cv::IMWRITE_JPEG_QUALITY, 80});
        const uint32_t n = static_cast<uint32_t>(jpg.size());
        const uint8_t hdr[4] = { static_cast<uint8_t>(n & 0xFF),
                                 static_cast<uint8_t>((n >> 8) & 0xFF),
                                 static_cast<uint8_t>((n >> 16) & 0xFF),
                                 static_cast<uint8_t>((n >> 24) & 0xFF) };
        if (!send_all(remote_fd_, hdr, 4) || !send_all(remote_fd_, jpg.data(), jpg.size())) {
            ::close(remote_fd_); remote_fd_ = -1;
            return false;
        }
        remote_pending_ = true;
        remote_sent_at_ = std::chrono::steady_clock::now();
        remote_rx_len_ = 0;
    }

    // ── 返回最近缓存(首次请求前为默认空) ──
    coke = remote_coke_cache_;
    football = remote_fb_cache_;
    return true;
}

// ═══════════════════════════════════════════════════════════
// 区域分隔黄线检测（竖向通道之间的分隔线）
// 算法：HSV 黄色阈值 → 取地面 ROI → 水平扫描跳变计数判实虚
//   - 实线：连续黄色像素段，扫描行跳变 ≤ 2 次（入线+出线）
//   - 虚线：多段黄色+间隙，扫描行跳变 ≥ 4 次（多段交替）
// ═══════════════════════════════════════════════════════════
Stage4Detector::DividerResult Stage4Detector::find_divider_line(const cv::Mat& hsv) {
    DividerResult result;
    if (hsv.empty()) return result;

    const int W = hsv.cols, H = hsv.rows;

    // 1. ROI：地面中下部（不看远处，避免误判）
    const int roi_top    = static_cast<int>(H * 0.45f);
    const int roi_bottom = static_cast<int>(H * 0.80f);
    const int roi_h = roi_bottom - roi_top;
    if (roi_h < 20) return result;

    // 2. HSV 黄色阈值（稍宽于 LaneDetector，适配阴影下的黄线）
    cv::Mat mask;
    cv::inRange(hsv,
        cv::Scalar(14, 70, 70),
        cv::Scalar(42, 255, 255),
        mask);

    // 3. 形态学：开运算去噪 + 闭运算补齐小断口（但不过度破坏虚线间隙）
    cv::Mat ker_open  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::Mat ker_close = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 2));
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  ker_open);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, ker_close);

    // 4. 只保留 ROI
    cv::Mat roi_mask = mask(cv::Rect(0, roi_top, W, roi_h)).clone();

    // 5. 统计 ROI 内黄色像素总量（不够 → 没检测到）
    const int yellow_count = cv::countNonZero(roi_mask);
    const int total_pixels = W * roi_h;
    if (yellow_count < total_pixels * 0.005f) return result;  // <0.5% 忽略

    // 6. 取 3 条水平扫描行（ROI 1/4、1/2、3/4 位置），统计每行跳变次数
    static const float kScanRatios[3] = {0.25f, 0.50f, 0.75f};
    int total_transitions = 0;
    int rows_used = 0;

    for (float r : kScanRatios) {
        int y = roi_top + static_cast<int>(roi_h * r);
        if (y < 0 || y >= H) continue;
        const uchar* row = mask.ptr<uchar>(y);

        int transitions = 0;
        uchar last = 0;
        // 只统计 [W*0.05, W*0.95] 区间（避开边缘噪声）
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

    // 7. 实虚判断：平均每行跳变次数
    //   实线：≤2 次/行（一段连续黄线：入+出）
    //   虚线：≥4 次/行（至少 2 段黄+2 间隙交替）
    float avg_trans = static_cast<float>(total_transitions) / rows_used;
    result.is_dashed = (avg_trans >= 4.0f);
    // 实线阈值：≤2.5 次/行（允许少量噪声跳变）
    bool is_solid = (avg_trans <= 2.5f && total_transitions > 0);
    if (!result.is_dashed && !is_solid) {
        // 中间态（2.5~4）：按虚线偏保守，允许借道（宁误判虚线不误判实线）
        result.is_dashed = (avg_trans >= 3.0f);
    }

    // 8. cx：ROI 内所有黄色像素列坐标加权平均
    //    同时估算 y 平均位置 → 用于单目测距
    long long sum_x = 0, sum_y = 0;
    int count = 0;
    const int x0c = static_cast<int>(W * 0.05f);
    const int x1c = static_cast<int>(W * 0.95f);
    for (int y = 0; y < roi_h; ++y) {
        const uchar* rp = roi_mask.ptr<uchar>(y);
        for (int x = x0c; x <= x1c; ++x) {
            if (rp[x]) {
                sum_x += x;
                sum_y += (y + roi_top);
                ++count;
            }
        }
    }
    if (count < 30) return result;  // 像素太少，丢弃
    result.found = true;

    float avg_px = static_cast<float>(sum_x) / count;
    float avg_py = static_cast<float>(sum_y) / count;
    result.cx = (avg_px - W * 0.5f) / (W * 0.5f);

    // 9. 距离估算：基于 y 平均位置反比（越靠近画面底部=越近）
    //    相机 pitch≈0 时，y_bottom(H*0.95) ≈ 0.25m, y_top(H*0.45) ≈ 2.5m
    const float y_ratio = (avg_py - static_cast<float>(roi_top)) / roi_h;
    result.dist = 2.5f - std::max(0.0f, std::min(1.0f, y_ratio)) * 2.2f;  // [0.3, 2.5]m
    if (result.dist < 0.05f) result.dist = 0.05f;

    return result;
}
