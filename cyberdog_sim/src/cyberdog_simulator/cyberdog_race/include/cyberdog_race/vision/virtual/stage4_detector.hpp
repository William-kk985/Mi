#pragma once
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>
#include <string>

struct Stage4Result {
    // 限高杆（灰色横杆，蹲下通过）
    bool  limbar_found{false};
    float limbar_cx{0.0f};
    float limbar_dist{0.0f};
    cv::Rect limbar_box;         // (2026-08-17 颜色检测bbox, 供可视化画框)

    // 可乐瓶（黑色圆柱，撞倒）
    bool  coke_found{false};
    float coke_cx{0.0f};
    float coke_dist{0.0f};
    cv::Rect coke_box;           // (2026-08-17 python回传真实框, 供可视化)
    float coke_conf{0.0f};

    // 橙色球（形状优先+颜色判断）
    bool  ball_found{false};
    bool  ball_is_orange{false};
    float ball_cx{0.0f};
    float ball_dist{0.0f};
    cv::Point2f ball_center;     // (2026-08-17 供可视化画圆)
    float ball_radius{0.0f};

    // 足球（白色圆球）
    bool  football_found{false};
    float football_cx{0.0f};
    float football_dist{0.0f};
    cv::Rect football_box;       // (2026-08-17 python回传真实框)
    float football_conf{0.0f};

    // 障碍物（淡蓝色方块，绕行）
    bool  obstacle_found{false};
    float obstacle_cx{0.0f};
    float obstacle_dist{0.0f};
    cv::Rect obstacle_box;      // (2026-08-18 可视化画框用)

    // 区域分隔黄线（实/虚判断，用于借道决策）
    bool  divider_found{false};
    float divider_cx{0.0f};
    float divider_dist{0.0f};
    bool  divider_is_dashed{false};
};

class Stage4Detector {
public:
    Stage4Detector() = default;
    Stage4Result detect(const cv::Mat& frame);

    // ── 模型接入（用户提供 ONNX 模型后填路径） ──
    // 调用 set_xxx_model(路径) 加载；路径为空则不加载，回退到颜色检测。
    // 模型格式：YOLOv5/v8 ONNX，后处理自适应 v5(有obj)/v8(无obj)。
    // target_class_id：目标类别在模型类别表中的索引。
    //   - 单类模型（只训练了目标本身）填 0
    //   - 多类模型需查训练时 data.yaml 的 names 顺序确定索引
    void set_coke_model(const std::string& path, int target_class_id = 0);
    void set_football_model(const std::string& path, int target_class_id = 0);
    bool coke_model_ready()     const { return coke_model_loaded_; }
    bool football_model_ready() const { return football_model_loaded_; }

private:
    static constexpr float FOCAL_LEN   = 402.0f;
    static constexpr float BALL_RADIUS = 0.10f;

    // (2026-08-18 真机gc02m1低饱和相机(S≈25): S≥100→50→30, V 100→60→50,
    //  实测S=50仍判不过橙球, 放宽到30; H 0-30)
    static constexpr int ORANGE_H_LOW  = 0;
    static constexpr int ORANGE_H_HIGH = 30;
    static constexpr int ORANGE_S_MIN  = 30;
    static constexpr int ORANGE_V_MIN  = 50;

    struct CircleResult {
        bool  found{false};
        float cx{0.0f};
        float dist{0.0f};
        float radius{0.0f};
        cv::Point2f center;
        cv::Rect box;            // (2026-08-17 模型框像素坐标)
        float conf{0.0f};
    };

    // 区域分隔黄线检测结果
    struct DividerResult {
        bool  found{false};
        float cx{0.0f};            // 画面x归一化 [-1,1]
        float dist{0.0f};          // 估计距离 (m)
        bool  is_dashed{false};    // true=虚线可借道, false=实线
    };

    // (2026-08-18 max_r 100→150: 1280x960下近距球(距<0.37m)半径>100px被过滤不识别)
    std::vector<CircleResult> find_all_circles(const cv::Mat& gray, int min_r=10, int max_r=150);
    bool is_orange(const cv::Mat& hsv, const cv::Point2f& center, float radius);
    CircleResult find_limbar(const cv::Mat& hsv);    // 限高杆：灰色横向矩形
    CircleResult find_coke(const cv::Mat& hsv);      // 可乐瓶：黑色圆柱（颜色检测回退路径）
    CircleResult find_football(const cv::Mat& hsv);  // 足球：白色圆球（颜色检测回退路径）
    CircleResult find_obstacle(const cv::Mat& hsv);  // 障碍物：蓝色方块
    DividerResult find_divider_line(const cv::Mat& hsv); // 区域分隔黄线：HSV+跳变计数判实虚

    // ── 通用 YOLO ONNX 推理（v5/v8 自适应后处理） ──
    // net: 已加载的 DNN 网络；frame_bgr: 原图；input_size: 模型输入边长
    // conf/nms: 置信度与 NMS 阈值；target_class_id: 目标类别索引
    // real_size: 目标实际尺寸(m)，用 bbox 高度做单目测距
    CircleResult detect_yolo(const cv::dnn::Net& net, const cv::Mat& frame_bgr,
                             int input_size, float conf_thresh, float nms_thresh,
                             int target_class_id, float real_size);

    // ── 模型推理封装（调 detect_yolo） ──
    CircleResult find_coke_by_model(const cv::Mat& frame_bgr);
    CircleResult find_football_by_model(const cv::Mat& frame_bgr);

    // ── NX本地python推理服务 (2026-08-17): 系统OpenCV4.1.1加载不了新opset模型,
    //   可乐/足球经unix socket发给本地python(onnxruntime)推理, 结果回填
    //   异步: 推理~750ms不能阻塞on_rgb, 发帧不阻塞, 下轮detect读回 ──
    static constexpr const char* kRemoteSockPath = "/tmp/s4_detect.sock";
    int  remote_fd_{-1};
    bool remote_pending_{false};
    char remote_rx_buf_[256]{};
    int  remote_rx_len_{0};
    CircleResult remote_coke_cache_;
    CircleResult remote_fb_cache_;
    std::chrono::steady_clock::time_point remote_sent_at_{};
    bool remote_detect_async(const cv::Mat& frame, CircleResult& coke, CircleResult& football);

    // ── 可乐模型参数 ──
    // 标签名 "c"（来自训练 data.yaml）。若模型为单类，target_class_id=0；
    // 若多类，按 data.yaml 的 names 顺序填索引。
    static constexpr int   kCokeInputSize   = 640;
    static constexpr float kCokeConfThresh  = 0.45f;
    static constexpr float kCokeNmsThresh   = 0.45f;
    static constexpr float kCokeRealHeight  = 0.30f;   // 可乐瓶实际高度(m)

    // ── 足球模型参数 ──
    // 标签名 "soccer"（来自训练 data.yaml）。
    static constexpr int   kFootballInputSize  = 640;
    static constexpr float kFootballConfThresh = 0.45f;
    static constexpr float kFootballNmsThresh  = 0.45f;
    static constexpr float kFootballRealSize   = 0.22f;  // 足球直径(m)

    // 可乐模型状态
    std::string   coke_model_path_;
    cv::dnn::Net  coke_net_;
    bool          coke_model_loaded_{false};
    int           coke_target_class_id_{0};

    // 足球模型状态
    std::string   football_model_path_;
    cv::dnn::Net  football_net_;
    bool          football_model_loaded_{false};
    int           football_target_class_id_{0};
};
