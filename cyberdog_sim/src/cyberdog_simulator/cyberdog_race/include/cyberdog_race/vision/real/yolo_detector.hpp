#pragma once
#include <chrono>
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>

#include "cyberdog_race/vision/vision_result.hpp"   // YoloResult（vision 公共结果结构）

// ═══════════════════════════════════════════════════════════
// YoloDetector — 真机多目标检测（vision/real）
//   真机用法：可乐/足球 经 unix socket 发本地 python 推理服务
//   (s4_detect_server.py, onnxruntime) 异步回填；限高杆/障碍/橙球/
//   分隔线 用本地 CV（HSV+形状，适配 gc02m1 低饱和相机）。
//   真机专用（仿真已无独立 Stage4 检测器）。
// ═══════════════════════════════════════════════════════════
class YoloDetector {
public:
    YoloDetector() = default;
    // 主入口：本地 CV（限高杆/障碍/橙球/分隔线）+ 远程 YOLO（可乐/足球）
    YoloResult detect(const cv::Mat& frame);

private:
    struct Target {
        bool  found{false};
        float cx{0.0f};
        float dist{0.0f};
        cv::Rect box;
        float conf{0.0f};
    };
    struct CircleResult {
        bool  found{false};
        float cx{0.0f};
        float dist{0.0f};
        float radius{0.0f};
        cv::Point2f center;
        cv::Rect box;
        float conf{0.0f};
    };
    struct DividerResult {
        bool  found{false};
        float cx{0.0f};
        float dist{0.0f};
        bool  is_dashed{false};
    };

    // 单目测距/橙球阈值（gc02m1 低饱和相机 S≈25，阈值放宽）
    static constexpr float FOCAL_LEN   = 402.0f;
    static constexpr float BALL_RADIUS = 0.10f;
    static constexpr int ORANGE_H_LOW  = 0;
    static constexpr int ORANGE_H_HIGH = 30;
    static constexpr int ORANGE_S_MIN  = 30;
    static constexpr int ORANGE_V_MIN  = 50;

    // ── 本地 CV（真机专用） ──
    std::vector<CircleResult> find_all_circles(const cv::Mat& gray, int min_r = 10, int max_r = 150);
    bool is_orange(const cv::Mat& hsv, const cv::Point2f& center, float radius);
    CircleResult find_limbar(const cv::Mat& hsv);    // 限高杆：红色横杆
    CircleResult find_coke(const cv::Mat& hsv);      // 可乐：蓝黑瓶身（CV 回退）
    CircleResult find_football(const cv::Mat& hsv);  // 足球：黑白球体（CV 回退）
    CircleResult find_obstacle(const cv::Mat& hsv);  // 障碍物：淡蓝方块
    DividerResult find_divider_line(const cv::Mat& hsv); // 分隔黄线：跳变计数判实虚

    // ── 远程 YOLO 客户端（unix socket → s4_detect_server.py） ──
    // 协议：请求 = 4字节LE长度 + JPEG；响应 = "coke f cx dist x y w h conf;fb ..."
    // 异步：send_frame 发帧不阻塞，poll 非阻塞收结果，返回最近缓存
    class RemoteYolo {
    public:
        RemoteYolo() = default;
        ~RemoteYolo();
        // 无未决请求时发半分辨率 JPEG；连接失败/已连接均可安全调用
        bool send_frame(const cv::Mat& frame);
        // 有未决请求时非阻塞收结果并写回 coke/football；返回最近缓存
        bool poll(Target& coke, Target& football);
        bool connected() const { return fd_ >= 0; }
    private:
        int   fd_{-1};
        bool  pending_{false};
        char  rx_buf_[256]{};
        int   rx_len_{0};
        std::chrono::steady_clock::time_point sent_at_{};
        Target coke_cache_, fb_cache_;
        static constexpr const char* kSockPath = "/tmp/s4_detect.sock";
    };
    RemoteYolo remote_;
};
