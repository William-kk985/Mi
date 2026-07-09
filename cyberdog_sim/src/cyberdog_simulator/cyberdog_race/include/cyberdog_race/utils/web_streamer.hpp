#pragma once

// ============================================================
// WebStreamer — 嵌入式 MJPEG HTTP 推流服务（双流 + 多客户端）
// 由 ENABLE_WEB_STREAMING 宏控制编译，注释即零开销
//
// 用法：
//   WebStreamer ws;
//   ws.start(8080, 4);              // 启动 HTTP，最多 4 个客户端
//   ws.push_frame(cv::Mat);         // 推原始画面 → /stream
//   ws.push_debug_frame(cv::Mat);   // 推标注画面 → /stream/debug
//   ws.stop();
//
// 浏览器访问：
//   http://<IP>:8080/              →  双流对照页面
//   http://<IP>:8080/stream        →  原始 MJPEG 流
//   http://<IP>:8080/stream/debug  →  标注 MJPEG 流
// ============================================================

#include <opencv2/opencv.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class WebStreamer {
public:
    WebStreamer() = default;
    ~WebStreamer() { stop(); }

    WebStreamer(const WebStreamer&) = delete;
    WebStreamer& operator=(const WebStreamer&) = delete;

    /// 启动 HTTP 服务线程
    /// @param port   监听端口
    /// @param max_clients 最大同时客户端数（默认 4，超出拒绝新连接）
    bool start(int port, int max_clients = 4);

    /// 推送原始帧（线程安全，在 on_rgb 回调中调用）
    void push_frame(const cv::Mat& frame);

    /// 推送调试标注帧（线程安全，叠加了视觉检测结果）
    void push_debug_frame(const cv::Mat& frame);

    /// 推送 LiDAR 俯视图帧（线程安全）
    void push_lidar_frame(const cv::Mat& frame);

    /// 推送里程轨迹图帧（线程安全）
    void push_track_frame(const cv::Mat& frame);

    /// 推送遥测仪表盘帧（线程安全）
    void push_telemetry_frame(const cv::Mat& frame);

    /// 推送 D430i 红外相机帧（线程安全）
    void push_d435_frame(const cv::Mat& frame);

    /// 推送左鱼眼相机帧（灰度，线程安全）
    void push_fisheye_left_frame(const cv::Mat& frame);

    /// 推送右鱼眼相机帧（灰度，线程安全）
    void push_fisheye_right_frame(const cv::Mat& frame);

    /// 推送 D430i 深度图帧（伪彩色映射后，线程安全）
    void push_depth_frame(const cv::Mat& frame);

    /// 推送降暗帧（曝光偏移后，线程安全）→ /stream/dark
    void push_dark_frame(const cv::Mat& frame);

    /// 停止服务，等待所有客户端线程退出
    void stop();

    bool is_running() const { return running_.load(); }

    // ── 相机设置（Web ⚙️ 页面可调，conf 文件持久化） ──
    int  exposure_offset() const { return exposure_offset_; }
    void set_exposure_offset(int v) { exposure_offset_ = v; }
    int  jpeg_quality() const { return jpeg_quality_; }
    void set_jpeg_quality(int v) { jpeg_quality_ = v; }
    void load_settings(const std::string& path = "camera_config.conf");
    void save_settings(const std::string& path = "camera_config.conf");

private:
    void server_loop(int port);
    void client_handler(int client_fd, const std::string& path);
    static std::string make_html_page();

    // ── 线程管理 ──
    std::thread              server_thread_;
    int                      server_fd_{-1};         // P0: 保存 fd 用于 stop() 唤醒 accept
    std::vector<std::thread> client_threads_;
    std::mutex               client_threads_mutex_;
    std::atomic<int>         active_clients_{0};     // P1: 活跃客户端计数

    // ── 六帧缓冲 ──
    std::mutex               frame_mutex_;
    std::condition_variable  frame_cv_;
    std::vector<uint8_t>     jpeg_buffer_;        // 0: raw
    std::vector<uint8_t>     jpeg_debug_buffer_;  // 1: debug
    std::vector<uint8_t>     jpeg_lidar_buffer_;  // 2: lidar
    std::vector<uint8_t>     jpeg_track_buffer_;  // 3: track
    std::vector<uint8_t>     jpeg_telem_buffer_;  // 4: telemetry
    std::vector<uint8_t>     jpeg_d435_buffer_;   // 5: d435 infra1
    std::vector<uint8_t>     jpeg_dark_buffer_;   // 6: dark
    std::vector<uint8_t>     jpeg_fisheye_left_buffer_;  // 7: fisheye_left
    std::vector<uint8_t>     jpeg_fisheye_right_buffer_; // 8: fisheye_right
    std::vector<uint8_t>     jpeg_depth_buffer_;         // 9: depth colormap
    uint64_t                 frame_seq_{0};
    uint64_t                 debug_frame_seq_{0};
    uint64_t                 lidar_frame_seq_{0};
    uint64_t                 track_frame_seq_{0};
    uint64_t                 telem_frame_seq_{0};
    uint64_t                 d435_frame_seq_{0};
    uint64_t                 dark_frame_seq_{0};
    uint64_t                 fisheye_left_frame_seq_{0};
    uint64_t                 fisheye_right_frame_seq_{0};
    uint64_t                 depth_frame_seq_{0};
    bool                     has_frame_{false};
    bool                     has_debug_frame_{false};
    bool                     has_lidar_frame_{false};
    bool                     has_track_frame_{false};
    bool                     has_telem_frame_{false};
    bool                     has_d435_frame_{false};
    bool                     has_dark_frame_{false};
    bool                     has_fisheye_left_frame_{false};
    bool                     has_fisheye_right_frame_{false};
    bool                     has_depth_frame_{false};

    // ── 相机设置 ──
    int  exposure_offset_{-30};
    int  jpeg_quality_{70};

    // ── 控制 ──
    std::atomic<bool> running_{false};
    int max_clients_{4};
};
