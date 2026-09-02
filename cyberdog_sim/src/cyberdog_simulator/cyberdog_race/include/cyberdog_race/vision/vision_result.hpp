#pragma once
#include <opencv2/opencv.hpp>

// ═══ YoloResult — vision 公共多目标检测结果 ═══
// 真机 yolo_detector（远程 YOLO + CV）使用；字段名与写入 sensor_.xxx 的代码对齐。

struct YoloResult {
    // 限高杆（红色横杆）
    bool  limbar_found{false};
    float limbar_cx{0.0f};
    float limbar_dist{0.0f};
    cv::Rect limbar_box;

    // 可乐瓶（真机=远程 YOLO；仿真=CV 回退）
    bool  coke_found{false};
    float coke_cx{0.0f};
    float coke_dist{0.0f};
    cv::Rect coke_box;
    float coke_conf{0.0f};

    // 橙色球（CV：形状+颜色）
    bool  ball_found{false};
    bool  ball_is_orange{false};
    float ball_cx{0.0f};
    float ball_dist{0.0f};
    cv::Point2f ball_center;
    float ball_radius{0.0f};

    // 足球（真机=远程 YOLO；仿真=CV 回退）
    bool  football_found{false};
    float football_cx{0.0f};
    float football_dist{0.0f};
    cv::Rect football_box;
    float football_conf{0.0f};

    // 障碍物（淡蓝色方块）
    bool  obstacle_found{false};
    float obstacle_cx{0.0f};
    float obstacle_dist{0.0f};
    cv::Rect obstacle_box;

    // 区域分隔黄线（实/虚判断，用于借道决策）
    bool  divider_found{false};
    float divider_cx{0.0f};
    float divider_dist{0.0f};
    bool  divider_is_dashed{false};
};
