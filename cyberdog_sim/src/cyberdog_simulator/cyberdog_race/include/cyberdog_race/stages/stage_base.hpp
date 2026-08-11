#pragma once
#include <cmath>
#include <string>
#include <vector>
#include "cyberdog_race/motion_ctrl.hpp"
#include "cyberdog_race/sensor_data.hpp"

class StageBase {
public:
    explicit StageBase(MotionCtrl& motion, SensorData& sensor)
        : motion_(motion), sensor_(sensor) {}
    virtual ~StageBase() = default;

    virtual void init() = 0;
    virtual void run() = 0;
    [[nodiscard]] virtual bool is_done() = 0;

    // ── 参数查询：赛段按需 override，main.cpp 统一下发 ──
    virtual float get_desired_height()      const { return 0.25f; }
    virtual float get_desired_roll()        const { return 0.0f; }
    virtual float get_desired_step_height()  const { return 0.20f; }
    virtual bool  needs_rc_mode()           const { return false; }

    struct ExtraParam { std::string name; double value; };
    virtual std::vector<ExtraParam> get_extra_params() const { return {}; }

protected:
    MotionCtrl&  motion_;
    SensorData&  sensor_;

    // 工具函数：yaw角归一化到[-π, π]
    static float norm_yaw(float y) {
        while (y >  M_PI) y -= 2.0f * M_PI;
        while (y < -M_PI) y += 2.0f * M_PI;
        return y;
    }

    // ═══════════════════════════════════════════════════════
    // 相对目标点导航 (不借助地图, 2026-08-12)
    // 目标点 (dx,dy) 以【导航启动时狗位置为原点、起点朝向为 x 轴】表达
    // 流程: 转向对准目标方向 → 走到目标距离(累计位移+跳变保护)
    // ═══════════════════════════════════════════════════════
    enum class NavPhase { IDLE, TURN_TO, FORWARD_TO, DONE };
    NavPhase nav_phase_{NavPhase::IDLE};
    float    nav_syaw_{0.0f}, nav_target_dist_{0.0f}, nav_target_yaw_{0.0f};
    float    nav_speed_{0.0f}, nav_step_{0.0f};
    float    nav_last_x_{0.0f}, nav_last_y_{0.0f}, nav_traveled_{0.0f};
    int      nav_turn_guard_{0};

    void goto_relative(float dx, float dy, float speed, float step_h) {
        nav_syaw_        = sensor_.abs_yaw;
        nav_target_dist_ = std::hypot(dx, dy);
        nav_target_yaw_  = norm_yaw(nav_syaw_ + std::atan2(dy, dx));
        nav_speed_       = speed;
        nav_step_        = step_h;
        nav_traveled_    = 0.0f;
        nav_turn_guard_  = 0;
        nav_last_x_      = sensor_.odom_x;
        nav_last_y_      = sensor_.odom_y;
        nav_phase_       = NavPhase::TURN_TO;
    }

    bool update_nav() {
        if (nav_phase_ == NavPhase::IDLE) return false;
        if (nav_phase_ == NavPhase::DONE) return true;
        if (nav_phase_ == NavPhase::TURN_TO) {
            float yaw_err = norm_yaw(nav_target_yaw_ - sensor_.abs_yaw);
            if (nav_turn_guard_ > 10 && std::abs(yaw_err) < 0.05f) {
                nav_phase_    = NavPhase::FORWARD_TO;
                nav_traveled_ = 0.0f;
                nav_last_x_   = sensor_.odom_x;
                nav_last_y_   = sensor_.odom_y;
            } else {
                nav_turn_guard_++;
                float turn = std::max(-0.6f, std::min(0.6f, yaw_err * 0.8f));
                motion_.set_walk_velocity_step(0.0f, 0.0f, turn, nav_step_);
            }
            return false;
        }
        float moved = std::hypot(sensor_.odom_x - nav_last_x_, sensor_.odom_y - nav_last_y_);
        nav_last_x_ = sensor_.odom_x;
        nav_last_y_ = sensor_.odom_y;
        if (moved > 0.25f) moved = 0.0f;
        nav_traveled_ += moved;
        if (nav_traveled_ >= nav_target_dist_) {
            motion_.stop();
            nav_phase_ = NavPhase::DONE;
            return true;
        }
        // ★ norm_yaw 必须: 跨±π边界时防止转向饱和 (2026-08-12 修, 同Stage1/2)
        float yaw_cmd = std::max(-0.5f, std::min(0.5f, -norm_yaw(sensor_.abs_yaw - nav_target_yaw_) * 0.8f));
        motion_.set_walk_velocity_step(nav_speed_, 0.0f, yaw_cmd, nav_step_);
        return false;
    }

    [[nodiscard]] bool nav_done() const { return nav_phase_ == NavPhase::DONE; }
};
