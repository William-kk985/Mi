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
};
