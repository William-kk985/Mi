#pragma once
// ═══════════════════════════════════════════════════════════════
// LedIndicator — 赛段 LED 指示（TODO: 需 protocol::srv::LedExecute）
// 拿到真狗 bridges 包后取消注释即可启用，不同赛段不同灯色
// ═══════════════════════════════════════════════════════════════
// #include "protocol/srv/led_execute.hpp"

#include <cstdint>

class LedIndicator {
public:
    enum class Stage : uint8_t {
        IDLE = 0,
        STAGE1, STAGE2, STAGE3, STAGE4, STAGE5, STAGE6,
        DONE, ERROR
    };

    explicit LedIndicator(rclcpp::Node* node) : node_(node) {}
    // TODO: 拿到 protocol::srv::LedExecute 后取消下面构造函数的注释:
    //   , client_(node_->create_client<protocol::srv::LedExecute>("led_execute"))
    // {}

    void set_stage(Stage s) {
        if (s == cur_) return;
        cur_ = s;
        // TODO: 拿到 LedExecute 后取消注释:
        // auto req = std::make_shared<protocol::srv::LedExecute::Request>();
        // req->target = 1;    // HEAD_LED
        // req->mode   = 0x01; // SYSTEM_PREDEFINED
        // req->client = "vp";
        // switch (s) {
        //     case Stage::STAGE1: req->effect = 0xA0; break; // 蓝灯呼吸
        //     case Stage::STAGE2: req->effect = 0xA0; break;
        //     case Stage::STAGE3: req->effect = 0xB0; break; // 黄灯闪烁
        //     case Stage::STAGE4: req->effect = 0xB2; break; // 黄灯呼吸
        //     case Stage::STAGE5: req->effect = 0x90; break; // 红灯闪烁
        //     case Stage::STAGE6: req->effect = 0xAF; break; // 黄灯常亮
        //     case Stage::DONE:   req->effect = 0x06; break; // 逐个点亮(绿)
        //     case Stage::ERROR:  req->effect = 0x90; break; // 红灯呼吸
        //     default: break;
        // }
        // client_->async_send_request(req);
    }

private:
    rclcpp::Node* node_;
    Stage cur_{Stage::IDLE};
    // rclcpp::Client<protocol::srv::LedExecute>::SharedPtr client_;
};
