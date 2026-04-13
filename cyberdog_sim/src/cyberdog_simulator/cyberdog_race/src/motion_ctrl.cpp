#include "cyberdog_race/motion_ctrl.hpp"
#include <cstring>

MotionCtrl::MotionCtrl() {
    memset(&cmd_, 0, sizeof(cmd_));
}

void MotionCtrl::set_velocity(float x, float y, float yaw) {
    cmd_.leftStickAnalog[1] = x;
    cmd_.leftStickAnalog[0] = y;
    cmd_.rightStickAnalog[0] = yaw;
    publish();
}

void MotionCtrl::set_pitch(float pitch) {
    cmd_.rightStickAnalog[1] = pitch;
    publish();
}

void MotionCtrl::stand() {
    cmd_.x = 1;
    publish();
    cmd_.x = 0;
}

void MotionCtrl::locomotion() {
    cmd_.y = 1;
    publish();
    cmd_.y = 0;
}

void MotionCtrl::lie_down() {
    cmd_.a = 1;
    publish();
    cmd_.a = 0;
}

void MotionCtrl::recovery() {
    cmd_.b = 1;
    publish();
    cmd_.b = 0;
}

void MotionCtrl::stop() {
    cmd_.leftStickAnalog[0] = 0;
    cmd_.leftStickAnalog[1] = 0;
    cmd_.rightStickAnalog[0] = 0;
    cmd_.rightStickAnalog[1] = 0;
    publish();
}

void MotionCtrl::publish() {
    lcm_.publish("gamepad_lcmt", &cmd_);
}
