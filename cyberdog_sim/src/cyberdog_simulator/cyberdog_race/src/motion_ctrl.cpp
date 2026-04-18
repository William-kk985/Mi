#include "cyberdog_race/motion_ctrl.hpp"
#include <cstring>

MotionCtrl::MotionCtrl() {
    memset(&gpad_, 0, sizeof(gpad_));
}

void MotionCtrl::set_velocity(float x, float y, float yaw) {
    gpad_.y                   = 1;
    gpad_.leftStickAnalog[1]  = x;
    gpad_.leftStickAnalog[0]  = -y;
    gpad_.rightStickAnalog[0] = -yaw;
    pub_gamepad();
    gpad_.y                   = 0;
    gpad_.leftStickAnalog[1]  = 0;
    gpad_.leftStickAnalog[0]  = 0;
    gpad_.rightStickAnalog[0] = 0;
}

void MotionCtrl::set_pitch(float pitch) {
    gpad_.rightStickAnalog[1] = pitch;
    pub_gamepad();
    gpad_.rightStickAnalog[1] = 0;
}

void MotionCtrl::stand()      { gpad_.x = 1; pub_gamepad(); gpad_.x = 0; }
void MotionCtrl::locomotion() { gpad_.y = 1; pub_gamepad(); gpad_.y = 0; }
void MotionCtrl::lie_down()   { gpad_.a = 1; pub_gamepad(); gpad_.a = 0; }
void MotionCtrl::recovery()   { gpad_.b = 1; pub_gamepad(); gpad_.b = 0; }

void MotionCtrl::stop() {
    gpad_.y                   = 1;
    gpad_.leftStickAnalog[1]  = 0;
    gpad_.leftStickAnalog[0]  = 0;
    gpad_.rightStickAnalog[0] = 0;
    pub_gamepad();
    gpad_.y = 0;
}

void MotionCtrl::pub_gamepad() {
    lcm_.publish("gamepad_lcmt", &gpad_);
}
