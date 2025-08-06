// virtual_encoder.c
// Implementation of a virtual encoder for C/C++ projects.  This module
// integrates angular velocity to maintain an unwrapped angle and provides
// helpers to wrap angles and predict future positions.  It mirrors the
// functionality of the C++ VirtualEncoder class used in the PC simulator.

#include "virtual_encoder.h"
#include <math.h>

void ve_init(VirtualEncoder *enc, float initial_angle) {
    enc->angle = initial_angle;
    enc->omega = 0.0f;
}

void ve_set_velocity(VirtualEncoder *enc, float omega) {
    enc->omega = omega;
}

void ve_update(VirtualEncoder *enc, float dt) {
    enc->angle += enc->omega * dt;
}

float ve_angle(const VirtualEncoder *enc) {
    return enc->angle;
}

float ve_predict(const VirtualEncoder *enc, float dt) {
    return enc->angle + enc->omega * dt;
}

float ve_wrap_to_pi(float x) {
    while (x > (float)M_PI)  x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

bool ve_will_cross(const VirtualEncoder *enc, float target, float dt) {
    float a  = ve_wrap_to_pi(enc->angle        - target);
    float an = ve_wrap_to_pi(ve_predict(enc, dt) - target);
    return (a <= 0.0f && an >  0.0f) || (a >= 0.0f && an < 0.0f);
}

bool ve_moving_closer(const VirtualEncoder *enc, float target, float dt) {
    float curDist = fabsf(ve_wrap_to_pi(enc->angle        - target));
    float nxtDist = fabsf(ve_wrap_to_pi(ve_predict(enc, dt) - target));
    return nxtDist < curDist;
}