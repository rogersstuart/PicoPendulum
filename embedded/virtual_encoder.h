// virtual_encoder.h
// A C-compatible virtual encoder for tracking angular position beyond ±π and
// answering predictive queries.  This header defines a simple structure and
// functions for integrating angular velocity and predicting angle crossings.

#ifndef VIRTUAL_ENCODER_H
#define VIRTUAL_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>

typedef struct {
    float angle; // unwrapped angular position (rad)
    float omega; // current angular velocity (rad/s)
} VirtualEncoder;

// Initialize the virtual encoder with a starting angle (rad).  The angle
// represents the upright‑referenced position (0 = upright).
void ve_init(VirtualEncoder *enc, float initial_angle);

// Set the current angular velocity (rad/s).  Call this before ve_update().
void ve_set_velocity(VirtualEncoder *enc, float omega);

// Integrate the angle forward by dt seconds.  This should be called once per
// control loop or simulation step to advance the internal unwrapped angle.
void ve_update(VirtualEncoder *enc, float dt);

// Return the current unwrapped angle (rad).  The value may grow beyond ±π and
// preserves the total number of rotations.
float ve_angle(const VirtualEncoder *enc);

// Predict where the angle will be after dt seconds, assuming constant velocity.
float ve_predict(const VirtualEncoder *enc, float dt);

// Wrap any angle difference into the range [–π, π].  Useful for comparing
// angular distances modulo 2π.
float ve_wrap_to_pi(float x);

// Determine whether the encoder will cross the target angle within the next
// dt seconds.  Returns true if a crossing will occur in either direction.
bool ve_will_cross(const VirtualEncoder *enc, float target, float dt);

// Determine whether the encoder is moving closer to the target angle over the
// next dt seconds.  Returns true if the absolute angular difference to the
// target will decrease.
bool ve_moving_closer(const VirtualEncoder *enc, float target, float dt);

#ifdef __cplusplus
}
#endif

#endif // VIRTUAL_ENCODER_H