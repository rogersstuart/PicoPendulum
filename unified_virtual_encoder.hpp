// unified_virtual_encoder.hpp
// C++ wrapper for the unified virtual encoder implementation.
// This provides the same interface as the original VirtualEncoder.hpp
// but uses the unified C implementation underneath for consistency.

#ifndef UNIFIED_VIRTUAL_ENCODER_HPP
#define UNIFIED_VIRTUAL_ENCODER_HPP

#include "unified_virtual_encoder.h"
#include <cmath>

class VirtualEncoder {
private:
    ::VirtualEncoder encoder_; // Use the C struct implementation

public:
    // Construct the encoder with an initial unwrapped angle (rad)
    explicit VirtualEncoder(float initialAngle = 0.0f) {
        ve_init(&encoder_, initialAngle);
    }

    // Set the current angular velocity (rad/s).  Call this before update().
    void setVelocity(float omega) { 
        ve_set_velocity(&encoder_, omega); 
    }

    // Integrate the angle forward by dt seconds.  This should be called
    // exactly once per control loop or simulation step.
    void update(float dt) { 
        ve_update(&encoder_, dt); 
    }

    // Get the current unwrapped angle (rad).  This value can grow beyond
    // ±π and preserves the number of full rotations that have occurred.
    float angle() const { 
        return ve_angle(&encoder_); 
    }

    // Get the current angular velocity (rad/s)
    float velocity() const {
        return ve_velocity(&encoder_);
    }

    // Predict where the angle will be after dt seconds, assuming the
    // current angular velocity is constant.  Does not update internal state.
    float predict(float dt) const { 
        return ve_predict(&encoder_, dt); 
    }

    // Wrap a difference into the range [‑π, π].  This is useful when
    // comparing angular distances modulo 2π.
    static float wrapToPi(float x) {
        return ve_wrap_to_pi(x);
    }

    // Returns true if the encoder will cross the target angle within the next dt
    // seconds, given the current angular velocity.  This works in both
    // directions (clockwise/counter‑clockwise) and correctly handles wrap‑around.
    bool willCross(float target, float dt) const {
        return ve_will_cross(&encoder_, target, dt);
    }

    // Returns true if, over the next dt seconds, the angle will move closer to
    // the target angle.  Useful for determining whether we are approaching
    // or moving away from a set‑point.
    bool movingCloser(float target, float dt) const {
        return ve_moving_closer(&encoder_, target, dt);
    }

    // Additional anti-spin functionality
    int getCompletedRotations() const {
        return ve_get_completed_rotations(&encoder_);
    }

    void resetRotationCounter() {
        ve_reset_rotation_counter(const_cast<::VirtualEncoder*>(&encoder_));
    }

    bool shouldApplyAntiSpin() const {
        return ve_should_apply_anti_spin(&encoder_);
    }

    float getAntiSpinCommand() {
        return ve_get_anti_spin_command(const_cast<::VirtualEncoder*>(&encoder_));
    }

    bool isAntiSpinComplete() const {
        return ve_is_anti_spin_complete(&encoder_);
    }

    void resetAntiSpin() {
        ve_reset_anti_spin(const_cast<::VirtualEncoder*>(&encoder_));
    }

    // Provide direct access to C struct for compatibility with existing C-style calls
    ::VirtualEncoder* c_encoder() { return &encoder_; }
    const ::VirtualEncoder* c_encoder() const { return &encoder_; }
};

// For compatibility with existing C-style code that uses ve_init
inline void ve_init(VirtualEncoder* encoder, float initial_angle) {
    ve_init(encoder->c_encoder(), initial_angle);
}

#endif // UNIFIED_VIRTUAL_ENCODER_HPP
