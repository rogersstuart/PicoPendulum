// VirtualEncoder.hpp
// An abstraction of a high‑speed encoder that integrates angular velocity
// and allows predictive queries (will we cross a target angle before the next update?
// are we moving closer to a target?).  This class stores an unwrapped angle
// that can grow beyond ±π so that high‑speed rotations are tracked accurately.

#ifndef VIRTUAL_ENCODER_HPP
#define VIRTUAL_ENCODER_HPP

#include <cmath>

class VirtualEncoder {
public:
    // Construct the encoder with an initial unwrapped angle (rad)
    explicit VirtualEncoder(float initialAngle = 0.0f)
        : angle_(initialAngle), omega_(0.0f) {}

    // Set the current angular velocity (rad/s).  Call this before update().
    void setVelocity(float omega) { omega_ = omega; }

    // Integrate the angle forward by dt seconds.  This should be called
    // exactly once per control loop or simulation step.
    void update(float dt) { angle_ += omega_ * dt; }

    // Get the current unwrapped angle (rad).  This value can grow beyond
    // ±π and preserves the number of full rotations that have occurred.
    float angle() const { return angle_; }

    // Predict where the angle will be after dt seconds, assuming the
    // current angular velocity is constant.  Does not update internal state.
    float predict(float dt) const { return angle_ + omega_ * dt; }

    // Wrap a difference into the range [‑π, π].  This is useful when
    // comparing angular distances modulo 2π.
    static float wrapToPi(float x) {
        while (x > M_PI)  x -= 2.0f * M_PI;
        while (x < -M_PI) x += 2.0f * M_PI;
        return x;
    }

    // Returns true if the encoder will cross the target angle within the next dt
    // seconds, given the current angular velocity.  This works in both
    // directions (clockwise/counter‑clockwise) and correctly handles wrap‑around.
    bool willCross(float target, float dt) const {
        float a  = wrapToPi(angle_       - target);
        float an = wrapToPi(predict(dt) - target);
        // Crossing occurs if the signs differ or either is zero
        return (a <= 0.0f && an >  0.0f) || (a >= 0.0f && an < 0.0f);
    }

    // Returns true if, over the next dt seconds, the angle will move closer to
    // the target angle.  Useful for determining whether we are approaching
    // or moving away from a set‑point.
    bool movingCloser(float target, float dt) const {
        float curDist = std::fabs(wrapToPi(angle_       - target));
        float nxtDist = std::fabs(wrapToPi(predict(dt) - target));
        return nxtDist < curDist;
    }

private:
    float angle_; // unwrapped angular position (rad)
    float omega_; // current angular velocity (rad/s)
};

#endif // VIRTUAL_ENCODER_HPP