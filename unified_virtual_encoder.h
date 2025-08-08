// unified_virtual_encoder.h
// A unified virtual encoder implementation that works for both C and C++ projects.
// This replaces both the separate C and C++ implementations to maintain consistency
// between the PC simulator and embedded Pico implementation.

#ifndef UNIFIED_VIRTUAL_ENCODER_H
#define UNIFIED_VIRTUAL_ENCODER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <math.h>

// Anti-spin tracking structure for precise rotation detection
typedef struct {
    float start_angle;          // Starting angle for rotation tracking
} AntiSpinTracker;

// Enhanced virtual encoder structure
typedef struct {
    float angle;                // unwrapped angular position (rad)
    float omega;                // current angular velocity (rad/s)
    
    // Anti-spin tracking
    AntiSpinTracker anti_spin;
    int completed_rotations;    // Count of completed full rotations
    
    // Anti-spin braking state
    bool anti_spin_active;      // True when anti-spin braking is engaged
    float anti_spin_start_angle;// Angle when anti-spin braking started
    float anti_spin_target_speed; // Target speed to reach (25% of peak)
    float anti_spin_peak_speed; // Peak speed when anti-spin was triggered
    int anti_spin_direction;    // Direction of rotation that triggered anti-spin
} VirtualEncoder;

// Initialize the virtual encoder with a starting angle (rad)
void ve_init(VirtualEncoder *enc, float initial_angle);

// Set the current angular velocity (rad/s) and update tracking
void ve_set_velocity(VirtualEncoder *enc, float omega);

// Integrate the angle forward by dt seconds and update anti-spin tracking
void ve_update(VirtualEncoder *enc, float dt);

// Return the current unwrapped angle (rad)
float ve_angle(const VirtualEncoder *enc);

// Return the unwrapped tracking angle (for debugging)
float ve_angle_unwrapped(const VirtualEncoder *enc);

// Return the current angular velocity (rad/s)
float ve_velocity(const VirtualEncoder *enc);

// Predict where the angle will be after dt seconds, assuming constant velocity
float ve_predict(const VirtualEncoder *enc, float dt);

// Wrap any angle difference into the range [–π, π]
float ve_wrap_to_pi(float x);

// Determine whether the encoder will cross the target angle within the next dt seconds
bool ve_will_cross(const VirtualEncoder *enc, float target, float dt);

// Determine whether the encoder is moving closer to the target angle
bool ve_moving_closer(const VirtualEncoder *enc, float target, float dt);

// Get the number of completed full rotations since last reset
int ve_get_completed_rotations(const VirtualEncoder *enc);

// Reset the rotation counter (typically called when pendulum changes direction significantly)
void ve_reset_rotation_counter(VirtualEncoder *enc);

// Check if anti-spin braking should be active
bool ve_should_apply_anti_spin(const VirtualEncoder *enc);

// Get the anti-spin brake command (-1 to 1, 0 if not active)
float ve_get_anti_spin_command(VirtualEncoder *enc);

// Check if anti-spin braking is complete (target speed reached or 90° traversed)
bool ve_is_anti_spin_complete(const VirtualEncoder *enc);

// Reset anti-spin state (call when returning to normal control)
void ve_reset_anti_spin(VirtualEncoder *enc);

// Get anti-spin peak speed (for debugging/display)
float ve_get_anti_spin_peak_speed(const VirtualEncoder *enc);

// Get anti-spin target speed (for debugging/display)
float ve_get_anti_spin_target_speed(const VirtualEncoder *enc);

// Control system interface - activate/deactivate anti-spin override
void ve_activate_anti_spin_enc(VirtualEncoder *enc, float current_speed);
void ve_deactivate_anti_spin_enc(VirtualEncoder *enc);

// Set up physics interface for real hardware simulation
void ve_set_physics_interface(float (*get_angle_func)(void));

// =============================================================================
// SIMPLIFIED GLOBAL API (for backward compatibility)
// =============================================================================

// Initialize global encoder
void ve_init_global(float initial_angle);

// Update global encoder
void ve_update_global(float dt);
void ve_set_velocity_global(float omega);

// Get global encoder state
float ve_angle_global(void);
float ve_angle_unwrapped_global(void);
float ve_velocity_global(void);
int ve_get_completed_rotations_global(void);

// Simplified control system interface - no parameters needed
void ve_activate_anti_spin(void);
void ve_deactivate_anti_spin(void);
bool ve_is_anti_spin_active(void);
float ve_get_anti_spin_output(void);

#ifdef __cplusplus
}
#endif

#endif // UNIFIED_VIRTUAL_ENCODER_H
