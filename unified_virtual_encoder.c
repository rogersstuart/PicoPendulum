// unified_virtual_encoder.c
// Implementation of a unified virtual encoder with advanced anti-spin protection.
// This implementation provides precise rotation tracking and implements the user's
// specification for anti-spin braking that activates after one full rotation
// and reduces speed by 25% within 90 degrees.

#include "unified_virtual_encoder.h"
#include "config.h"
#include <stdio.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Anti-spin constants (moved here since they were removed from config.h)
#define ANTI_SPIN_COAST_TARGET_PERCENT  0.25f   // Coast down to 25% of peak speed
#define ANTI_SPIN_BRAKE_DUTY           0.3f    // Light braking duty during coast-down

// Global encoder instance for simplified API
static VirtualEncoder global_encoder;
static bool global_encoder_initialized = false;

// Global physics interface for real hardware simulation
static float (*get_real_physics_angle)(void) = NULL;
static float last_real_angle = 0.0f;
static bool real_angle_interface_initialized = false;

// Helper function to get sign of a value
static float sign(float x) {
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

// Always synchronise with the real physics before any virtual encoder operation
static void ve_sync_with_physics(VirtualEncoder *enc) {
    if (get_real_physics_angle == NULL) {
        // No physics interface - use velocity integration as fallback
        return;
    }
    
    // In hardware simulation the virtual encoder should always reflect the
    // current real physics angle.  Just like real hardware, the encoder
    // reads the shaft position on every call.
    float current_real_angle = get_real_physics_angle();
    
    if (!real_angle_interface_initialized) {
        // First time - initialize tracking
        last_real_angle = current_real_angle;
        enc->angle = current_real_angle;  // Start VE at real position
        real_angle_interface_initialized = true;
        return;
    }
    
    // For rotation counting, track the unwrapped changes but always
    // use the current physics angle as the true position.  This
    // ensures accurate rotation counts when the pendulum spins
    // multiple times.
    
    // Calculate minimal angular change for unwrapped tracking
    float angle_change = current_real_angle - last_real_angle;
    
    // Handle wraparound correctly - find shortest angular distance  
    while (angle_change > M_PI) angle_change -= 2.0f * M_PI;
    while (angle_change < -M_PI) angle_change += 2.0f * M_PI;
    
    // Update unwrapped angle by accumulating changes (for rotation counting)
    enc->angle += angle_change;
    
    // Update tracking for next delta calculation
    last_real_angle = current_real_angle;
}

void ve_init(VirtualEncoder *enc, float initial_angle) {
    enc->angle = initial_angle;
    enc->omega = 0.0f;
    
    // Initialize anti-spin tracking
    enc->anti_spin.start_angle = initial_angle;
    
    enc->completed_rotations = 0;
    
    // Initialize anti-spin braking state
    enc->anti_spin_active = false;
    enc->anti_spin_start_angle = 0.0f;
    enc->anti_spin_target_speed = 0.0f;
    enc->anti_spin_peak_speed = 0.0f;
    enc->anti_spin_direction = 0;
}

void ve_set_velocity(VirtualEncoder *enc, float omega) {
    // ALWAYS sync with real physics first
    ve_sync_with_physics(enc);
    enc->omega = omega;
}

void ve_update(VirtualEncoder *enc, float dt) {
    // ALWAYS sync with real physics first
    ve_sync_with_physics(enc);
    
    // If we don't have real physics, fall back to velocity integration
    if (get_real_physics_angle == NULL) {
        enc->angle += enc->omega * dt;
    }
    
    // Simple rotation detection: check if we've traveled 2π radians (360°) from start
    // Preserve direction information by NOT using absolute value
    float angular_displacement = enc->angle - enc->anti_spin.start_angle;
    
    // Check if we've completed one full rotation in either direction
    // Positive displacement = CCW rotation, Negative = CW rotation
    if (angular_displacement >= (2.0f * (float)M_PI)) {
        // Completed a CCW (positive) rotation
        enc->completed_rotations++;
        enc->anti_spin.start_angle = enc->angle; // Reset for next rotation
    } else if (angular_displacement <= -(2.0f * (float)M_PI)) {
        // Completed a CW (negative) rotation  
        enc->completed_rotations++;
        enc->anti_spin.start_angle = enc->angle; // Reset for next rotation
    }
    
    // Reset rotation count if we slow down significantly, come to a stop, or change direction significantly
    if (fabsf(enc->omega) < 1.0f) {
        enc->completed_rotations = 0;
        enc->anti_spin.start_angle = enc->angle; // Reset tracking point
    }
}

float ve_angle(const VirtualEncoder *enc) {
    // ALWAYS sync with real physics first (cast away const for sync)
    ve_sync_with_physics((VirtualEncoder*)enc);
    
    // HARDWARE SIMULATION: Return current physics angle, not unwrapped tracking angle
    // The unwrapped angle is used internally for rotation counting only
    if (get_real_physics_angle != NULL) {
        return get_real_physics_angle();  // Always return current physics angle
    } else {
        return enc->angle;  // Fallback to integrated angle if no physics interface
    }
}

float ve_angle_unwrapped(const VirtualEncoder *enc) {
    // ALWAYS sync with real physics first (cast away const for sync)
    ve_sync_with_physics((VirtualEncoder*)enc);
    
    // Return the internal unwrapped tracking angle used for rotation counting
    return enc->angle;
}

float ve_velocity(const VirtualEncoder *enc) {
    // ALWAYS sync with real physics first (cast away const for sync)
    ve_sync_with_physics((VirtualEncoder*)enc);
    
    // Return the velocity set by the control system (which should match physics)
    return enc->omega;
}

float ve_predict(const VirtualEncoder *enc, float dt) {
    // ALWAYS sync with real physics first (cast away const for sync)
    ve_sync_with_physics((VirtualEncoder*)enc);
    return enc->angle + enc->omega * dt;
}

float ve_wrap_to_pi(float x) {
    while (x > (float)M_PI)  x -= 2.0f * (float)M_PI;
    while (x < -(float)M_PI) x += 2.0f * (float)M_PI;
    return x;
}

bool ve_will_cross(const VirtualEncoder *enc, float target, float dt) {
    float a  = ve_wrap_to_pi(enc->angle - target);
    float an = ve_wrap_to_pi(ve_predict(enc, dt) - target);
    return (a <= 0.0f && an >  0.0f) || (a >= 0.0f && an < 0.0f);
}

bool ve_moving_closer(const VirtualEncoder *enc, float target, float dt) {
    float curDist = fabsf(ve_wrap_to_pi(enc->angle - target));
    float nxtDist = fabsf(ve_wrap_to_pi(ve_predict(enc, dt) - target));
    return nxtDist < curDist;
}

int ve_get_completed_rotations(const VirtualEncoder *enc) {
    // ALWAYS sync with real physics first (cast away const for sync)
    ve_sync_with_physics((VirtualEncoder*)enc);
    return enc->completed_rotations;
}

void ve_reset_rotation_counter(VirtualEncoder *enc) {
    ve_sync_with_physics(enc);
    enc->completed_rotations = 0;
    enc->anti_spin.start_angle = enc->angle; // Reset tracking point to current position
}

bool ve_should_apply_anti_spin(const VirtualEncoder *enc) {
    ve_sync_with_physics((VirtualEncoder*)enc);
    return enc->anti_spin_active;
}

float ve_get_anti_spin_command(VirtualEncoder *enc) {
    ve_sync_with_physics(enc);
    if (!enc->anti_spin_active) {
        return 0.0f;
    }
    
    float current_speed = fabsf(enc->omega);
    float angle_traveled = fabsf(enc->angle - enc->anti_spin_start_angle);
    
    // Check if anti-spin should complete
    bool target_speed_reached = (current_speed <= enc->anti_spin_target_speed);
    bool angle_limit_reached = (angle_traveled >= (float)M_PI / 2.0f);
    
    if (target_speed_reached || angle_limit_reached) {
        // Auto-reset anti-spin when complete
        ve_reset_anti_spin(enc);
        return 0.0f;
    }
    
    // Apply brake in opposite direction to rotation
    float brake_direction = -sign(enc->omega);
    return brake_direction * ANTI_SPIN_BRAKE_DUTY;
}

bool ve_is_anti_spin_complete(const VirtualEncoder *enc) {
    ve_sync_with_physics((VirtualEncoder*)enc);
    if (!enc->anti_spin_active) {
        return true; // Not active, so considered complete
    }
    
    float current_speed = fabsf(enc->omega);
    float angle_traveled = fabsf(enc->angle - enc->anti_spin_start_angle);
    
    // Complete if either target speed reached OR 90 degrees traveled
    return (current_speed <= enc->anti_spin_target_speed) || 
           (angle_traveled >= (float)M_PI / 2.0f);
}

void ve_reset_anti_spin(VirtualEncoder *enc) {
    ve_sync_with_physics(enc);
    if (enc->anti_spin_active) {
        printf("ANTI-SPIN DEACTIVATED: speed=%.1f, rotations reset to 0\n", fabsf(enc->omega));
    }
    enc->anti_spin_active = false;
    enc->anti_spin_start_angle = 0.0f;
    enc->anti_spin_target_speed = 0.0f;
    enc->anti_spin_peak_speed = 0.0f;
    enc->anti_spin_direction = 0;
    
    // Reset rotation counter when anti-spin is complete
    enc->completed_rotations = 0;
}

// Control system interface - activate anti-spin override
void ve_activate_anti_spin_enc(VirtualEncoder *enc, float current_speed) {
    ve_sync_with_physics(enc);
    if (!enc->anti_spin_active) {
        enc->anti_spin_active = true;
        enc->anti_spin_start_angle = enc->angle;
        enc->anti_spin_peak_speed = current_speed;
        enc->anti_spin_target_speed = current_speed * ANTI_SPIN_COAST_TARGET_PERCENT;
        enc->anti_spin_direction = (enc->omega > 0) ? 1 : -1;
        printf("ANTI-SPIN ACTIVATED: peak_speed=%.1f, target_speed=%.1f\n", 
               enc->anti_spin_peak_speed, enc->anti_spin_target_speed);
    }
}

// Control system interface - deactivate anti-spin override  
void ve_deactivate_anti_spin_enc(VirtualEncoder *enc) {
    ve_reset_anti_spin(enc);
}

// Set up physics interface for real hardware simulation
void ve_set_physics_interface(float (*get_angle_func)(void)) {
    get_real_physics_angle = get_angle_func;
    real_angle_interface_initialized = false; // Reset to reinitialize
}

// =============================================================================
// SIMPLIFIED GLOBAL API (for backward compatibility)
// =============================================================================

void ve_init_global(float initial_angle) {
    ve_init(&global_encoder, initial_angle);
    global_encoder_initialized = true;
}

void ve_update_global(float dt) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    ve_update(&global_encoder, dt);
}

void ve_set_velocity_global(float omega) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    ve_set_velocity(&global_encoder, omega);
}

float ve_angle_global(void) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    return ve_angle(&global_encoder);
}

float ve_angle_unwrapped_global(void) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    return ve_angle_unwrapped(&global_encoder);
}

float ve_velocity_global(void) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    return ve_velocity(&global_encoder);
}

int ve_get_completed_rotations_global(void) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    return ve_get_completed_rotations(&global_encoder);
}

// Simplified control system interface - no parameters needed
void ve_activate_anti_spin(void) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    float current_speed = fabsf(global_encoder.omega);
    ve_activate_anti_spin_enc(&global_encoder, current_speed);
}

void ve_deactivate_anti_spin(void) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    ve_reset_anti_spin(&global_encoder);
}

bool ve_is_anti_spin_active(void) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    return ve_should_apply_anti_spin(&global_encoder);
}

float ve_get_anti_spin_output(void) {
    if (!global_encoder_initialized) {
        ve_init_global(0.0f);
    }
    return ve_get_anti_spin_command(&global_encoder);
}
