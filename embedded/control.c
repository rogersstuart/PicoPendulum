#include "control.h"
#include "config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "drv8833.h"
#include "debug.h"
#include "virtual_encoder.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"

// Ensure M_PI is defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// External access to motor driver for PWM frequency control and protection status
extern drv8833_t drv;

// External motor inversion flag
extern bool MOTOR_INVERT;

// External sensor inversion flag  
extern bool SENSOR_INVERT;

static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
static inline float sgn(float x) {
    if (x >  0.f) return  1.f;
    if (x <  0.f) return -1.f;
    return 0.f;
}
static inline float wrap_pi(float x) {
    while (x > M_PI)  x -= 2*M_PI;
    while (x <= -M_PI) x += 2*M_PI;
    return x;
}

// Global virtual encoder (kept for compatibility but simplified)
static VirtualEncoder ve;

void ctrl_init(ctrl_params_t *p, ctrl_state_t *s) {
    s->theta_b = 0.f;
    s->theta_u = 0.f;
    s->omega   = 0.f;
    s->u = 0.f;
    s->ui = 0.f;
    s->E = 0.f;
    s->Edes = 0.f; // Will be set later when parameters are configured
    s->state = ST_IDLE;  // Start in IDLE
    
    // Initialize filtered values
    s->theta_u_filtered = 0.f;
    s->omega_filtered = 0.f;
    
    // Initialize simple breakaway/drive variables (matching PC version)
    s->kick_active    = false;
    s->kick_direction = 1;     // arbitrary start direction
    s->drive_level    = INITIAL_DRIVE_LEVEL;  // Use config value

    // Initialize complex variables to default values (for compatibility)
    s->energy_bleed_gain = 1.0f;
    s->last_apex_angle = 0.0f;
    s->last_omega_sign = 0.0f;
    s->adaptive_brake_duty = 0.0f;
    s->apex_detected_this_cycle = false;
    s->continuous_rotations = 0;

    // Initialize the virtual encoder
    ve_init(&ve, 0.0f);
}

void ctrl_reset_integrator(ctrl_state_t *s) { 
    s->ui = 0.f; 
    s->kick_active = false;
    s->drive_level = INITIAL_DRIVE_LEVEL; // Use config value
}

static float energy_control(const ctrl_params_t *p, ctrl_state_t *s) {
    /*
     * Enhanced energy control for swing‑up matching pendulum_simulator.cpp.
     * This implementation includes peak tracking and adaptive drive level adjustment.
     */

    // Convert upright‑referenced angle to bottom‑referenced angle for rest detection
    float theta_b = wrap_pi(s->theta_u + (float)M_PI);

    // Safety: ensure angular velocity is not NaN
    if (isnan(s->omega)) {
        s->omega = 0.0f;
    }

    // Compute total energy relative to the hanging position - using config.h macros
    float cos_theta_u = cosf(s->theta_u);
    float J = (p->m * p->L * p->L) / 3.0f + p->Jm;  // Calculate moment of inertia
    s->E = 0.5f * J * s->omega * s->omega +
           p->m * 9.81f * (p->L * 0.5f) * (1.0f + cos_theta_u);
    float e = s->E - s->Edes;

    // Thresholds for detecting when the pendulum is essentially at rest
    const float STATIONARY_ANGLE  = 0.05f;   // From PC version
    const float STATIONARY_SPEED  = 0.05f;   // From PC version
    const float MOVE_THRESHOLD    = 0.035f;  // From PC version
    // BREAKAWAY_DUTY is defined in config.h

    // Determine if the pendulum is nearly hanging and barely moving
    bool at_rest = (fabsf(theta_b) < STATIONARY_ANGLE) && (fabsf(s->omega) < STATIONARY_SPEED);

    float u_energy = 0.0f;

    if (e < 0.0f) {
        // Below target energy: need to add energy to the system
        if (s->kick_active) {
            // Continue applying breakaway impulse until motion is detected
            u_energy = BREAKAWAY_DUTY * (float)s->kick_direction;
            if (fabsf(theta_b) >= MOVE_THRESHOLD || fabsf(s->omega) >= STATIONARY_SPEED) {
                // End breakaway when the pendulum starts moving
                s->kick_active = false;
                s->drive_level = 0.4f;
            }
        } else if (at_rest) {
            // Pendulum is stuck at the bottom – start a kick
            // Push away from the bottom if off‑centre, otherwise alternate direction
            if (fabsf(theta_b) > STATIONARY_ANGLE) {
                s->kick_direction = -sgn(theta_b);
            } else {
                s->kick_direction = -s->kick_direction;
            }
            s->kick_active = true;
            u_energy = BREAKAWAY_DUTY * (float)s->kick_direction;
        } else {
            // Pendulum is swinging: pump energy in the direction of motion
            float direction = sgn(s->omega);
            u_energy = s->drive_level * direction;

            // Track the maximum excursion in this half swing to adapt drive level - INDUSTRY STANDARD
            static float prev_peak = 0.0f;
            static float current_peak = 0.0f;
            static float last_omega_sign = 0.0f;
            current_peak = fmaxf(current_peak, fabsf(theta_b));
            float omega_sign = sgn(s->omega);
            if (omega_sign != last_omega_sign && last_omega_sign != 0.0f) {
                // End of half swing – compare peaks
                if (current_peak < prev_peak + 0.01f) {
                    // Peak did not increase – gently increase drive level
                    s->drive_level = fminf(s->drive_level + 0.05f, 0.8f);
                }
                prev_peak = current_peak;
                current_peak = 0.0f;
            }
            last_omega_sign = omega_sign;

            // ENHANCED SPINOUT CONTROL: Use VirtualEncoder for precise unwrapped angle tracking
            float unwrapped_angle = ve_angle(&ve);
            static float last_unwrapped_angle = 0.0f;
            static bool unwrapped_initialized = false;
            
            if (!unwrapped_initialized) {
                last_unwrapped_angle = unwrapped_angle;
                unwrapped_initialized = true;
            }
            
            // Detect full rotations using unwrapped angle (more accurate than wrap detection)
            float angle_change = unwrapped_angle - last_unwrapped_angle;
            if (fabsf(angle_change) > 2.0f * M_PI) {
                int new_rotations = (int)(fabsf(angle_change) / (2.0f * M_PI));
                s->continuous_rotations += new_rotations;
                if (debug_is_output_enabled()) {
                    printf("VIRTUAL ENCODER: %d rotations detected (total: %d)\n", new_rotations, s->continuous_rotations);
                }
                last_unwrapped_angle = unwrapped_angle;
            }
            
            // Reset rotation count when we detect a proper apex with low speed
            if (fabsf(s->omega) < 1.0f && current_peak > prev_peak) {
                s->continuous_rotations = 0;
                last_unwrapped_angle = unwrapped_angle; // Reset tracking
            }

            // ANTI-SPIN PROTECTION: After completing full rotations, coast down with light braking
            static float spinout_peak_speed = 0.0f;
            static bool spinout_active = false;
            bool true_spinout = (s->continuous_rotations >= SPINOUT_ROTATION_THRESHOLD);
            
            if (true_spinout) {
                if (!spinout_active) {
                    // First detection - record peak speed and start coasting
                    spinout_peak_speed = fabsf(s->omega);
                    spinout_active = true;
                    if (debug_is_output_enabled()) {
                        printf("ANTI-SPIN ACTIVATED: %d rotations, peak_ω=%.1f rad/s - COASTING DOWN\n",
                               s->continuous_rotations, spinout_peak_speed);
                    }
                }
                
                float current_speed = fabsf(s->omega);
                float target_speed = spinout_peak_speed * ANTI_SPIN_COAST_TARGET_PERCENT;
                
                if (current_speed > target_speed) {
                    // Still above target speed - apply light braking to prevent energy buildup
                    u_energy = -sgn(s->omega) * ANTI_SPIN_BRAKE_DUTY;
                    if (debug_is_output_enabled()) {
                        printf("ANTI-SPIN BRAKE: ω=%.1f/%.1f rad/s, brake=%.1f\n",
                               current_speed, target_speed, ANTI_SPIN_BRAKE_DUTY);
                    }
                } else {
                    // Reached target speed - release anti-spin and reset
                    spinout_active = false;
                    spinout_peak_speed = 0.0f;
                    s->continuous_rotations = 0; // Reset rotation counter
                    u_energy = 0.0f; // No command - let normal control resume
                    if (debug_is_output_enabled()) {
                        printf("ANTI-SPIN RELEASED: ω=%.1f rad/s - RESUMING NORMAL CONTROL\n", current_speed);
                    }
                }
            }
        }
    } else {
        // At or above target energy – coast to allow catch controller to take over
        u_energy = 0.0f;
        s->kick_active = false;
    }

    // Limit command to swing‑up saturation to avoid over‑driving the motor
    if (u_energy > p->swing_sat) {
        u_energy = p->swing_sat;
    } else if (u_energy < -p->swing_sat) {
        u_energy = -p->swing_sat;
    }
    s->u = u_energy;
    return u_energy;
}

float ctrl_update(ctrl_params_t *p, ctrl_state_t *s) {
    // Wrap theta_u to ensure proper control calculations
    s->theta_u = wrap_pi(s->theta_u);
    
    // Simple upright detection (matches PC version exactly)
    bool is_upright = (fabsf(s->theta_u) < p->theta_catch) && (fabsf(s->omega) < p->omega_catch);

    // State transitions (simplified like PC version)
    if (s->state == ST_BALANCE && !is_upright) {
        s->state = ST_SWINGUP; // BALANCE -> SWINGUP
        if (debug_is_output_enabled()) {
            printf("BALANCE->SWINGUP: Lost balance\n");
        }
    }

    float u = 0.0f;
    switch (s->state) {
        case ST_IDLE: // IDLE
            // Don't auto-start swing-up, wait for user command
            break;
            
        case ST_SWINGUP: // SWINGUP
            u = energy_control(p, s);
            
            // ENHANCED TRANSITION: Use VirtualEncoder for predictive upright detection
            if (is_upright) {
                // Additional check: predict if we'll still be upright in the next control cycle
                float predicted_angle = ve_predict(&ve, p->dt);
                float predicted_theta_u = ve_wrap_to_pi(predicted_angle);
                bool will_stay_upright = (fabsf(predicted_theta_u) < p->theta_catch * 1.2f); // 20% margin
                
                if (will_stay_upright) {
                    s->state = ST_BALANCE; 
                    ctrl_reset_integrator(s);
                    if (debug_is_output_enabled()) {
                        printf("SWINGUP->BALANCE: Reached upright (predicted stable)\n");
                    }
                } else if (debug_is_output_enabled()) {
                    printf("SWINGUP: Near upright but trajectory unstable, continuing...\n");
                }
            }
            break;
            
        case ST_BALANCE: // BALANCE
            if (!is_upright) { 
                s->state = ST_SWINGUP; 
                break; 
            }
            // STATE-DEPENDENT ULTRA-AGGRESSIVE filtering ONLY for balance mode
            // Apply additional post-processing filter to Kalman output for balance precision
            static float theta_extra_filtered = 0.0f;
            static float omega_extra_filtered = 0.0f;
            static bool extra_filter_initialized = false;
            
            if (!extra_filter_initialized) {
                theta_extra_filtered = s->theta_u_filtered;
                omega_extra_filtered = s->omega_filtered;
                extra_filter_initialized = true;
            }
            
            // BALANCE-ONLY: Additional low-pass filtering with very aggressive smoothing
            const float extra_filter_alpha = 0.75f; // Heavy additional filtering for balance precision
            theta_extra_filtered = extra_filter_alpha * theta_extra_filtered + (1.0f - extra_filter_alpha) * s->theta_u_filtered;
            omega_extra_filtered = extra_filter_alpha * omega_extra_filtered + (1.0f - extra_filter_alpha) * s->omega_filtered;
            
            // Use the double-filtered values for control calculations
            float theta_u_wrapped = wrap_pi(theta_extra_filtered);
            
            // Distance-based gain scaling (matches PC version)
            float distance_factor = 1.0f + 2.0f * fabsf(theta_u_wrapped) / (M_PI/6.0f); // Scale up to 3x for large angles
            distance_factor = clampf(distance_factor, 1.0f, 3.0f);
            
            // EXTREME AGGRESSIVE PID GAINS to match PC version exactly
            float Kp_aggressive = p->Kp * distance_factor * 5.0f;  // EXTREME proportional gain
            float Kd_aggressive = p->Kd * distance_factor * 3.0f;  // Higher derivative for stability  
            float Ki_aggressive = p->Ki * distance_factor * 6.0f;  // EXTREME integral to break through friction
            
            // BREAKTHROUGH FORCE THRESHOLD: Use much higher minimum force to guarantee movement
            float min_force_threshold = 0.25f;  // Minimum 25% motor authority to breakthrough friction
            
            // Calculate PID terms with negative feedback for stable control using DOUBLE-FILTERED values
            float proportional = -Kp_aggressive * theta_u_wrapped;
            float derivative = -Kd_aggressive * omega_extra_filtered;  // Use double-filtered velocity for maximum stability
            float integral_term = Ki_aggressive * theta_u_wrapped;
            
            // AGGRESSIVE INTEGRAL BUILDUP for overcoming static friction
            s->ui += integral_term;
            float max_integral = 5.0f * distance_factor; // Allow massive integral buildup
            s->ui = clampf(s->ui, -max_integral, max_integral);
            
            // Combine PID terms
            u = proportional + derivative + s->ui;
            
            // Add enhanced feed-forward gravity compensation (matches PC version)
            float gravity_compensation = (p->m * 9.81f * p->L * 0.5f * sinf(theta_u_wrapped)) / p->u_to_tau;
            u += gravity_compensation;
            
            // VIBRATION DEADBAND: Suppress tiny commands that cause vibration
            const float vibration_deadband = 0.02f; // 2% deadband to eliminate micro-movements
            if (fabsf(u) < vibration_deadband) {
                u = 0.0f; // Zero out tiny commands that just cause vibration
            }
            
            // STATIC FRICTION OVERRIDE: If command is above deadband but too small, boost it
            if (fabsf(u) > vibration_deadband && fabsf(u) < min_force_threshold) {
                float friction_boost = min_force_threshold * sgn(u);
                u = friction_boost;
            }
            
            // BALANCE-SPECIFIC command smoothing filter to reduce PWM noise
            static float u_filtered = 0.0f;
            static bool u_filter_initialized = false;
            if (!u_filter_initialized) {
                u_filtered = u;
                u_filter_initialized = true;
            }
            // Lighter command filtering to maintain responsiveness
            const float cmd_filter_alpha = 0.85f; // Moderate command smoothing (was 0.8f)
            u_filtered = cmd_filter_alpha * u_filtered + (1.0f - cmd_filter_alpha) * u;
            u = u_filtered;
            
            u = clampf(u, -p->balance_sat, p->balance_sat);
            break;
    }
    
    // Apply saturation
    u = clampf(u, -1.0f, 1.0f);
    
    // Apply motor inversion if needed
    if (MOTOR_INVERT) {
        u = -u;
    }
    
    s->u = u;
    return u;
}

// Main control step function called from main.c
float ctrl_step(const ctrl_params_t *p, ctrl_state_t *s) {
    // Update the virtual encoder with current sensor data
    ve_set_velocity(&ve, s->omega);
    ve_update(&ve, p->dt);
    
    float u = ctrl_update((ctrl_params_t*)p, s);
    s->u = u;
    return u;
}
