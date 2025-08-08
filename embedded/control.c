#include "control.h"
#include "config.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "drv8833.h"
#include "debug.h"
#include "adaptive_mass/mass_integration.h"
#include "unified_virtual_encoder.h"
#include "common/energy_control.h"
#include "common/control_utils.h"
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

// ---------------------------------------------------------------------------
// Local VirtualEncoder instance
//
// While a global encoder API is provided by unified_virtual_encoder.c, the
// embedded controller uses a dedicated VirtualEncoder instance for
// predictive calculations (e.g. to anticipate upright crossings).  This
// instance is separate from the global encoder used elsewhere.  It is
// defined here and initialised in ctrl_init().
static VirtualEncoder ve;

// ============================================================================
// Adaptive mass estimator integration
//
// The adaptive mass estimator is encapsulated in the adaptive_mass module.
// It maintains its own global state internally.  We simply call
// adaptive_mass_init() once during controller initialisation and
// adaptive_mass_update() on every control cycle.

/* Use control_utils.h for common helpers.  Do not redefine clampf,
 * sgn or wrap_pi here. */

// Global virtual encoder (kept for compatibility but simplified)
// Now using the unified virtual encoder global API

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

    // Initialize the unified virtual encoder
    ve_init_global(s->theta_u);
    // Initialise the local VirtualEncoder used for predictive calculations
    ve_init(&ve, 0.0f);

    // Initialise the adaptive mass estimator using the configured mass
    adaptive_mass_init(p);
    // Compute the initial desired energy based on the starting mass
    s->Edes = CALCULATE_ENERGY_TARGET(p->m, p->L);
}

void ctrl_reset_integrator(ctrl_state_t *s) { 
    s->ui = 0.f; 
    s->kick_active = false;
    s->drive_level = INITIAL_DRIVE_LEVEL; // Use config value
}

/*
 * Legacy energy control function retained for reference.  The actual
 * energy control logic has been factored into the common module
 * common/energy_control.c and should be called via energy_control()
 * from that module.  This legacy implementation is no longer used
 * but kept to illustrate the original algorithm for comparison.
 */
static float legacy_energy_control(const ctrl_params_t *p, ctrl_state_t *s) {
    /*
     * This legacy implementation is no longer used.  The real energy
     * control logic has been refactored into common/energy_control.c.
     * To keep the linker satisfied we simply return zero.
     */
    (void)p;
    (void)s;
    return 0.0f;
}

float ctrl_update(ctrl_params_t *p, ctrl_state_t *s) {
    // Wrap theta_u to ensure proper control calculations
    s->theta_u = ctl_wrap_pi(s->theta_u);
    
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
            /* Use the common energy control module for swing‑up. */
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
            // Apply additional filtering during balance mode to improve precision.
            // A low‑pass filter smooths the Kalman output further when balancing.
            static float theta_extra_filtered = 0.0f;
            static float omega_extra_filtered = 0.0f;
            static bool extra_filter_initialized = false;
            
            if (!extra_filter_initialized) {
                theta_extra_filtered = s->theta_u_filtered;
                omega_extra_filtered = s->omega_filtered;
                extra_filter_initialized = true;
            }
            
            // Balance‑only additional low‑pass filtering.  A higher alpha provides more smoothing to reduce noise.
            const float extra_filter_alpha = 0.75f;
            theta_extra_filtered = extra_filter_alpha * theta_extra_filtered + (1.0f - extra_filter_alpha) * s->theta_u_filtered;
            omega_extra_filtered = extra_filter_alpha * omega_extra_filtered + (1.0f - extra_filter_alpha) * s->omega_filtered;
            
            // Use the double-filtered values for control calculations
            float theta_u_wrapped = ctl_wrap_pi(theta_extra_filtered);
            
            // Scale the gains based on the angular distance from upright.
            float distance_factor = 1.0f + 2.0f * fabsf(theta_u_wrapped) / (M_PI/6.0f); // Scale up to 3x for large angles
            distance_factor = ctl_clampf(distance_factor, 1.0f, 3.0f);
            
            // Compute aggressive PID gains for balance.  Higher scaling factors help stabilise
            // the upright position across large angles.
            float Kp_aggressive = p->Kp * distance_factor * 5.0f;  // Scaled proportional gain
            float Kd_aggressive = p->Kd * distance_factor * 3.0f;  // Scaled derivative gain  
            float Ki_aggressive = p->Ki * distance_factor * 6.0f;  // Scaled integral gain
            
            // Minimum command threshold to overcome static friction.
            float min_force_threshold = 0.25f;
            
            // Calculate PID terms with negative feedback for stable control using DOUBLE-FILTERED values
            float proportional = -Kp_aggressive * theta_u_wrapped;
            float derivative = -Kd_aggressive * omega_extra_filtered;  // Use double-filtered velocity for maximum stability
            float integral_term = Ki_aggressive * theta_u_wrapped;
            
            // Allow the integral term to build up sufficiently to overcome static friction.
            s->ui += integral_term;
            float max_integral = 5.0f * distance_factor; // Allow massive integral buildup
            s->ui = ctl_clampf(s->ui, -max_integral, max_integral);
            
            // Combine PID terms
            u = proportional + derivative + s->ui;
            
            // Add feed‑forward gravity compensation.
            float gravity_compensation = (p->m * 9.81f * p->L * 0.5f * sinf(theta_u_wrapped)) / p->u_to_tau;
            u += gravity_compensation;
            
            // Deadband threshold to suppress tiny commands that can cause vibration.
            const float vibration_deadband = 0.02f;
            if (fabsf(u) < vibration_deadband) {
                u = 0.0f; // Zero out tiny commands that just cause vibration
            }
            
            // If the command is above the deadband but below the threshold, boost it to overcome static friction.
            if (fabsf(u) > vibration_deadband && fabsf(u) < min_force_threshold) {
                float friction_boost = min_force_threshold * ctl_sgn(u);
                u = friction_boost;
            }
            
            // Apply command smoothing to reduce PWM noise in balance mode.
            static float u_filtered = 0.0f;
            static bool u_filter_initialized = false;
            if (!u_filter_initialized) {
                u_filtered = u;
                u_filter_initialized = true;
            }
            // Use a moderate smoothing factor to balance responsiveness and noise suppression.
            const float cmd_filter_alpha = 0.85f;
            u_filtered = cmd_filter_alpha * u_filtered + (1.0f - cmd_filter_alpha) * u;
            u = u_filtered;
            
            u = ctl_clampf(u, -p->balance_sat, p->balance_sat);
            break;
    }
    
    // Apply saturation
    u = ctl_clampf(u, -1.0f, 1.0f);
    
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

    // Integrate adaptive mass estimation.  This call updates the mass
    // parameter and desired energy target and handles tamper detection.
    adaptive_mass_update((ctrl_params_t*)p, s, u);
    return u;
}
