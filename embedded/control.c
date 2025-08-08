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
#ifdef PICO_BUILD
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#endif

// Ensure M_PI is defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// External access to motor driver for PWM frequency control and protection status
extern drv8833_t drv;

#ifdef PICO_BUILD
// External motor inversion flag
extern bool MOTOR_INVERT;

// External sensor inversion flag  
extern bool SENSOR_INVERT;
#else
// PC build stubs
static bool MOTOR_INVERT = false;
static bool SENSOR_INVERT = false;
#endif

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

// Add this near the top of the file to track state transition
static unsigned int balance_mode_start_time = 0;
static bool balance_mode_initialized = false;

float ctrl_update(ctrl_params_t *params, ctrl_state_t *state) {
    float u = 0.0f;
    
    // Handle state transition with hysteresis to prevent oscillating
    if (state->state == ST_SWINGUP) {
        // Check if we're close to upright BEFORE applying control
        // This prevents adaptive braking from interfering with capture
        if (fabsf(state->theta_u) < 0.5f && fabsf(state->omega) < 4.0f) {
            // Transition to balance mode immediately
            state->state = ST_BALANCE;
            balance_mode_start_time = 0;
            balance_mode_initialized = true;
            state->ui = 0.0f;
            
            // Reset adaptive brake to prevent interference
            state->adaptive_brake_duty = 0.0f;
            
            printf("CAPTURED: theta_u=%f, omega=%f, switching to balance mode\n", 
                   state->theta_u, state->omega);
            
            // Apply initial balance control right away
            u = -4.0f * state->theta_u - 0.8f * state->omega;
            u = ctl_clampf(u, -0.3f, 0.3f);
        } else {
            // Only apply energy control when not near upright
            // Disable adaptive braking when approaching upright
            if (fabsf(state->theta_u) < 0.8f) {
                // Near upright - use simple energy control without braking
                float E_error = state->E - state->Edes;
                // Simple proportional control on energy
                u = -0.5f * E_error * ctl_sgn(state->omega * cosf(state->theta_u));
                u = ctl_clampf(u, -0.3f, 0.3f);
                
                // Reset adaptive brake duty to prevent interference
                state->adaptive_brake_duty = 0.0f;
            } else {
                // Far from upright - use full energy control with adaptive features
                u = energy_control(params, state);
                u = ctl_clampf(u, -params->swing_sat, params->swing_sat);
            }
        }
    }
    
    // Handle balance mode with a stabilization period
    else if (state->state == ST_BALANCE) {
        // Count time in balance mode
        balance_mode_start_time++;
        
        // Apply control based on state
        if (balance_mode_start_time < 300) {  // Reduced stabilization period
            // Initial stabilization with moderate gains
            float angle_gain = 4.0f;    // Back to original
            float velocity_gain = 0.8f;  // Back to original
            
            // Apply stabilizing control based on angle and angular velocity
            u = -angle_gain * state->theta_u - velocity_gain * state->omega;
            
            // Limit control output
            u = ctl_clampf(u, -0.5f, 0.5f);
            
            // Add debug output during initial stabilization
            if (balance_mode_start_time % 50 == 0) {
                printf("STABILIZING: t=%d, theta_u=%f, omega=%f, u=%f\n",
                      balance_mode_start_time, state->theta_u, state->omega, u);
            }
            
            // Only fall back to swing-up if we're really far from upright
            if (fabsf(state->theta_u) > 1.5f) {
                printf("BALANCE FAILED: falling back to swing-up\n");
                state->state = ST_SWINGUP;
                balance_mode_initialized = false;
                state->ui = 0.0f;
                // Reset adaptive parameters
                state->adaptive_brake_duty = 0.0f;
                state->apex_detected_this_cycle = false;
            }
            
            state->u = u;
            return u;
        }
        
        // After stabilization period, use normal balance controller
        float theta_u_wrapped = ctl_wrap_pi(state->theta_u);
        
        // Standard gains
        float Kp_balance = 4.0f;
        float Kd_balance = 0.8f;
        float Ki_balance = 0.01f;
        
        // Calculate PID terms with negative feedback for stable control
        float proportional = -Kp_balance * theta_u_wrapped;
        float derivative = -Kd_balance * state->omega;
        
        // Update integrator with anti-windup
        if (fabsf(theta_u_wrapped) < 0.05f) {
            float integral_term = Ki_balance * theta_u_wrapped;
            state->ui += integral_term * params->dt;
            float ui_limit = 0.1f;
            state->ui = ctl_clampf(state->ui, -ui_limit, ui_limit);
        } else {
            state->ui *= 0.98f;
        }
        
        // Combine PID terms
        u = proportional + derivative + state->ui;
        
        // Add gravity compensation
        float gravity_compensation = -0.3f * sinf(theta_u_wrapped);
        u += gravity_compensation;
        
        // Simple first-order filter
        static float u_prev = 0.0f;
        u = 0.7f * u + 0.3f * u_prev;
        u_prev = u;
        
        // Apply saturation
        u = ctl_clampf(u, -params->balance_sat, params->balance_sat);
        
        // If we're losing control, fall back to swing-up
        if (fabsf(theta_u_wrapped) > 1.0f) {
            printf("BALANCE LOST: theta=%f, falling back to swing-up\n", theta_u_wrapped);
            state->state = ST_SWINGUP;
            balance_mode_initialized = false;
            state->ui = 0.0f;
            u_prev = 0.0f;
            // Reset adaptive parameters
            state->adaptive_brake_duty = 0.0f;
            state->apex_detected_this_cycle = false;
        }
        
        // Debug output
        if (balance_mode_start_time % 100 == 1) {
            printf("BALANCE: t=%d, theta=%f, omega=%f, u=%f, ui=%f\n", 
                   balance_mode_start_time, theta_u_wrapped, state->omega, u, state->ui);
        }
    }
    
    // Apply saturation
    u = ctl_clampf(u, -1.0f, 1.0f);
    
    // Apply motor inversion if needed
    if (MOTOR_INVERT) {
        u = -u;
    }
    
    state->u = u;
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
