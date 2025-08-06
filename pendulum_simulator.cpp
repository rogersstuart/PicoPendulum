// Fixed main pendulum simulator file
// This fixes the original pendulum_simulator.cpp without simplifying it

// [Include all the original includes and preprocessor directives from your code]

#ifdef PC_DEBUG
    #define PLATFORM_PC
    #include <SDL2/SDL.h>
    #include <SDL2/SDL_ttf.h>
    #include <imgui.h>
    #include <imgui_impl_sdl2.h>
    #include <imgui_impl_opengl3.h>
    #include <GL/gl3w.h>
    #include <iostream>
    #include <cmath>
    #include <cstdlib>
    #include <vector>
    #include <chrono>
    #include <thread>
    #include <fstream>
    #include <stdio.h>
    #include <string.h>
    #include <stdarg.h>
    
    // Include control system definitions for PC build
    #include "embedded/control.h"
    #include "embedded/drv8833.h"  // Motor protection system
#include "embedded/motor_protection.h"  // Direct include for testing

// Include the virtual encoder abstraction for high‑speed angle tracking
    // Use the C virtual encoder for both PC and Pico builds.  This header
    // provides a C-compatible API that we can call from C++ as well.
    extern "C" {
        #include "virtual_encoder.h"
    }
#else
    #define PLATFORM_PICO
    #include <stdio.h>
    #include <math.h>
    #include <string.h>
    #include <stdint.h>
    #include "pico/stdlib.h"
    #include "pico/time.h"
    #include "hardware/i2c.h"
    #include "hardware/gpio.h"
    #include "hardware/pwm.h"
    #include "hardware/watchdog.h"
    
    // Include the embedded control system for Pico
    #include "control.h"
    #include "as5600.h"
    #include "drv8833.h"
    #include "filters.h"
    #include "debug.h"
#endif

// [Include all your original control system code - it's correct]

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define CONTROL_HZ 1000.0f

// [All your original control structures and functions - they're working correctly]
// Just keeping the essential parts here for brevity

#ifdef PLATFORM_PC

// Include the enhanced physics simulation header properly
#include "enhanced_physics_simulation.hpp"
#include "embedded/config.h"  // Include centralized configuration

// Use the enhanced physics model for industrial-grade accuracy
typedef EnhancedPendulumPhysics PendulumPhysics;

// FIX: Add control system implementation for PC build
// These are the same control algorithms used on the Pico

// Define state constants to match the enum in control.h
#define ST_IDLE 0
#define ST_CALIB 1
#define ST_SWINGUP 2
#define ST_CATCH 3
#define ST_BALANCE 4
#define ST_FAULT 5

namespace {
    static inline float clampf(float x, float lo, float hi) { 
        return x < lo ? lo : (x > hi ? hi : x); 
    }
    
    static inline float sgn(float x) { 
        return x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f); 
    }
    
    static inline float wrap_pi(float x) {
        while (x > M_PI) x -= 2*M_PI;
        while (x <= -M_PI) x += 2*M_PI;
        return x;
    }
    
    static float energy_control(const ctrl_params_t *p, ctrl_state_t *s) {
        /*
         * Simplified energy control for swing‑up.
         *
         * This implementation mirrors the embedded controller's approach.  It computes the
         * total energy relative to the hanging position using the upright angle s->theta_u,
         * then pumps energy into the system when it is below the desired energy target.  A
         * brief “kick” is used when the pendulum is stuck at the bottom.  Continuous
         * pumping gradually increases the drive level if swing peaks do not grow.  When
         * sufficient energy has been accumulated the controller coasts until the catch
         * logic in ctrl_update() hands over to the balance controller.
         */

        // Convert upright‑referenced angle to bottom‑referenced angle for rest detection
        float theta_b = wrap_pi(s->theta_u + (float)M_PI);

        // Safety: ensure angular velocity is not NaN
        if (isnan(s->omega)) {
            s->omega = 0.0f;
        }

        // Compute total energy relative to the hanging position.  Using theta_u here is
        // equivalent to the bottom‑based expression 1‑cos(theta_b) because
        // cos(theta_b) = −cos(theta_u).
        float cos_theta_u = cosf(s->theta_u);
        float J = CALCULATE_MOMENT_OF_INERTIA(p->m, p->L, p->Jm); // From config.h
        s->E = 0.5f * J * s->omega * s->omega +
               p->m * GRAVITY_ACCEL * (p->L * 0.5f) * (1.0f + cos_theta_u);
        float e = s->E - s->Edes;

        // Thresholds for detecting when the pendulum is essentially at rest
        const float STATIONARY_ANGLE  = STATIONARY_ANGLE_THRESHOLD;  // From config.h
        const float STATIONARY_SPEED  = STATIONARY_SPEED_THRESHOLD;  // From config.h
        const float MOVE_THRESHOLD    = MOVEMENT_THRESHOLD;           // From config.h
        // BREAKAWAY_DUTY is now defined in config.h

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

                // DEBUG: Add periodic debug output for energy pumping
                static int pump_debug_counter = 0;
                if (++pump_debug_counter % 50 == 0) {
                    printf("ENERGY PUMP DEBUG: E=%.4f/%.4f (%.1f%%), θ_u=%.1f°, ω=%.1f, drive=%.3f, cmd=%.3f\n",
                           s->E, s->Edes, (s->E/s->Edes)*100, s->theta_u * 180/M_PI, s->omega, s->drive_level, u_energy);
                }

                // Track the maximum excursion in this half swing to adapt drive level
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
            }
        } else {
            // At or above target energy – coast to allow catch controller to take over
            u_energy = 0.0f;
            s->kick_active = false;
            
            // DEBUG: Add periodic debug output to understand why energy pumping stops
            static int energy_debug_counter = 0;
            if (++energy_debug_counter % 100 == 0) {
                printf("ENERGY CONTROL DEBUG: Coasting because E=%.4f >= Edes=%.4f (ratio=%.1f%%), θ_u=%.1f°, max_angle=%.1f°\n",
                       s->E, s->Edes, (s->E/s->Edes)*100, s->theta_u * 180/M_PI, fabsf(s->theta_u) * 180/M_PI);
            }
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
}

// Control system functions - identical to Pico implementation
void ctrl_init(ctrl_params_t *p, ctrl_state_t *s) {
    (void)p; // Suppress unused parameter warning
    s->theta_b = 0.f; s->theta_u = 0.f; s->omega = 0.f; s->u = 0.f;
    s->ui = 0.f; s->E = 0.f; s->Edes = 0.f; s->state = (decltype(s->state))ST_IDLE;
    s->kick_active = false; s->kick_direction = 1; s->drive_level = 0.4f;
    
    // Initialize new energy prediction and overshoot estimation variables
    s->energy_bleed_gain = 1.0f;        // Start with default gain
    s->last_apex_angle = 0.0f;          // No previous apex
    s->last_omega_sign = 0.0f;          // No previous velocity sign
    s->adaptive_brake_duty = 0.0f;      // No braking initially
    s->apex_detected_this_cycle = false; // No apex detected yet
    s->continuous_rotations = 0;        // No rotations counted yet
}

void ctrl_reset_integrator(ctrl_state_t *s) { 
    s->ui = 0.f; s->kick_active = false; s->drive_level = 0.4f;
    // Keep energy prediction state during integrator reset
}

// Use the header declaration for ctrl_update - implement our version
float ctrl_update(ctrl_params_t *p, ctrl_state_t *s) {
    s->theta_u = wrap_pi(s->theta_u);
    
    // CRITICAL FIX: Only catch when ACTUALLY near upright (not hanging down)
    // theta_u = 0 is upright, theta_u = ±π is hanging down
    // CRITICAL FIX: Correct near_upright detection to prevent catching at bottom
    float theta_u_wrapped = wrap_pi(s->theta_u);  // Ensure angle is properly wrapped
    // Catch zone: within 30° of upright
    bool near_catch = (fabsf(theta_u_wrapped) < 30.0f * M_PI / 180.0f);
    bool well_settled = (fabsf(s->theta_u) < 5.0f * M_PI / 180.0f) && (fabsf(s->omega) < 0.5f); // 5° and 0.5 rad/s
    
    // CRITICAL FIX: Add sustained settling requirement to prevent false triggers
    static int settle_counter = 0;
    static int catch_counter = 0;
    
    if (s->state == (decltype(s->state))ST_CATCH) {
        if (well_settled && near_catch) {
            settle_counter++;
        } else {
            settle_counter = 0; // Reset if not both settled and near upright
        }
        
        if (near_catch) {
            catch_counter++;
        } else {
            catch_counter = 0; // Reset if pendulum leaves catch zone
        }
    } else {
        settle_counter = 0;
        catch_counter = 0;
        catch_counter = 0;
    }
    
    // Debug output with angle position indicator
    static int debug_counter = 0;
    if (++debug_counter % 200 == 0) { // More frequent debug
        float angle_deg = s->theta_u * 180/M_PI;
        
        // CORRECTED position detection: Check distance from upright (0°) vs hanging (±180°)
        float dist_from_upright = fabsf(angle_deg);
        float dist_from_hanging = fminf(fabsf(angle_deg - 180.0f), fabsf(angle_deg + 180.0f));
        
        const char* position;
        if (dist_from_upright < 30.0f) {
            position = "UPRIGHT";
        } else if (dist_from_hanging < 30.0f) {
            position = "HANGING";
        } else {
            position = "SIDE";
        }
        
    printf("DEBUG: theta=%.1f° (%s), omega=%.1f rad/s, near_catch=%d, settled=%d, state=%s\n", 
               angle_deg, position, s->omega, near_catch, well_settled,
               (s->state == ST_IDLE) ? "IDLE" :
               (s->state == ST_CALIB) ? "CALIB" :
               (s->state == ST_SWINGUP) ? "SWINGUP" :
               (s->state == ST_CATCH) ? "CATCH" :
               (s->state == ST_BALANCE) ? "BALANCE" :
               (s->state == ST_FAULT) ? "FAULT" : "UNKNOWN");
    }

    float u = 0.0f;
    switch (s->state) {
        case ST_IDLE: // IDLE
            // Don't automatically start swing-up - let user control it
            break;
            
        case ST_SWINGUP: { // SWINGUP - energy-based swing-up control with proactive braking
            
            // CRITICAL FIX: Direct motor command scaling based on LEVER ARM (m × L)
            // Lever equation: Required torque scales with mass × length
            float default_lever_arm = 0.004f * 0.3048f;  // Default 4g at 12 inches
            float current_lever_arm = p->m * p->L;        // Current pendulum lever arm
            float lever_ratio = current_lever_arm / default_lever_arm;
            float motor_scale = sqrtf(lever_ratio); // Square root for stability
            motor_scale = fmaxf(motor_scale, 0.3f); // Increased minimum power for adequate swing (30%)
            
            // DEBUG: Print lever arm scaling for verification
            static int debug_counter = 0;
            if (++debug_counter % 100 == 0 && (p->m < 0.003f || p->L < 0.2f)) { // Debug for small pendulums
                printf("LEVER ARM SCALING: m=%.3fg, L=%.3fm, lever_arm=%.6f kg⋅m, scale=%.3f\n",
                       p->m * 1000.0f, p->L, current_lever_arm, motor_scale);
            }
            
            /*
             * Progressive rotation limiter and emergency braking
             *
             * We track the total absolute angular motion of the pendulum.  When the
             * accumulated rotation exceeds 2π radians (one full revolution), we
             * engage a progressive braking mode.  In this mode we compute the
             * minimum motor duty required to decelerate the pendulum and prevent
             * another full rotation.  Braking is scaled according to the
             * pendulum’s moment of inertia and the available motor torque.  Once
             * the speed drops below a safe threshold, the limiter disengages and
             * normal energy control resumes.
             */
            static float rotation_accum = 0.0f;
            // NOTE: Removed old progressive braking static variables - no longer needed
            // New anti-spin logic uses its own static variables within the true_spinout block

            // NOTE: Removed EMERGENCY_BRAKE_THRESHOLD - no longer using velocity-based thresholds

            // Update accumulated rotation using an unwrapped angle estimate.  We
            // integrate the angular velocity to form a continuous angle so that
            // multiple rotations can be detected.  Using the wrapped theta_u
            // would reset at ±π and undercount rotations.
            static float enc_unwrapped = 0.0f;
            static float last_unwrapped = 0.0f;
            enc_unwrapped += s->omega * p->dt;
            float dtheta = enc_unwrapped - last_unwrapped;
            rotation_accum += fabsf(dtheta);
            last_unwrapped = enc_unwrapped;

            // NOTE: Removed velocity-based emergency braking - anti-spin now rotation-based only

            // Detect full rotation and engage braking.  When engaged, initialise the
            // progressive brake duty and record the current angular speed.  The
            // progressive brake will then adjust its duty in a closed loop to bring
            // the pendulum smoothly to rest at the top.
            // Disable progressive brake engagement during swing‑up.  The swing‑up
            // controller now relies solely on the physics‑based predictive braking
            // NOTE: Removed old progressive braking code - replaced with rotation-based anti-spin
            // that coasts down to target speed with light braking as needed

            // Progressive braking mode: closed‑loop duty control.  Instead of
            // computing a single duty based on ideal deceleration, we measure the
            // NOTE: Removed old progressive braking code - replaced with rotation-based anti-spin
            
            // Normal energy control - no longer gated by velocity thresholds
            {
                // Declare variables at the beginning of the block
                float current_theta = s->theta_u;  // Current angle from vertical (rad) - INVERTED!
                float current_omega = s->omega;    // Current angular velocity (rad/s)
                float dt = p->dt;                  // Time step (seconds)
                
                // Normal swing-up speeds – continue with energy control.  Before performing
                // predictive braking calculations, check if the pendulum already has
                // excess energy relative to the desired target.  If the current
                // mechanical energy exceeds the desired energy by more than a small
                // margin, apply a proportional braking duty to bleed off the excess
                // energy over the course of the upswing.  This adaptive braking
                // prevents excessive pumping and reduces the need for abrupt
                // braking near the top.
                // float energy_ratio = (s->Edes > 1e-6f) ? (s->E / s->Edes) : 0.0f; // REMOVED: Redeclaration error
                // If the energy ratio is above 1.05 (5 % over target), compute a
                // proportional brake command based on the excess ratio.  The gain
                // determines how aggressively energy is bled off; adjust as needed.
                // const float k_energy_brake = 0.6f; // REMOVED: Old energy braking system
                // if (energy_ratio > 1.05f) {
                //     float brake_cmd = k_energy_brake * (energy_ratio - 1.0f);
                //     if (brake_cmd > 1.0f) brake_cmd = 1.0f;
                //     // Apply brake opposite to the current direction of motion
                //     u = -sgn(current_omega) * brake_cmd;
                //     // Skip predictive braking for this step; continue to next iteration
                //     // after logging debug information below.
                // }
                
                // ADVANCED ENERGY PREDICTION AND OVERSHOOT ESTIMATION SYSTEM
                // ================================================================
                
                // 1. APEX DETECTION AND CALIBRATION ON THE FLY
                // Detect zero-crossings of angular velocity to identify swing apexes
                float omega_sign = sgn(current_omega);
                s->apex_detected_this_cycle = false;
                
                if (s->last_omega_sign != 0.0f && omega_sign != s->last_omega_sign && fabsf(current_omega) < 2.0f) {
                    // Zero-crossing detected - we just reached an apex
                    s->apex_detected_this_cycle = true;
                    float current_apex_angle = fabsf(current_theta) * 180.0f / M_PI; // Convert to degrees
                    
                    // Calibration: Compare actual apex with expected target (upright = 0°)
                    if (s->last_apex_angle > 0.0f) { // Have previous apex for comparison
                        float target_apex = 0.0f; // Target is upright
                        float apex_error = current_apex_angle - target_apex;
                        
                        // Adjust energy bleed gain based on overshoot/undershoot
                        if (apex_error > 5.0f) { // Overshooting by more than 5°
                            s->energy_bleed_gain *= 1.05f; // Increase bleeding slightly
                        } else if (apex_error < -5.0f) { // Undershooting by more than 5°
                            s->energy_bleed_gain *= 0.95f; // Decrease bleeding slightly
                        }
                        
                        // Clamp gain to reasonable bounds
                        s->energy_bleed_gain = fmaxf(0.5f, fminf(s->energy_bleed_gain, 2.0f));
                        
                        printf("APEX CALIBRATION: angle=%.1f°, error=%.1f°, gain=%.3f\n", 
                               current_apex_angle, apex_error, s->energy_bleed_gain);
                    }
                    
                    s->last_apex_angle = current_apex_angle;
                }
                s->last_omega_sign = omega_sign;
                
                // 2. CONTINUOUS ROTATION DETECTION FOR SPIN-OUT PROTECTION
                // Track continuous rotations more intelligently
                static float last_wrapped_angle = 0.0f;
                float current_wrapped = wrap_pi(current_theta);
                float angle_delta = current_wrapped - last_wrapped_angle;
                
                // Detect when pendulum crosses ±π boundary (full rotation)
                if (fabsf(angle_delta) > M_PI) {
                    s->continuous_rotations++;
                    printf("CONTINUOUS ROTATION #%d detected\n", s->continuous_rotations);
                } else if (s->apex_detected_this_cycle && fabsf(current_omega) < 1.0f) {
                    // Reset rotation count when we detect a proper apex with low speed
                    s->continuous_rotations = 0;
                }
                last_wrapped_angle = current_wrapped;
                
                // 3. ENERGY RATIO AND OVERSHOOT PREDICTION
                // float energy_ratio = (s->Edes > 1e-6f) ? (s->E / s->Edes) : 0.0f; // REMOVED: Redeclaration error
                
                // Predict if next swing will overshoot based on current energy and physics
                // float moment_of_inertia = CALCULATE_MOMENT_OF_INERTIA(p->m, p->L, p->Jm); // REMOVED: Unused variable
                // float gravity_accel = GRAVITY_ACCEL * (p->L * 0.5f) / moment_of_inertia; // REMOVED: Unused variable
                
                // Calculate predicted apex angle for next swing using energy conservation
                float predicted_next_apex = 0.0f;
                if (s->E > 0 && s->Edes > 0) {
                    // Energy conservation: E = mgh + (1/2)Jω²
                    // At apex: ω = 0, so E = mgh = mg(L/2)(1 + cos(θ_apex))
                    // Solving: cos(θ_apex) = (E/(mg*L/2)) - 1
                    float energy_height_factor = s->E / (p->m * GRAVITY_ACCEL * p->L * 0.5f);
                    if (energy_height_factor <= 2.0f) { // Physically possible
                        float cos_apex = energy_height_factor - 1.0f;
                        cos_apex = fmaxf(-1.0f, fminf(1.0f, cos_apex)); // Clamp to valid range
                        predicted_next_apex = acosf(-cos_apex); // Convert to upright-referenced
                        predicted_next_apex = fabsf(predicted_next_apex) * 180.0f / M_PI;
                    }
                }
                
                // 4. ADAPTIVE BRAKING DUTY CALCULATION
                s->adaptive_brake_duty = 0.0f; // Reset
                
                float current_energy_ratio = (s->Edes > 1e-6f) ? (s->E / s->Edes) : 0.0f;
                if (current_energy_ratio > 1.02f) { // More than 2% over target
                    // Calculate exact energy to bleed off using motor torque-to-duty conversion
                    float excess_energy = s->E - s->Edes;
                    float excess_ratio = excess_energy / s->Edes;
                    
                    // Calculate braking torque needed to bleed off excess energy over one swing period
                    // Assume swing period ≈ 2π√(L/g) for small angles, scale for actual pendulum
                    float estimated_swing_period = 2.0f * M_PI * sqrtf(p->L / GRAVITY_ACCEL);
                    float energy_bleed_rate = excess_energy / estimated_swing_period; // Energy/time
                    
                    // Convert energy bleed rate to braking torque
                    // Power = Torque × Angular_velocity, so Torque = Power / Angular_velocity
                    float abs_omega = fabsf(current_omega);
                    if (abs_omega > 0.1f) {
                        float required_brake_torque = energy_bleed_rate / abs_omega;
                        
                        // Convert torque to motor duty using motor torque-to-duty conversion
                        s->adaptive_brake_duty = required_brake_torque / p->u_to_tau;
                        
                        // Apply calibrated energy bleed gain for fine-tuning
                        s->adaptive_brake_duty *= s->energy_bleed_gain;
                        
                        // Scale based on excess ratio - more excess = more braking
                        s->adaptive_brake_duty *= (1.0f + excess_ratio);
                        
                        // Clamp to reasonable braking limits (max 30% brake during energy pumping)
                        s->adaptive_brake_duty = fmaxf(0.0f, fminf(s->adaptive_brake_duty, 0.3f));
                        
                        printf("ENERGY OVERSHOOT PREDICTION: ratio=%.2f, excess=%.3fJ, pred_apex=%.1f°, brake_duty=%.3f\n",
                               current_energy_ratio, excess_energy, predicted_next_apex, s->adaptive_brake_duty);
                    }
                }
                
                // 5. ROTATION-BASED ANTI-SPIN PROTECTION
                // After completing full rotations, cut power and coast down with light braking
                bool true_spinout = (s->continuous_rotations >= SPINOUT_ROTATION_THRESHOLD);
                
                if (true_spinout) {
                    // Record peak speed when first detecting spin-out
                    static float spinout_peak_speed = 0.0f;
                    static bool spinout_active = false;
                    
                    if (!spinout_active) {
                        // First detection - record peak speed and start coasting
                        spinout_peak_speed = fabsf(current_omega);
                        spinout_active = true;
                        printf("ANTI-SPIN ACTIVATED: %d rotations, peak_ω=%.1f rad/s - COASTING DOWN\n",
                               s->continuous_rotations, spinout_peak_speed);
                    }
                    
                    float current_speed = fabsf(current_omega);
                    float target_speed = spinout_peak_speed * ANTI_SPIN_COAST_TARGET_PERCENT;
                    
                    if (current_speed > target_speed) {
                        // Still above target speed - apply light braking to prevent energy buildup
                        u = -sgn(current_omega) * ANTI_SPIN_BRAKE_DUTY;
                        printf("ANTI-SPIN BRAKE: ω=%.1f/%.1f rad/s (%.0f%% of peak), brake=%.1f\n",
                               current_speed, target_speed, (current_speed/spinout_peak_speed)*100, ANTI_SPIN_BRAKE_DUTY);
                    } else {
                        // Reached target speed - release anti-spin and reset
                        spinout_active = false;
                        spinout_peak_speed = 0.0f;
                        s->continuous_rotations = 0; // Reset rotation counter
                        u = 0.0f; // No command - let normal control resume
                        printf("ANTI-SPIN RELEASED: ω=%.1f rad/s (reached %.0f%% target) - RESUMING NORMAL CONTROL\n",
                               current_speed, ANTI_SPIN_COAST_TARGET_PERCENT * 100);
                    }
                } else {
                    // Normal swing-up with adaptive braking during upswing
                
                // PHYSICS-BASED INTELLIGENT PREDICTIVE BRAKING SYSTEM
                // Strategy: Use physics and speed-based optimal braking timing for effective capture
                // Calculate optimal advance timing based on pendulum dynamics and motor authority
                
                // ANGLE CORRECTION: Convert theta_u to normal convention (0=upright, ±π=hanging)
                float normal_theta = wrap_pi(current_theta + M_PI);  // Convert to normal convention
                
                // PHYSICS-BASED APEX PREDICTION: Where will pendulum be at zero velocity?
                float gravity_accel = GRAVITY_ACCEL * (p->L * 0.5f) / (CALCULATE_MOMENT_OF_INERTIA(p->m, p->L, p->Jm)); // rad/s²
                float predicted_apex_normal = normal_theta;
                if (fabsf(current_omega) > 0.1f) {
                    float decel_from_gravity = gravity_accel * sinf(normal_theta);
                    predicted_apex_normal = normal_theta + (current_omega * current_omega) / (2.0f * decel_from_gravity);
                }
                
                // INTELLIGENT BRAKING CALCULATION
                // Calculate optimal braking parameters based on physics and pendulum inertia.  Instead of
                // using fixed advance and brake times, we derive them from the moment of inertia and
                // available motor torque.  This adapts the brake timing for any pendulum mass and length.
                // Total moment of inertia includes the pendulum body and motor rotor inertia.
                float moment_of_inertia = p->m * p->L * p->L + p->Jm;
                float angular_momentum = moment_of_inertia * current_omega;
                float abs_omega = fabsf(current_omega);

                // Compute the maximum angular deceleration the motor can produce at full duty.  The
                // maximum torque is p->u_to_tau (N·m) when the command is ±1.  Dividing by the total
                // inertia yields the maximum angular acceleration (rad/s²).
                float alpha_max = p->u_to_tau / moment_of_inertia;
                // Use a fraction of the maximum to avoid harsh over‑braking.  A value of 0.6 gives
                // reasonably gentle braking while still bringing the pendulum to rest in time.
                float alpha_desired = 0.6f * alpha_max;
                // Compute the ideal brake time needed to reduce the current angular velocity to zero
                // using the desired deceleration.  Constrain the value to a reasonable range to avoid
                // extremely short or excessively long brakes.
                float optimal_brake_time = 0.0f;
                if (alpha_desired > 0.001f) {
                    optimal_brake_time = abs_omega / alpha_desired;
                }
                // Clamp brake time: minimum 0.10s, maximum 1.20s.  Heavier pendulums will get longer
                // braking times, while very small velocities will not produce unrealistically long times.
                optimal_brake_time = fminf(fmaxf(optimal_brake_time, 0.10f), 1.20f);
                // Choose an advance time as a fraction of the brake time.  Starting the brake about
                // 40 % of the brake time before the apex generally yields good results.
                float optimal_advance_time = 0.40f * optimal_brake_time;
                // Ensure a minimum advance to give the controller time to actuate.
                optimal_advance_time = fmaxf(optimal_advance_time, 0.04f);

                // Calculate required torque for optimal braking.  The pendulum has angular
                // momentum L = J · ω, and if we apply a constant torque τ for t seconds the change in
                // angular momentum is τ · t.  Therefore τ = L / t (direction opposite to motion).
                float optimal_brake_torque = (optimal_brake_time > 1e-4f) ? fabsf(angular_momentum) / optimal_brake_time : 0.0f;
                float optimal_brake_cmd = optimal_brake_torque / p->u_to_tau;

                // Clamp brake command to realistic motor limits (±1.0)
                optimal_brake_cmd = fminf(optimal_brake_cmd, 1.0f);
                // Convert back to theta_u convention for timing calculation.  The predicted apex
                // (normal convention) is converted to theta_u by subtracting π and wrapping.
                float predicted_apex_theta_u = wrap_pi(predicted_apex_normal - M_PI);

                // INTELLIGENT BRAKE TIMING
                // Determine the angular position at which braking should commence.  We compute the
                // angle we expect to be at optimal_advance_time seconds before the apex by backing
                // off along the direction of motion: θ_brake_start = θ_apex − sgn(ω)·ω·t_adv.
                float brake_start_angle = predicted_apex_theta_u - sgn(current_omega) * (optimal_advance_time * abs_omega);

                // TIME TO BRAKE CALCULATION
                // Estimate how long until we reach the brake start angle under constant current
                // velocity.  This is a linear approximation; we only use it to detect when to
                // trigger the brake in a discrete simulation.
                float time_to_brake_start = -1.0f;
                if (fabsf(current_omega) > 0.1f) {
                    time_to_brake_start = (brake_start_angle - current_theta) / current_omega;
                }

                // DEBUG: Print braking calculation info periodically
                static int brake_debug_counter = 0;
                if (++brake_debug_counter % 50 == 0) {  // Every ~0.5 seconds
                    printf("BRAKE DEBUG: θ_u=%.1f°, ω=%.1f, E=%.3f/%.3f (%.0f%%), apex_pred=%.1f°, brake_start=%.1f°, t_brake=%.1fms, t_adv=%.1fms\n",
                           current_theta * 180/M_PI, current_omega, s->E, s->Edes, (s->E/s->Edes)*100,
                           predicted_apex_theta_u * 180/M_PI, brake_start_angle * 180/M_PI,
                           optimal_brake_time * 1000, optimal_advance_time * 1000);
                }

                // BRAKE DECISION: Start physics-based intelligent braking
                bool should_brake = false;
                // Sufficient energy means the system has at least 80 % of the desired energy.  A
                // higher threshold ensures the pendulum has accumulated nearly all of the
                // required energy before braking, preventing it from stalling around the
                // horizontal.
                bool sufficient_energy = (s->E >= 0.80f * s->Edes);

                // CRITICAL COORDINATE SAFETY CHECK: Only brake when near upright.  Instead of
                // using a fixed ±90° window, rely on the configured catch threshold
                // p->theta_catch to define the “near upright” region.  The value of
                // theta_catch is set during initialisation (e.g. 20°).  This prevents
                // triggering the brake when the pendulum is still far from vertical.
                float upright_distance = fabsf(current_theta);
                bool near_upright_region = (upright_distance < p->theta_catch);

                if (!near_upright_region) {
                    // If the pendulum is hanging down (bottom region), predictive braking is disabled to
                    // avoid fighting the swing‑up.  Print a periodic debug message to aid tuning.
                    static int bottom_brake_debug = 0;
                    if (++bottom_brake_debug % 100 == 0) {
                        // Note: the near‑upright region is defined by theta_catch (in radians).
                        float catch_deg = p->theta_catch * 180.0f / (float)M_PI;
                        printf("BRAKE SAFETY: Disabled at bottom θ_u=%.1f° (must be within ±%.1f° of upright)\n",
                               current_theta * 180.0f / (float)M_PI, catch_deg);
                    }
                    // Do not brake when far from upright
                    should_brake = false;
                    sufficient_energy = false;
                } else {
                    // Near upright region: determine whether to trigger braking based on position,
                    // velocity and predicted timing.  First, if we are within 25° of upright and
                    // moving slowly, we can initiate braking even if energy is low.  This avoids
                    // overshooting when the pendulum nearly reaches upright with low velocity.
                    if (upright_distance < 25.0f * M_PI/180.0f && abs_omega > 0.5f) {
                        should_brake = true;
                        sufficient_energy = true;  // override energy requirement for proximity
                        printf("UPRIGHT PROXIMITY BRAKE: θ_u=%.1f°, ω=%.1f rad/s\n",
                               current_theta * 180/M_PI, current_omega);
                    }
                    // Otherwise, check if we are at the predicted brake start position within a
                    // tolerance.  We allow a ±2° window around the calculated brake angle.
                    else if (fabsf(current_theta - brake_start_angle) < 2.0f * M_PI/180.0f) {
                        should_brake = true;
                        printf("INTELLIGENT BRAKE START: θ_u=%.1f°, advance=%.1fms, brake_time=%.1fms\n",
                               current_theta * 180/M_PI, optimal_advance_time * 1000, optimal_brake_time * 1000);
                    }
                    // Or, if the time to the brake start is positive and within a few timesteps, we
                    // trigger the brake based on predictive timing.  We allow up to five timesteps
                    // into the future to account for simulation step sizes.
                    else if (time_to_brake_start > 0 && time_to_brake_start <= 5.0f * dt) {
                        should_brake = true;
                        printf("PHYSICS-PREDICTIVE BRAKE: θ_u=%.1f°, in %.1fms\n",
                               current_theta * 180/M_PI, time_to_brake_start * 1000);
                    }
                }
                
                // Apply physics-based intelligent braking
                if (should_brake && sufficient_energy) {
                    // Removed arrow markers from debug output
                    printf("INTELLIGENT ACTIVE POSITIONING: θ_u=%.1f°, optimal_advance=%.1fms, brake_time=%.1fms\n",
                           current_theta * 180/M_PI, optimal_advance_time * 1000, optimal_brake_time * 1000);
                    
                    // PHYSICS-OPTIMIZED ACTIVE POSITIONING SYSTEM
                    // Use the calculated optimal braking parameters for effective positioning
                    
                    // Component 1: VELOCITY BRAKE TORQUE - use optimal brake time
                    float angular_momentum = moment_of_inertia * s->omega;
                    float velocity_brake_torque = -angular_momentum / optimal_brake_time;  // Use calculated optimal time
                    
                    // Component 2: POSITION CORRECTION TORQUE - adaptive based on speed
                    // In theta_u convention: target is 0 (upright), current is theta_u
                    // Position error is simply current_theta since target is 0
                    float position_error_theta_u = current_theta;  // Direct error from upright (0)
                    
                    // Wrap to shortest path: if |error| > π, wrap around
                    if (position_error_theta_u > M_PI) {
                        position_error_theta_u -= 2.0f * M_PI;  // Wrap to negative side
                    } else if (position_error_theta_u < -M_PI) {
                        position_error_theta_u += 2.0f * M_PI;  // Wrap to positive side
                    }
                    
                    // Adaptive position gain based on speed - higher speeds need gentler correction
                    // GENTLER POSITIONING: Reduce gains to prevent velocity decay
                    float speed_factor = fmaxf(0.3f, 1.0f - abs_omega / 8.0f);  // More aggressive reduction at high speeds
                    float adaptive_position_gain = 1.0f * speed_factor;  // Reduced from 2.0f to 1.0f
                    float position_torque = -position_error_theta_u * moment_of_inertia * adaptive_position_gain / (dt * dt);
                    
                    // Component 3: ANTICIPATORY TORQUE - predict where we'll be (much gentler)
                    float velocity_projection = current_omega * optimal_advance_time;  // Where we'll be at brake time
                    float anticipatory_gain = 0.75f * speed_factor;  // Reduced from 1.5f to 0.75f
                    float anticipatory_torque = -velocity_projection * moment_of_inertia * anticipatory_gain / (dt * dt);
                    
                    // TOTAL POSITIONING TORQUE
                    float total_required_torque = velocity_brake_torque + position_torque + anticipatory_torque;
                    
                    // GENTLE CONTROL AUTHORITY - reduce to prevent over-braking
                    float base_authority = 0.8f;  // Reduced from 1.5f to 0.8f for gentler control
                    float speed_authority_multiplier = fmaxf(0.6f, 1.0f - abs_omega / 12.0f);  // More conservative
                    float intelligent_authority = base_authority * speed_authority_multiplier;
                    total_required_torque *= intelligent_authority;
                    
                    // ADAPTIVE MOTOR COMMAND CALCULATION
                    // Use speed-adaptive motor scaling for effective braking
                    float adaptive_u_to_tau = p->u_to_tau;
                    
                    // Speed-based scaling for high-speed scenarios
                    if (abs_omega > 7.0f) {  // Above safe braking threshold
                        float target_brake_cmd = 0.90f;  // Target 90% motor authority
                        adaptive_u_to_tau = fabsf(total_required_torque) / target_brake_cmd;
                    }
                    
                    // Convert torque to motor command with adaptive scaling
                    float brake_cmd = total_required_torque / adaptive_u_to_tau;
                    
                    // Apply INTELLIGENT ACTIVE POSITIONING command
                    u = brake_cmd;
                    
                    // Removed arrow markers from debug output
                    printf("INTELLIGENT POSITIONING: vel_brake=%.4f, pos_torque=%.4f, anticipatory=%.4f, total=%.4f Nm, cmd=%.3f\n", 
                           velocity_brake_torque, position_torque, anticipatory_torque, total_required_torque, brake_cmd);
                    
                    // Transition to BALANCE mode
                    s->state = (decltype(s->state))ST_BALANCE;
                    ctrl_reset_integrator(s);
                    break;
                } 
                else {
                    // Continue energy pumping to reach upright with adaptive braking overlay
                    u = energy_control(p, s);
                    
                    // Apply motor scaling to the energy control output
                    u *= motor_scale;
                    
                    // 6. ADAPTIVE BRAKING DURING UPSWING
                    // Apply adaptive braking duty ON TOP of normal energy pump commands
                    // This ensures smooth energy bleed without fighting the pump
                    if (s->adaptive_brake_duty > 0.0f) {
                        // Calculate braking component that opposes current motion
                        float adaptive_brake_component = -sgn(current_omega) * s->adaptive_brake_duty;
                        
                        // Combine energy pump and adaptive brake using weighted sum
                        // Higher energy ratios get more braking influence
                        // float energy_ratio = (s->Edes > 1e-6f) ? (s->E / s->Edes) : 0.0f; // REMOVED: Redeclaration error
                        float current_energy_ratio = (s->Edes > 1e-6f) ? (s->E / s->Edes) : 0.0f;
                        float brake_weight = fminf((current_energy_ratio - 1.02f) * 5.0f, 0.5f); // Max 50% brake influence
                        brake_weight = fmaxf(0.0f, brake_weight);
                        
                        // Weighted combination: mostly energy pump with adaptive brake overlay
                        u = u * (1.0f - brake_weight) + adaptive_brake_component * brake_weight;
                        
                        printf("ADAPTIVE BRAKE OVERLAY: pump=%.3f, brake=%.3f, weight=%.3f, final=%.3f\n",
                               u / motor_scale, adaptive_brake_component, brake_weight, u);
                    }
                }
                } // End of spin-out check
            }
            break;
        }
            
            case ST_CATCH: { // CATCH - Legacy state, now bypassed for direct swing-up to balance
            // This state is no longer used - swing-up transitions directly to balance
            // If somehow we end up here, just go to balance mode
            s->state = (decltype(s->state))ST_BALANCE;
            ctrl_reset_integrator(s);
                // Informative debug without arrow markers
                printf("CATCH state bypassed, going directly to BALANCE\n");
            break;
        }
        
        case ST_BALANCE: { // BALANCE - Industrial textbook control with unrestricted power for catching
            // TEXTBOOK INDUSTRIAL BALANCE CONTROL
            // Core principle: Use maximum available motor authority to maintain balance
            // Only thermal protection can limit power - no artificial software limits
            
            float abs_omega = fabsf(s->omega);
            float abs_theta = fabsf(s->theta_u);
            
            // Balance loss condition - wider thresholds for better hysteresis
            if ((abs_theta > p->theta_catch) || (abs_omega > p->omega_catch)) {
                // INDUSTRIAL EMERGENCY BRAKING - UNRESTRICTED POWER
                // Calculate maximum available torque from motor specifications
                float moment_of_inertia = p->m * p->L * p->L;
                
                // Method 1: Direct momentum cancellation approach (industrial standard)
                float angular_momentum = moment_of_inertia * s->omega;
                float stop_time = 0.3f; // Aggressive 300ms stop time for industrial control
                float required_torque = fabsf(angular_momentum) / stop_time;
                
                // Method 2: Energy dissipation approach for high speeds
                float kinetic_energy = 0.5f * moment_of_inertia * abs_omega * abs_omega;
                float energy_torque = kinetic_energy / (abs_omega * stop_time);
                
                // Use the larger torque requirement (more conservative)
                float max_required_torque = fmaxf(required_torque, energy_torque);
                
                // ADAPTIVE MOTOR SCALING for emergency braking - prevent clamping at high speeds
                float adaptive_u_to_tau = p->u_to_tau;
                if (abs_omega > 7.0f) {  // Above safe braking threshold
                    // Scale u_to_tau to ensure brake_cmd ≤ 0.95 before speed multipliers
                    float target_brake_cmd = 0.95f;
                    adaptive_u_to_tau = max_required_torque / target_brake_cmd;
                    // Informative debug without arrow markers
                    printf("EMERGENCY ADAPTIVE SCALING: ω=%.1f rad/s, standard=%.4f, adaptive=%.4f\n",
                           abs_omega, p->u_to_tau, adaptive_u_to_tau);
                }
                
                // Convert to motor command with adaptive scaling
                float brake_cmd = max_required_torque / adaptive_u_to_tau;
                
                // Scale by speed - faster requires more aggressive braking
                float speed_multiplier = 1.0f;
                if (abs_omega > 15.0f) {        // > 2.4 rev/s - maximum emergency
                    speed_multiplier = 4.0f;
                    // Notify of maximum emergency brake without arrow markers
                    printf("MAXIMUM EMERGENCY BRAKE: ω=%.1f rad/s (%.2f rev/s)\n", 
                           s->omega, s->omega / 6.28f);
                } else if (abs_omega > 10.0f) { // > 1.6 rev/s - high emergency  
                    speed_multiplier = 3.0f;
                    // Notify of high emergency brake without arrow markers
                    printf("HIGH EMERGENCY BRAKE: ω=%.1f rad/s (%.2f rev/s)\n", 
                           s->omega, s->omega / 6.28f);
                } else if (abs_omega > 5.0f) {  // > 0.8 rev/s - standard emergency
                    speed_multiplier = 2.0f;
                    // Notify of standard emergency brake without arrow markers
                    printf("STANDARD EMERGENCY BRAKE: ω=%.1f rad/s (%.2f rev/s)\n", 
                           s->omega, s->omega / 6.28f);
                } else {
                    // Low speed - switch to swing-up instead of continued braking
                    s->state = (decltype(s->state))ST_SWINGUP;
                    ctrl_reset_integrator(s);
                    // Notify of balance loss without arrow markers
                    printf("BALANCE LOST - LOW SPEED: switching to SWING-UP\n");
                    break;
                }
                
                // Apply UNRESTRICTED braking command - only thermal protection can limit
                u = -sgn(s->omega) * brake_cmd * speed_multiplier;
                
                // NO SOFTWARE LIMITS - let thermal protection handle motor safety
                // Informative debug for industrial brake without arrow markers
                printf("INDUSTRIAL BRAKE: req_torque=%.4f Nm, cmd=%.4f (x%.1f speed factor)\n", 
                       max_required_torque, u, speed_multiplier);
                
            } else {
                // AGGRESSIVE PULL-TO-BALANCE CONTROLLER - DESIGNED TO OVERCOME STATIC FRICTION
                // Strategy: Use much higher gains and intelligent deadband to actually pull pendulum into place
                
                float theta_u_wrapped = wrap_pi(s->theta_u);
                float abs_theta = fabsf(theta_u_wrapped);
                
                // DYNAMIC GAIN SCHEDULING: Much higher gains when close to upright to overcome static friction
                float distance_factor = 1.0f;
                if (abs_theta < 15.0f * M_PI / 180.0f) {        // Within 15 degrees
                    distance_factor = 8.0f;  // 8x higher gains for final pull-in
                    // Informative debug for close-range pull-in without arrow markers
                    printf("CLOSE-RANGE PULL-IN: Using 8x gain boost\n");
                } else if (abs_theta < 30.0f * M_PI / 180.0f) { // Within 30 degrees  
                    distance_factor = 4.0f;  // 4x higher gains for medium range
                    // Informative debug for medium-range pull-in without arrow markers
                    printf("MEDIUM-RANGE PULL-IN: Using 4x gain boost\n");
                } else {
                    distance_factor = 2.0f;  // 2x standard gains for far range
                }
                
                // EXTREME AGGRESSIVE PID GAINS - designed to forcefully overcome static friction and ACTUALLY MOVE the pendulum
                float Kp_aggressive = p->Kp * distance_factor * 5.0f;  // EXTREME proportional gain (was 3.0f)
                float Kd_aggressive = p->Kd * distance_factor * 3.0f;  // Higher derivative for stability (was 2.0f)  
                float Ki_aggressive = p->Ki * distance_factor * 6.0f;  // EXTREME integral to break through friction (was 4.0f)
                
                // BREAKTHROUGH FORCE THRESHOLD: Use much higher minimum force to guarantee movement
                float min_force_threshold = 0.25f;  // Minimum 25% motor authority to breakthrough friction (was 15%)
                
                // Calculate PID terms
                float proportional = -Kp_aggressive * theta_u_wrapped;
                float derivative = -Kd_aggressive * s->omega;
                float integral_term = Ki_aggressive * theta_u_wrapped;
                
                // AGGRESSIVE INTEGRAL BUILDUP for overcoming static friction
                s->ui += integral_term;
                float max_integral = 5.0f * distance_factor; // Allow massive integral buildup for pulling
                s->ui = clampf(s->ui, -max_integral, max_integral);
                
                // Combine PID terms
                u = proportional + derivative + s->ui;
                
                // Add enhanced feed-forward compensation
                float gravity_compensation = (p->m * GRAVITY_ACCEL * p->L * 0.5f * sinf(theta_u_wrapped)) / p->u_to_tau;
                u += gravity_compensation;
                
                // STATIC FRICTION OVERRIDE: If command is too small, boost it to overcome friction
                if (fabsf(u) > 0.01f && fabsf(u) < min_force_threshold) {
                    float friction_boost = min_force_threshold * sgn(u);
                    u = friction_boost;
                }
                
                // PERSISTENT BALANCE MAINTENANCE: Even when very close to center, maintain active control
                // This prevents the pendulum from "giving up" when perfectly balanced
                if (abs_theta < 5.0f * M_PI / 180.0f && fabsf(s->omega) < 1.0f) {  // Very close to vertical and slow
                    // Apply gentle but persistent centering force to maintain active balance
                    float persistent_force = -sgn(theta_u_wrapped) * 0.10f;  // 10% persistent centering force
                    if (fabsf(u) < fabsf(persistent_force)) {  // Only if current command is weaker
                        u = persistent_force;
                    }
                }
                
                // DEADBAND SNAP-TO-CENTER: If very close to center with low velocity, apply strong centering force
                if (abs_theta < 2.0f * M_PI / 180.0f && fabsf(s->omega) < 0.3f) {
                    float snap_force = -sgn(theta_u_wrapped) * 0.25f;  // 25% snap force toward center
                    u = snap_force;
                }
                
                // Add velocity-dependent damping for stability
                if (abs_omega > 2.0f) {  // Lower threshold for damping activation
                    float extra_damping = -sgn(s->omega) * abs_omega * 0.3f;  // Stronger damping
                    u += extra_damping;
                }
                
                // MINIMUM CONTROL AUTHORITY: Never let the controller output fall below threshold that can maintain balance
                if (fabsf(u) < 0.05f && (abs_theta > 0.5f * M_PI / 180.0f || fabsf(s->omega) > 0.1f)) {
                    // If pendulum is not perfectly centered, ensure minimum control authority
                    float min_balance_force = 0.08f * sgn(u);  // 8% minimum continuous balance force
                    if (u == 0.0f) min_balance_force = -sgn(theta_u_wrapped) * 0.08f;  // Use angle if no existing command
                    u = min_balance_force;
                }
            }
            break;
        }
        
        case ST_CALIB:
        case ST_FAULT:
        default:
            // Handle other states or do nothing
            break;
    }
    
    // INDUSTRIAL CONTROL: No artificial software limits on motor commands
    // Only thermal protection system should limit motor power for safety
    // This allows full motor authority for catching and emergency braking
    return u;
}

float ctrl_step(const ctrl_params_t *p, ctrl_state_t *s) {
    return s->u = ctrl_update((ctrl_params_t*)p, s);
}

// FIX: PC Debug GUI Application with corrected initialization
class PCDebugSimulator {
private:
    SDL_Window* window;
    SDL_GLContext gl_context;
    bool running;
    
    // Simulation objects - SAME CONTROL CODE AS PICO
    PendulumPhysics physics;
    ctrl_params_t control_params;
    ctrl_state_t control_state;
    drv8833_t motor_driver;  // Motor protection system - same as embedded

    // Virtual encoder to integrate angular velocity and allow predictive queries.
    // This tracks the true upright‑referenced angle (0 = upright) without
    // wrapping, so we can detect crossings even at very high speeds.
    VirtualEncoder encoder;
    
    // Debug/visualization data
    std::vector<std::tuple<float, float, float, float, float, int>> data_log;
    float sim_time;
    bool paused, logging_enabled;
    
    // Debug flag for tracking disturbance application
    int disturbance_debug_cycles;
    
    // FPS and display control
    bool show_fps_display;
    bool is_fullscreen;
    
public:
    PCDebugSimulator() : window(nullptr), gl_context(nullptr), running(true),
                        sim_time(0), paused(false), logging_enabled(false),
                        disturbance_debug_cycles(0), show_fps_display(false), is_fullscreen(false) {
        // Initialise the virtual encoder at upright (0 rad).  Use the C API to
        // set up the encoder so that it can be shared across PC and Pico builds.
        ve_init(&encoder, 0.0f);
        initSDL();
        initImGui();
        initControlSystem();
        reset();
    }
    
    // [Keep all your original init functions - they're fine]
    void initSDL() {
        if (SDL_Init(SDL_INIT_VIDEO) < 0) {
            throw std::runtime_error("SDL initialization failed");
        }
        
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
        SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
        
        window = SDL_CreateWindow("Pendulum Control System - Enhanced Physics",
            SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 1200, 800,
            SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE);
        
        if (!window) throw std::runtime_error("Window creation failed");
        
        gl_context = SDL_GL_CreateContext(window);
        SDL_GL_MakeCurrent(window, gl_context);
        SDL_GL_SetSwapInterval(1);
        
        if (gl3wInit() != 0) throw std::runtime_error("OpenGL initialization failed");
    }
    
    void initImGui() {
        IMGUI_CHECKVERSION();
        ImGui::CreateContext();
        ImGuiIO& io = ImGui::GetIO(); (void)io;
        
        // Enable DPI scaling for high-DPI displays (compatible with older ImGui)
        // Auto-detect system DPI scaling
        float dpi_scale = 1.0f;
        int display_w, display_h;
        SDL_GetWindowSize(window, &display_w, &display_h);
        int drawable_w, drawable_h;
        SDL_GL_GetDrawableSize(window, &drawable_w, &drawable_h);
        
        if (display_w > 0 && drawable_w > display_w) {
            dpi_scale = (float)drawable_w / (float)display_w;
            printf("Detected DPI scale factor: %.2f\n", dpi_scale);
        }
        
        // Apply DPI scaling to fonts and UI elements (compatible method)
        io.FontGlobalScale = dpi_scale;
        io.DisplayFramebufferScale = ImVec2(dpi_scale, dpi_scale);
        
        // Scale the default style for high-DPI displays
        ImGuiStyle& style = ImGui::GetStyle();
        style.ScaleAllSizes(dpi_scale);
        
        ImGui::StyleColorsDark();
        ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
        ImGui_ImplOpenGL3_Init("#version 330");
    }
    
    void initControlSystem() {
        // Initialize with realistic parameters for the simple model
        ctrl_init(&control_params, &control_state);
        
        control_params.m = 0.004f;         // 4g rod (default)
        control_params.L = 0.3048f;      // 12 inch rod (default)
        control_params.Jm = 1e-5f;       // Small motor inertia
        control_params.b_vis = 0.0002f;  // Light damping
        control_params.tau_ff = 0.001f;  // Friction
        control_params.dt = 1.0f / CONTROL_HZ;
        
        // CRITICAL FIX: Scale motor parameters based on pendulum LEVER ARM (m × L)
        // Lever equation: Torque = Force × Distance = (m × g) × L
        // Default reference: 4g pendulum at 12 inches (0.3048m)
        float default_lever_arm = 0.004f * 0.3048f;  // kg⋅m
        float current_lever_arm = control_params.m * control_params.L;  // kg⋅m
        float lever_ratio = current_lever_arm / default_lever_arm;
        
        // Use square root scaling for gentler response (industrial practice)
        float motor_scaling_factor = sqrtf(lever_ratio);
        
        // Apply minimum thresholds for very light/short pendulums
        float min_scaling_factor = 0.05f;  // Minimum 5% of default power
        motor_scaling_factor = fmaxf(motor_scaling_factor, min_scaling_factor);
        
        // Scale maximum torque based on lever arm requirements
        control_params.tau_max = 0.050f * motor_scaling_factor; // Increased max torque for energy pumping
        
        // Scale voltage-to-torque conversion (motor authority)
        // SIGNIFICANTLY INCREASED for effective energy pumping from bottom position
        control_params.u_to_tau = 0.050f * motor_scaling_factor; // 10x increase for strong energy pumping
        
        // Energy gain scaling - longer/heavier pendulums need different energy pumping
        control_params.k_energy = ENERGY_PUMP_GAIN_MULTIPLIER * sqrtf(lever_ratio); // From config.h
        control_params.swing_sat = 1.0f;
        
        // AGGRESSIVE PID gains designed to overcome static friction and pull pendulum into place
        control_params.Kp = 25.0f;      // Much higher proportional gain for pulling force (was 12.0f)
        control_params.Kd = 4.0f;       // Higher derivative gain for stability (was 2.0f)
        control_params.Ki = 2.0f;       // Much higher integral gain to overcome friction (was 0.5f)
        control_params.ui_max = 1.0f;   // Allow larger integral buildup (was 0.2f)
        control_params.balance_sat = 1.0f; // Full balance mode authority (was 0.5f)
        
        // Tighter catch thresholds for transition to balance mode.  By reducing
        // theta_catch and omega_catch, we prevent the controller from declaring
        // balance when the pendulum is still far from upright and moving quickly.
        control_params.theta_catch = 20.0f * M_PI / 180.0f; // ±20° catch zone
        control_params.omega_catch = OMEGA_CATCH_PC; // From config.h
        
        // Set target energy for swing-up (increased to 3.0x to actually reach upright)
        control_state.Edes = CALCULATE_ENERGY_TARGET(control_params.m, control_params.L); // From config.h
        
        // Configure physics with same parameters
        physics.setParameters(control_params.m, control_params.L, 
                            control_params.b_vis, control_params.tau_ff);
        physics.setTimestep(control_params.dt);
        
        // Initialize motor driver with FIXED settings for PC simulation
        drv8833_init(&motor_driver, 14, 15, 1000.0f);  // 1 kHz PWM
        
        printf("Control system initialized with working PWM simulation\n");
    }
    
    void update() {
        static auto last_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        float real_dt = std::chrono::duration<float>(current_time - last_time).count();
        last_time = current_time;
        
        if (!paused) {
            // Limit simulation steps to prevent excessive CPU load
            int steps = (int)(real_dt / control_params.dt);
            if (steps < 1) steps = 1;
            if (steps > 10) steps = 10; // Reduced from 100 to 10 for better performance
            
                for (int i = 0; i < steps; i++) {
                // Enhanced physics coordinate system handling
                // Acquire angular velocity from the physics engine.  Update the
                // virtual encoder with this velocity and integrate it forward by the
                // control timestep.  This allows us to reconstruct the unwrapped
                // upright‑referenced angle even when the pendulum spins multiple
                // revolutions between updates.
                control_state.omega = physics.getOmega();
                ve_set_velocity(&encoder, control_state.omega);
                ve_update(&encoder, control_params.dt);
                // Use the encoder’s unwrapped angle, wrapped to [−π, π], as the
                // controller’s upright‑referenced angle.  This mirrors the Pi Pico
                // behaviour by using the same C API for angle wrapping.
                control_state.theta_u = ve_wrap_to_pi(ve_angle(&encoder));
                // Also update the bottom‑referenced angle for compatibility.  This
                // value continues to be taken from the physics engine because the
                // bottom reference is used only for visualisation and does not require
                // unwrapped tracking.
                control_state.theta_b = physics.getThetaBottom();

                // NOTE: Removed velocity-based anti-spin emergency braking
                // Anti-spin should only trigger after completing full rotations, not based on speed alone
                // The control algorithm itself handles rotation-based spin-out detection
                
                float motor_cmd;
                if (drv8833_is_protection_active(&motor_driver)) {
                    // Motor protection (including thermal) is active - command zero and wait
                    motor_cmd = 0.0f;
                    printf("MOTOR PROTECTION ACTIVE: Setting motor_cmd=0.0f\n");
                } else {
                    // Normal operation - run control algorithm
                    motor_cmd = ctrl_step(&control_params, &control_state);
                    
                    // DISTURBANCE DEBUGGING: Show detailed control behavior after disturbance
                    if (disturbance_debug_cycles > 0) {
                        printf("--- POST-DISTURBANCE CONTROL CYCLE %d ---\n", 11 - disturbance_debug_cycles);
                        printf("Physics: theta=%.6f rad (%.3f°), omega=%.6f rad/s\n", 
                               physics.getTrueTheta(), physics.getTrueTheta() * 180.0f / M_PI, physics.getTrueOmega());
                        printf("         angle_from_bottom=%.6f rad (%.3f°)\n",
                               fmod(physics.getTrueTheta() + M_PI, 2*M_PI) - M_PI,
                               (fmod(physics.getTrueTheta() + M_PI, 2*M_PI) - M_PI) * 180.0f / M_PI);
                        printf("         energy=%.6f J, target=%.6f J (%.1f%%)\n",
                               physics.getEnergy(), control_state.Edes, 
                               100.0f * physics.getEnergy() / control_state.Edes);
                        printf("Control: state=%s, motor_cmd=%.6f V\n",
                               (control_state.state == ST_BALANCE) ? "BALANCE" : 
                               (control_state.state == ST_SWINGUP) ? "SWINGUP" : "UNKNOWN",
                               motor_cmd);
                        printf("         theta_u=%.6f rad (%.3f°), omega=%.6f rad/s\n",
                               control_state.theta_u, control_state.theta_u * 180.0f / M_PI, control_state.omega);
                        printf("         energy_err=%.6f J, integral=%.6f\n",
                               control_state.E - control_state.Edes, control_state.ui);
                        disturbance_debug_cycles--;
                        if (disturbance_debug_cycles == 0) {
                            printf("--- END POST-DISTURBANCE DEBUGGING ---\n\n");
                        }
                    }
                }

                // Apply motor command to DRV8833 PWM simulation with motor speed for protection
                // Use normal command with thermal protection
                drv8833_cmd(&motor_driver, motor_cmd);
                
                // CRITICAL FIX: Always step the motor simulation to update protection state
                // Step the PWM simulation forward (this updates the internal state and motor protection)
                drv8833_step_simulation(&motor_driver, control_params.dt, physics.getOmega());
                
                // MOTOR PROTECTION: Check if protection is active and reset to swing-up if needed
                static bool protection_reset_printed = false;
                
                if (drv8833_is_protection_active(&motor_driver) || drv8833_is_emergency_brake_active(&motor_driver)) {
                    // Reset control system to swing-up mode when protection activates
                    if (control_state.state != ST_SWINGUP) {
                        control_state.state = (decltype(control_state.state))ST_SWINGUP;
                        control_state.ui = 0.0f;  // Reset integrator
                        control_state.E = 0.0f;   // Reset energy
                        // CRITICAL FIX: Use Pico working energy target value
                        control_state.Edes = CALCULATE_ENERGY_TARGET(control_params.m, control_params.L); // From config.h
                        
                        if (!protection_reset_printed) {
                            // Informative debug without arrow markers; unify both thermal and normal protection messages
                            printf("MOTOR PROTECTION ACTIVE: System reset to SWING-UP mode (Edes=%.3f J)\n", control_state.Edes);
                            protection_reset_printed = true;
                        }
                    }
                } else {
                    protection_reset_printed = false;  // Reset print flag when protection clears
                }
                
                // CRITICAL FIX: Use the motor simulation output which includes protection
                // Let the DRV8833 simulation handle protection internally
                float motor_voltage = 0.0f;
                float sim_voltage, sim_current, sim_torque;
                drv8833_get_simulation_state(&motor_driver, &sim_voltage, &sim_current, &sim_torque);
                motor_voltage = sim_voltage; // This voltage already includes protection scaling
                
                // Apply the PWM voltage to enhanced physics motor model
                physics.setMotorVoltage(motor_voltage);
                physics.step();

                sim_time += control_params.dt;

                // Log data for debugging - reduced frequency for performance
                if (logging_enabled && (data_log.empty() || 
                    sim_time - std::get<0>(data_log.back()) >= 0.05f)) { // Reduced to 20Hz logging
                    data_log.push_back(std::make_tuple(sim_time, 
                        control_state.theta_u * 180/M_PI, control_state.omega,
                        motor_cmd, physics.getEnergy(), control_state.state));
                }
            }
        }
    }
    
    void render() {
        // Clear with a simple color to reduce GPU load
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        
        // Optimize ImGui rendering
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        
        // Performance optimization: Reduce UI complexity when paused
        if (!paused || ImGui::GetFrameCount() % 2 == 0) {
            renderControlPanel();
            renderVisualization();
            renderDataLogging();
        } else {
            // Minimal UI when paused to save GPU
            ImGui::Begin("Control Panel - Enhanced Physics");
            ImGui::Text("SIMULATION PAUSED - Press Space to Resume");
            if (ImGui::Button("Resume")) paused = false;
            ImGui::End();
        }
        
        // Render with optimizations
        ImGui::Render();
        
        // Optimize for different window sizes
        int display_w, display_h;
        SDL_GetWindowSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        
        // Use adaptive vsync for better performance in fullscreen
        static bool vsync_adaptive_tried = false;
        if (!vsync_adaptive_tried) {
            // Try adaptive vsync first (reduces lag in fullscreen)
            if (SDL_GL_SetSwapInterval(-1) == 0) {
                printf("Adaptive VSync enabled\n");
            } else {
                SDL_GL_SetSwapInterval(1); // Fallback to regular vsync
                printf("Regular VSync enabled\n");
            }
            vsync_adaptive_tried = true;
        }
        
        SDL_GL_SwapWindow(window);
    }
    
    void renderControlPanel() {
        ImGui::Begin("Control Panel - Enhanced Physics");
        
        if (ImGui::Button(paused ? "Resume" : "Pause")) paused = !paused;
        ImGui::SameLine();
        if (ImGui::Button("Reset")) reset();
        
        ImGui::Separator();
        
        // Enhanced physics status display
        ImGui::Text("=== ENHANCED PHYSICS STATUS ===");
        ImGui::Text("Simulation Time: %.2f s", sim_time);
        ImGui::Text("Angle from upright: %.3f rad (%.1f deg)", 
                   control_state.theta_u, control_state.theta_u * 180/M_PI);
        ImGui::Text("Angular velocity: %.3f rad/s", control_state.omega);
        ImGui::Text("Motor command: %.3f", control_state.u);
        
        // Show motor scaling based on lever arm (mass × length) - CRITICAL FEEDBACK
        float default_lever_arm = 0.004f * 0.3048f; // Default 4g at 12 inches
        float current_lever_arm = control_params.m * control_params.L;
        float lever_scaling = current_lever_arm / default_lever_arm;
        ImGui::Text("Lever Arm Scaling: %.2fx (%.6f kg⋅m vs %.6f default)", 
                   lever_scaling, current_lever_arm, default_lever_arm);
        ImGui::Text("Max Torque: %.4f Nm | Voltage→Torque: %.4f", control_params.tau_max, control_params.u_to_tau);
        
        // Motor protection status display - ENHANCED WITH DEBUG INFO
        ImGui::Separator();
        ImGui::Text("=== MOTOR PROTECTION STATUS ===");
        bool protection_active = drv8833_is_protection_active(&motor_driver);
        bool emergency_brake = drv8833_is_emergency_brake_active(&motor_driver);
        bool thermal_halt = drv8833_is_thermal_halted(&motor_driver);
        float energy_used = drv8833_get_protection_energy_used(&motor_driver);
        float utilization = drv8833_get_protection_utilization(&motor_driver);
        
        // Debug: Print values to console periodically
        static int debug_protection_counter = 0;
        if (++debug_protection_counter % 100 == 0) { // Every ~1 second
            printf("MOTOR PROTECTION DEBUG: active=%d, emergency=%d, halt=%d, energy=%.3f, util=%.3f\n", 
                   protection_active, emergency_brake, thermal_halt, energy_used, utilization);
            
            // Additional debug: check motor protection internal state
            printf("  -> command=%.3f, output=%.3f, initialized=%d\n",
                   motor_driver.current_command,
                   motor_protection_get_output(&motor_driver.protection),
                   motor_driver.protection.initialized);
        }
        
        if (thermal_halt) {
            ImGui::TextColored(ImVec4(1, 0, 1, 1), "THERMAL HALT - WAITING FOR 90%% DECAY");
        } else if (emergency_brake) {
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "EMERGENCY BRAKE ACTIVE");
        } else if (protection_active) {
            ImGui::TextColored(ImVec4(1, 1, 0, 1), "Protection Active (%.1f%% reduction)", utilization * 50.0f);
        } else {
            ImGui::TextColored(ImVec4(0, 1, 0, 1), "Normal Operation");
        }
        
        ImGui::Text("Thermal Energy: %.3f J", energy_used);
        ImGui::Text("Protection Level: %.1f%%", utilization * 100.0f);
        
        // Motor Protection Gauge - Visual indicator
        ImGui::Separator();
        ImGui::Text("=== THERMAL PROTECTION GAUGE ===");
        
        // Circular gauge visualization
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 gauge_center = ImGui::GetCursorScreenPos();
        gauge_center.x += 60; // Offset from left edge
        gauge_center.y += 60; // Offset from top
        float gauge_radius = 50.0f;
        
        // Draw gauge background circle
        draw_list->AddCircle(gauge_center, gauge_radius, IM_COL32(100, 100, 100, 255), 32, 2.0f);
        
        // Calculate angles for protection and emergency thresholds
        float protection_angle = (energy_used / MOTOR_THERMAL_CAPACITY) * M_PI * 1.5f; // 1.5π = 270° max
        float emergency_angle = (energy_used / MOTOR_EMERGENCY_CAPACITY) * M_PI * 1.5f;
        
        // Clamp angles
        protection_angle = fminf(protection_angle, M_PI * 1.5f);
        emergency_angle = fminf(emergency_angle, M_PI * 1.5f);
        
        // Draw protection threshold arc (green to yellow)
        if (protection_angle > 0) {
            ImU32 prot_color = energy_used >= MOTOR_THERMAL_CAPACITY ? IM_COL32(255, 0, 0, 255) : 
                              energy_used >= (MOTOR_THERMAL_CAPACITY * 0.8f) ? IM_COL32(255, 128, 0, 255) : IM_COL32(0, 255, 0, 255);
            
            // Draw arc segments
            for (float a = 0; a < protection_angle; a += 0.1f) {
                float angle_start = a + M_PI * 0.75f; // Start at bottom left
                float angle_end = fminf(a + 0.1f, protection_angle) + M_PI * 0.75f;
                
                ImVec2 start_pos(gauge_center.x + cosf(angle_start) * (gauge_radius - 8),
                                gauge_center.y + sinf(angle_start) * (gauge_radius - 8));
                ImVec2 end_pos(gauge_center.x + cosf(angle_end) * (gauge_radius - 8),
                              gauge_center.y + sinf(angle_end) * (gauge_radius - 8));
                
                draw_list->AddLine(start_pos, end_pos, prot_color, 8.0f);
            }
        }
        
        // Draw threshold markers
        float prot_marker_angle = M_PI * 0.75f + M_PI * 1.5f; // Protection at 270°
        ImVec2 prot_marker_inner(gauge_center.x + cosf(prot_marker_angle) * (gauge_radius - 15),
                                gauge_center.y + sinf(prot_marker_angle) * (gauge_radius - 15));
        ImVec2 prot_marker_outer(gauge_center.x + cosf(prot_marker_angle) * gauge_radius,
                                gauge_center.y + sinf(prot_marker_angle) * gauge_radius);
        draw_list->AddLine(prot_marker_inner, prot_marker_outer, IM_COL32(255, 255, 0, 255), 3.0f);
        
        // Draw center circle with current reading
        draw_list->AddCircleFilled(gauge_center, 20, IM_COL32(50, 50, 50, 255));
        
        // Add text labels next to gauge
        ImVec2 text_pos(gauge_center.x + 70, gauge_center.y - 50);
        ImGui::SetCursorScreenPos(text_pos);
        ImGui::BeginGroup();
        ImGui::Text("THERMAL GAUGE");
        ImGui::Text("%.2f J", energy_used);
        if (energy_used >= MOTOR_THERMAL_CAPACITY) {
            ImGui::TextColored(ImVec4(1,0,0,1), "PROTECTION ON");
        } else if (energy_used >= (MOTOR_THERMAL_CAPACITY * 0.8f)) {
            ImGui::TextColored(ImVec4(1,0.5,0,1), "WARNING");
        } else {
            ImGui::TextColored(ImVec4(0,1,0,1), "NORMAL");
        }
        ImGui::EndGroup();
        
        // Reserve space for the gauge and labels
        ImGui::SetCursorScreenPos(ImVec2(gauge_center.x - 60, gauge_center.y + 70));
        
        // Calculate percentages for different thresholds
        float protection_percentage = (energy_used / MOTOR_THERMAL_CAPACITY) * 100.0f;  // Use actual protection threshold
        float emergency_percentage = (energy_used / MOTOR_EMERGENCY_CAPACITY) * 100.0f;   // Use actual emergency threshold
        
        // Clamp percentages to 100%
        protection_percentage = fminf(protection_percentage, 100.0f);
        emergency_percentage = fminf(emergency_percentage, 100.0f);
        
        // Protection threshold gauge (0-100% to protection activation)
        ImGui::Text("Protection Threshold (%.1fJ):", MOTOR_THERMAL_CAPACITY);
        ImGui::SameLine();
        ImGui::TextColored(protection_percentage >= 100.0f ? ImVec4(1,0,0,1) : 
                          protection_percentage >= 80.0f ? ImVec4(1,0.5,0,1) : ImVec4(0,1,0,1), 
                          "%.1f%%", protection_percentage);
        
        ImVec4 protection_color = protection_percentage >= 100.0f ? ImVec4(1,0,0,1) : 
                                 protection_percentage >= 80.0f ? ImVec4(1,0.5,0,1) : ImVec4(0,0.8,0,1);
        ImGui::ProgressBar(protection_percentage / 100.0f, ImVec2(300, 20), "");
        ImGui::SameLine(0, 10);
        ImGui::TextColored(protection_color, protection_percentage >= 100.0f ? "ACTIVE" : 
                          protection_percentage >= 80.0f ? "WARNING" : "NORMAL");
        
        // Emergency threshold gauge (0-100% to emergency brake)
        ImGui::Text("Emergency Threshold (%.1fJ):", MOTOR_EMERGENCY_CAPACITY);
        ImGui::SameLine();
        ImGui::TextColored(emergency_percentage >= 100.0f ? ImVec4(1,0,0,1) : 
                          emergency_percentage >= 60.0f ? ImVec4(1,0.5,0,1) : ImVec4(0,1,0,1), 
                          "%.1f%%", emergency_percentage);
        
        ImVec4 emergency_color = emergency_percentage >= 100.0f ? ImVec4(1,0,0,1) : 
                                emergency_percentage >= 60.0f ? ImVec4(1,0.5,0,1) : ImVec4(0,0.8,0,1);
        ImGui::ProgressBar(emergency_percentage / 100.0f, ImVec2(300, 20), "");
        ImGui::SameLine(0, 10);
        ImGui::TextColored(emergency_color, emergency_percentage >= 100.0f ? "EMERGENCY!" : 
                          emergency_percentage >= 60.0f ? "CAUTION" : "SAFE");
        
        // Numerical readout
        ImGui::Text("Current: %.3fJ | Protection: %.1fJ | Emergency: %.1fJ", energy_used, MOTOR_THERMAL_CAPACITY, MOTOR_EMERGENCY_CAPACITY);
        
        // Time-based decay indicator
        static float last_energy_reading = 0.0f;
        static auto last_reading_time = std::chrono::steady_clock::now();
        auto current_time = std::chrono::steady_clock::now();
        float time_diff = std::chrono::duration<float>(current_time - last_reading_time).count();
        
        if (time_diff > 0.5f) { // Update every 500ms
            float energy_change = energy_used - last_energy_reading;
            if (fabsf(energy_change) > 0.001f) {
                const char* trend_icon = energy_change > 0 ? "▲" : "▼";
                ImVec4 trend_color = energy_change > 0 ? ImVec4(1,0.3,0.3,1) : ImVec4(0.3,1,0.3,1);
                ImGui::SameLine();
                ImGui::TextColored(trend_color, "%s %.3fJ/s", trend_icon, energy_change / time_diff);
            }
            last_energy_reading = energy_used;
            last_reading_time = current_time;
        }
        
        // Add manual motor protection testing
        ImGui::Separator();
        ImGui::Text("=== MOTOR PROTECTION TESTING ===");
        if (ImGui::Button("Test High Load (5s)")) {
            // Apply high motor command for testing - Integration-based approach
            printf("Starting motor protection integration test...\n");
            uint32_t start_time = (uint32_t)(sim_time * 1000.0f);
            for (int i = 0; i < 50; i++) { // 50 integration steps over 5 seconds
                uint32_t test_time = start_time + (i * 100); // Every 100ms
                motor_protection_set_command(&motor_driver.protection, 0.9f, test_time, 0.0f); // Add motor speed parameter
                motor_protection_step(&motor_driver.protection, test_time);
                printf("Test step %d: cmd=0.9, time=%u, energy=%.3f\n", 
                       i, test_time, motor_protection_get_thermal_energy(&motor_driver.protection));
            }
            printf("Motor protection test complete. Final energy: %.3f, active: %d\n",
                   drv8833_get_protection_energy_used(&motor_driver),
                   drv8833_is_protection_active(&motor_driver));
        }
        ImGui::SameLine();
        if (ImGui::Button("Reset Protection")) {
            // Reset protection system by reinitializing
            drv8833_init(&motor_driver, 14, 15, 1000.0f);
            printf("Motor protection system reset\n");
        }
        
        // Add PWM simulation status
        float sim_voltage, sim_current, sim_torque;
        drv8833_get_simulation_state(&motor_driver, &sim_voltage, &sim_current, &sim_torque);
        ImGui::Separator();
        ImGui::Text("=== PWM SIMULATION STATUS ===");
        ImGui::Text("PWM Voltage: %.3f V", sim_voltage);
        ImGui::Text("Motor Current: %.3f A", sim_current);
        ImGui::Text("Motor Torque: %.4f Nm", sim_torque);
        
        // Motor Protection History Graph
        ImGui::Separator();
        ImGui::Text("=== THERMAL PROTECTION HISTORY ===");
        
        // Maintain a rolling buffer of thermal energy readings
        static std::vector<float> thermal_history;
        static float last_thermal_sample_time = 0.0f;
        
        // Sample thermal energy every 100ms for the graph
        if (sim_time - last_thermal_sample_time >= 0.1f) {
            thermal_history.push_back(energy_used);
            last_thermal_sample_time = sim_time;
            
            // Keep only last 300 samples (30 seconds at 10Hz sampling)
            if (thermal_history.size() > 300) {
                thermal_history.erase(thermal_history.begin());
            }
        }
        
        // Plot thermal energy over time with threshold lines
        if (!thermal_history.empty()) {
            // Find max value for scaling, but ensure we show thresholds
            float max_thermal = 30.0f; // Start with emergency threshold + margin
            for (float val : thermal_history) {
                if (val > max_thermal) max_thermal = val * 1.1f;
            }
            
            ImGui::PlotLines("Thermal Energy (J)", thermal_history.data(), thermal_history.size(), 
                           0, nullptr, 0.0f, max_thermal, ImVec2(400, 80));
            
            // Add threshold reference lines (drawn as text overlay)
            ImGui::SameLine();
            ImGui::BeginGroup();
            ImGui::TextColored(ImVec4(1,1,0,1), "%.1fJ - Protection", MOTOR_THERMAL_CAPACITY);
            ImGui::TextColored(ImVec4(1,0,0,1), "%.1fJ - Emergency", MOTOR_EMERGENCY_CAPACITY);
            ImGui::Text("Max: %.1fJ", max_thermal);
            ImGui::Text("Samples: %zu", thermal_history.size());
            ImGui::EndGroup();
        }
        
        ImGui::Separator();
        
        ImGui::Text("Energy: %.4f J (target: %.4f J)", 
                   physics.getEnergy(), control_state.Edes);
        
        const char* state_names[] = {"IDLE", "CALIB", "SWINGUP", "CATCH", "BALANCE", "FAULT"};
        ImGui::Text("Control State: %s", state_names[control_state.state]);
        
        // Add control debug information
        ImGui::Text("Current Energy: %.4f J", control_state.E);
        ImGui::Text("Energy Error: %.4f J", control_state.E - control_state.Edes);
        ImGui::Text("Angle from bottom: %.3f rad (%.1f deg)", 
                   fabsf(control_state.theta_u - M_PI), fabsf(control_state.theta_u - M_PI) * 180/M_PI);
        if (control_state.kick_active) {
            ImGui::Text("Kick Active: Direction %d", control_state.kick_direction);
        } else if (control_state.state == (decltype(control_state.state))ST_SWINGUP) {
            ImGui::Text("Energy Pumping: Drive Level %.2f", control_state.drive_level);
        }
        
        ImGui::Separator();
        
        // Control actions with clear labels
        if (ImGui::Button("Start Swing-Up")) {
            control_state.state = (decltype(control_state.state))ST_SWINGUP;
            ctrl_reset_integrator(&control_state);
            // CRITICAL FIX: Use Pico working energy target value (3.0f matches embedded implementation)
            control_state.Edes = CALCULATE_ENERGY_TARGET(control_params.m, control_params.L); // From config.h
            printf("Started swing-up control with Edes=%.3f J (INCREASED TARGET)\n", control_state.Edes);
        }
        ImGui::SameLine();
        if (ImGui::Button("Stop / Idle")) {
            control_state.state = (decltype(control_state.state))ST_IDLE;
            printf("Control stopped (idle mode)\n");
        }
        
        if (ImGui::Button("Force Balance Mode")) {
            control_state.state = (decltype(control_state.state))ST_BALANCE;
            ctrl_reset_integrator(&control_state);
            printf("Forced balance mode\n");
        }
        
        ImGui::Separator();
        
        // Enhanced physics model controls - RESTORED
        if (ImGui::CollapsingHeader("Enhanced Physics Model")) {
            static int integration_method = 2; // Default to RK2
            const char* methods[] = {"Euler", "Modified Euler", "RK2 (Heun)", "RK4"};
            if (ImGui::Combo("Integration Method", &integration_method, methods, 4)) {
                physics.setIntegrationMethod((EnhancedPendulumPhysics::IntegrationMethod)integration_method);
            }
            
            static float temperature = 25.0f;
            if (ImGui::SliderFloat("Temperature (°C)", &temperature, 0.0f, 80.0f)) {
                physics.setTemperature(temperature);
            }
            
            static float measurement_noise = 0.001f;
            static float process_noise = 0.0001f;
            if (ImGui::SliderFloat("Measurement Noise", &measurement_noise, 0.0f, 0.01f, "%.5f") ||
                ImGui::SliderFloat("Process Noise", &process_noise, 0.0f, 0.001f, "%.6f")) {
                physics.setNoise(measurement_noise, process_noise);
            }
            
            if (ImGui::Button("Show Physics Diagnostics")) {
                physics.printDiagnostics();
            }
        }
        
        // Control parameter tuning - THESE NOW UPDATE PROPERLY
        if (ImGui::CollapsingHeader("Control Parameters - LIVE TUNING")) {
            bool param_changed = false;
            
            // PID gains
            param_changed |= ImGui::SliderFloat("Kp", &control_params.Kp, 0.0f, 50.0f);
            param_changed |= ImGui::SliderFloat("Kd", &control_params.Kd, 0.0f, 10.0f);
            param_changed |= ImGui::SliderFloat("Ki", &control_params.Ki, 0.0f, 5.0f);
            param_changed |= ImGui::SliderFloat("Energy Gain", &control_params.k_energy, 0.0f, 5.0f);
            
            ImGui::Separator();
            
            // Physical parameters
            bool physics_changed = false;
            physics_changed |= ImGui::SliderFloat("Mass (kg)", &control_params.m, 0.05f, 1.0f, "%.3f");
            physics_changed |= ImGui::SliderFloat("Length (m)", &control_params.L, 0.1f, 0.8f, "%.3f");
            physics_changed |= ImGui::SliderFloat("Damping", &control_params.b_vis, 0.0f, 0.01f, "%.5f");
            
            ImGui::Separator();
            ImGui::Text("Motor Parameters");
            param_changed |= ImGui::SliderFloat("Max Torque (Nm)", &control_params.tau_max, 0.01f, 0.2f, "%.4f");
            param_changed |= ImGui::SliderFloat("Motor Inertia (kg⋅m²)", &control_params.Jm, 1e-6f, 1e-3f, "%.6f");
            param_changed |= ImGui::SliderFloat("Friction Torque (Nm)", &control_params.tau_ff, 0.0f, 0.01f, "%.4f");
            param_changed |= ImGui::SliderFloat("Voltage to Torque", &control_params.u_to_tau, 0.001f, 0.1f, "%.4f");
            
            ImGui::Separator();
            ImGui::Text("Control Limits");
            param_changed |= ImGui::SliderFloat("Balance Saturation", &control_params.balance_sat, 0.1f, 1.0f, "%.2f");
            param_changed |= ImGui::SliderFloat("Swing Saturation", &control_params.swing_sat, 0.1f, 1.0f, "%.2f");
            param_changed |= ImGui::SliderFloat("Integral Max", &control_params.ui_max, 0.1f, 1.0f, "%.2f");
            
            ImGui::Separator();
            ImGui::Text("Catch Thresholds");
            float theta_catch_deg = control_params.theta_catch * 180.0f / M_PI;
            if (ImGui::SliderFloat("Angle Catch (deg)", &theta_catch_deg, 10.0f, 60.0f, "%.1f")) {
                control_params.theta_catch = theta_catch_deg * M_PI / 180.0f;
                param_changed = true;
            }
            param_changed |= ImGui::SliderFloat("Speed Catch (rad/s)", &control_params.omega_catch, 1.0f, 10.0f, "%.1f");
            
            ImGui::Separator();
            ImGui::Text("Emergency Braking");
            param_changed |= ImGui::SliderFloat("Light Brake", &control_params.brake_light, 0.1f, 1.0f, "%.2f");
            param_changed |= ImGui::SliderFloat("Moderate Brake", &control_params.brake_moderate, 0.1f, 1.0f, "%.2f");
            param_changed |= ImGui::SliderFloat("Strong Brake", &control_params.brake_strong, 0.1f, 1.0f, "%.2f");
            param_changed |= ImGui::SliderFloat("Maximum Brake", &control_params.brake_maximum, 0.1f, 1.0f, "%.2f");
            
            if (param_changed || physics_changed) {
                // Update desired energy when parameters change (use Pico working value)
                control_state.Edes = CALCULATE_ENERGY_TARGET(control_params.m, control_params.L); // From config.h
                
                // Update physics parameters if they changed
                if (physics_changed) {
                    // CRITICAL ADAPTIVE FIX: Preserve energy ratio when mass/length changes
                    // Calculate current energy ratio before updating physics parameters
                    float current_energy_ratio = physics.getEnergy() / control_state.Edes;
                    
                    // Update physics simulation with new parameters  
                    physics.setParameters(control_params.m, control_params.L, 
                                        control_params.b_vis, control_params.tau_ff);
                    
                    // Recalculate energy target with new mass/length
                    float old_edes = control_state.Edes;
                    control_state.Edes = CALCULATE_ENERGY_TARGET(control_params.m, control_params.L);
                    
                    // DEBUG: Print energy target calculation details
                    printf("\n=== ENERGY TARGET RECALCULATION (Mass Slider) ===\n");
                    printf("Mass: %.3f kg, Length: %.3f m\n", control_params.m, control_params.L);
                    printf("Base energy: %.3f × 9.81 × (%.3f × 0.5) = %.4f J\n", 
                           control_params.m, control_params.L, 
                           control_params.m * 9.81f * (control_params.L * 0.5f));
                    printf("Old target: %.4f J → New target: %.4f J\n", old_edes, control_state.Edes);
                    printf("Effective multiplier: %.2f\n", 
                           control_state.Edes / (control_params.m * 9.81f * (control_params.L * 0.5f)));
                    printf("====================================\n\n");
                    
                    // ADAPTIVE SYNC: Scale current physics energy to maintain consistent ratio
                    // This prevents energy mismatch when mass changes
                    float target_energy_for_current_state = control_state.Edes * current_energy_ratio;
                    
                    // Get current pendulum state (preserve angle and velocity)
                    float current_theta = physics.getTrueTheta();
                    float current_omega = physics.getTrueOmega();
                    
                    // Calculate what omega should be to achieve target energy at current angle
                    // E = mgh + (1/2)Iω² where h = L(1-cos(θ)) and I = mL²
                    float potential_energy = control_params.m * 9.81f * control_params.L * (1.0f - cosf(current_theta));
                    float kinetic_energy = target_energy_for_current_state - potential_energy;
                    float moment_of_inertia = control_params.m * control_params.L * control_params.L;
                    
                    // Only adjust if kinetic energy is positive (pendulum can have this energy state)
                    if (kinetic_energy > 0.0f) {
                        float adjusted_omega = sqrtf(2.0f * kinetic_energy / moment_of_inertia);
                        // Preserve direction of rotation
                        if (current_omega < 0.0f) adjusted_omega = -adjusted_omega;
                        
                        // Reset physics with energy-consistent state
                        physics.reset(current_theta, adjusted_omega);
                        
                        printf("ADAPTIVE ENERGY SYNC: mass=%.3f→%.3f kg, energy_ratio=%.3f preserved\n",
                               control_params.m, control_params.m, current_energy_ratio);
                        printf("  energy: %.4f→%.4f J (target: %.4f→%.4f J)\n",
                               physics.getEnergy(), physics.getEnergy(), 
                               control_state.Edes, control_state.Edes);
                    } else {
                        // If energy state is invalid, reset to hanging position
                        physics.reset(0.0f, 0.0f);  // Reset to hanging down
                        printf("ADAPTIVE ENERGY SYNC: Invalid energy state, reset to hanging\n");
                    }
                    
                    // CRITICAL FIX: Auto-scale motor parameters based on LEVER ARM (m × L)
                    // Auto-scale motor torque and voltage conversion for lever arm changes
                    static bool auto_scale_motor = true;
                    ImGui::Checkbox("Auto-scale motor power for mass/length", &auto_scale_motor);
                    
                    if (auto_scale_motor) {
                        // Use lever arm scaling consistently with initialization
                        float default_lever_arm = 0.004f * 0.3048f;  // Default 4g at 12 inches
                        float current_lever_arm = control_params.m * control_params.L;
                        float lever_ratio = current_lever_arm / default_lever_arm;
                        float motor_scaling_factor = sqrtf(lever_ratio); // Square root scaling
                        
                        // Apply minimum thresholds for very light/short pendulums
                        float min_scaling_factor = 0.05f;  // Minimum 5% of default power
                        motor_scaling_factor = fmaxf(motor_scaling_factor, min_scaling_factor);
                        
                        // Scale maximum torque and voltage conversion based on lever arm
                        control_params.tau_max = 0.050f * motor_scaling_factor;
                        // SIGNIFICANTLY INCREASED for effective energy pumping from zero energy
                        control_params.u_to_tau = 0.050f * motor_scaling_factor;
                        control_params.k_energy = ENERGY_PUMP_GAIN_MULTIPLIER * sqrtf(lever_ratio); // From config.h
                        
                        printf("AUTO-SCALED MOTOR: tau_max=%.4f, u_to_tau=%.4f, k_energy=%.2f (lever_ratio=%.2f)\n",
                               control_params.tau_max, control_params.u_to_tau, control_params.k_energy, lever_ratio);
                    }
                    
                    // CRITICAL FIX: Scale control gains based on mass for stability
                    float ctrl_mass_ratio = control_params.m / 0.004f;  // Ratio to default mass (changed from 0.2f)
                    float length_ratio = control_params.L / 0.3048f;  // Ratio to default length
                    
                    // Scale PID gains to maintain stability
                    // Heavier/longer pendulums need different control response
                    float control_inertia_ratio = ctrl_mass_ratio * length_ratio * length_ratio;
                    
                    // Automatic gain scaling (can be overridden by manual tuning)
                    static bool auto_scale_gains = true;
                    ImGui::Checkbox("Auto-scale gains for mass/length", &auto_scale_gains);
                    
                    if (auto_scale_gains) {
                        control_params.Kp = 12.0f * sqrtf(control_inertia_ratio);  // Scale with sqrt of inertia (fixed base)
                        control_params.Kd = 2.0f * sqrtf(control_inertia_ratio);   // Scale damping proportionally
                        control_params.Ki = 0.5f / sqrtf(control_inertia_ratio);   // Reduce integral for heavy systems (fixed base)
                    }
                    
                    printf("Physics updated: m=%.3f kg, L=%.3f m, b=%.5f, Kp=%.2f, Kd=%.2f, Ki=%.2f\n", 
                           control_params.m, control_params.L, control_params.b_vis,
                           control_params.Kp, control_params.Kd, control_params.Ki);
                }
            }
        }
        
        // Enhanced test scenarios with proper coordinate handling
        if (ImGui::CollapsingHeader("Test Scenarios - Enhanced Physics")) {
            if (ImGui::Button("Small Disturbance (±0.1 rad)")) {
                float current_theta = physics.getTrueTheta(); // Get upright-referenced angle
                float new_theta = current_theta + 0.1f;
                float new_omega = 0.5f;
                
                // COMPREHENSIVE DEBUGGING FOR SMALL DISTURBANCE
                printf("\n=== SMALL DISTURBANCE BUTTON PRESSED ===\n");
                printf("BEFORE: current_theta = %.6f rad (%.3f deg)\n", current_theta, current_theta * 180.0f / M_PI);
                printf("        current_omega = %.6f rad/s\n", physics.getTrueOmega());
                printf("        angle from bottom = %.6f rad (%.3f deg)\n", 
                       fmod(current_theta + M_PI, 2*M_PI) - M_PI, 
                       (fmod(current_theta + M_PI, 2*M_PI) - M_PI) * 180.0f / M_PI);
                printf("        energy = %.6f J\n", physics.getEnergy());
                
                printf("SETTING: new_theta = %.6f rad (%.3f deg)\n", new_theta, new_theta * 180.0f / M_PI);
                printf("         new_omega = %.6f rad/s\n", new_omega);
                printf("         new angle from bottom = %.6f rad (%.3f deg)\n", 
                       fmod(new_theta + M_PI, 2*M_PI) - M_PI, 
                       (fmod(new_theta + M_PI, 2*M_PI) - M_PI) * 180.0f / M_PI);
                
                physics.reset(new_theta, new_omega);
                
                printf("AFTER:   actual_theta = %.6f rad (%.3f deg)\n", physics.getTrueTheta(), physics.getTrueTheta() * 180.0f / M_PI);
                printf("         actual_omega = %.6f rad/s\n", physics.getTrueOmega());
                printf("         actual angle from bottom = %.6f rad (%.3f deg)\n", 
                       fmod(physics.getTrueTheta() + M_PI, 2*M_PI) - M_PI, 
                       (fmod(physics.getTrueTheta() + M_PI, 2*M_PI) - M_PI) * 180.0f / M_PI);
                printf("         new energy = %.6f J (target: %.6f J)\n", 
                       physics.getEnergy(), control_state.Edes);
                printf("         control state: %s\n", 
                       (control_state.state == ST_BALANCE) ? "BALANCE" : 
                       (control_state.state == ST_SWINGUP) ? "SWINGUP" : "UNKNOWN");
                printf("=========================================\n\n");
                
                // Enable detailed debugging for next 10 control cycles
                disturbance_debug_cycles = 10;
            }
            ImGui::SameLine();
            if (ImGui::Button("Medium Disturbance (±0.3 rad)")) {
                float current_theta = physics.getTrueTheta();
                physics.reset(current_theta + 0.3f, 1.0f);
            }
            
            if (ImGui::Button("Start Hanging Down")) {
                physics.reset(0.0f, 0.0f);  // Hanging down (bottom-ref angle = 0)
                control_state.state = (decltype(control_state.state))ST_SWINGUP;
                ctrl_reset_integrator(&control_state);
            }
            ImGui::SameLine();
            if (ImGui::Button("Start Near Upright")) {
                // Reset to near upright: bottom-ref angle = π - small offset
                physics.reset(M_PI - 0.1f, 0.0f);  // Near upright (bottom-ref angle)
                control_state.state = (decltype(control_state.state))ST_BALANCE;
                ctrl_reset_integrator(&control_state);
            }
            
            if (ImGui::Button("Temperature Shock (+30°C)")) {
                physics.setTemperature(55.0f);
            }
            ImGui::SameLine();
            if (ImGui::Button("Parameter Test (+10% gains)")) {
                control_params.Kp *= 1.1f;
                control_params.Kd *= 1.1f;
                control_params.Ki *= 1.1f;
            }
            
            if (ImGui::Button("Reset Parameters")) {
                initControlSystem(); // Reset to defaults
            }
            ImGui::SameLine();
            if (ImGui::Button("Light Pendulum (0.1kg)")) {
                // ADAPTIVE FIX: Preserve energy ratio when preset changes mass/length
                float current_energy_ratio = physics.getEnergy() / control_state.Edes;
                float current_theta = physics.getTrueTheta();
                float current_omega = physics.getTrueOmega();
                
                control_params.m = 0.1f;
                control_params.L = 0.3048f;
                control_params.Kp = 10.0f;
                control_params.Kd = 1.5f;
                control_params.Ki = 0.7f;
                physics.setParameters(control_params.m, control_params.L, 
                                    control_params.b_vis, control_params.tau_ff);
                control_state.Edes = CALCULATE_ENERGY_TARGET(control_params.m, control_params.L); // From config.h
                
                // Sync energy state to prevent mismatch
                float target_energy = control_state.Edes * current_energy_ratio;
                float potential_energy = control_params.m * 9.81f * control_params.L * (1.0f - cosf(current_theta));
                float kinetic_energy = target_energy - potential_energy;
                if (kinetic_energy > 0.0f) {
                    float moment_of_inertia = control_params.m * control_params.L * control_params.L;
                    float adjusted_omega = sqrtf(2.0f * kinetic_energy / moment_of_inertia);
                    if (current_omega < 0.0f) adjusted_omega = -adjusted_omega;
                    physics.reset(current_theta, adjusted_omega);
                }
            }
            
            if (ImGui::Button("Heavy Pendulum (0.5kg)")) {
                // ADAPTIVE FIX: Preserve energy ratio when preset changes mass/length
                float current_energy_ratio = physics.getEnergy() / control_state.Edes;
                float current_theta = physics.getTrueTheta();
                float current_omega = physics.getTrueOmega();
                
                control_params.m = 0.5f;
                control_params.L = 0.3048f;
                control_params.Kp = 25.0f;
                control_params.Kd = 3.5f;
                control_params.Ki = 0.5f;
                physics.setParameters(control_params.m, control_params.L, 
                                    control_params.b_vis, control_params.tau_ff);
                control_state.Edes = CALCULATE_ENERGY_TARGET(control_params.m, control_params.L); // From config.h
                
                // Sync energy state to prevent mismatch
                float target_energy = control_state.Edes * current_energy_ratio;
                float potential_energy = control_params.m * 9.81f * control_params.L * (1.0f - cosf(current_theta));
                float kinetic_energy = target_energy - potential_energy;
                if (kinetic_energy > 0.0f) {
                    float moment_of_inertia = control_params.m * control_params.L * control_params.L;
                    float adjusted_omega = sqrtf(2.0f * kinetic_energy / moment_of_inertia);
                    if (current_omega < 0.0f) adjusted_omega = -adjusted_omega;
                    physics.reset(current_theta, adjusted_omega);
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Long Pendulum (0.6m)")) {
                // ADAPTIVE FIX: Preserve energy ratio when preset changes mass/length
                float current_energy_ratio = physics.getEnergy() / control_state.Edes;
                float current_theta = physics.getTrueTheta();
                float current_omega = physics.getTrueOmega();
                
                control_params.m = 0.2f;
                control_params.L = 0.6f;
                control_params.Kp = 30.0f;
                control_params.Kd = 4.0f;
                control_params.Ki = 0.3f;
                physics.setParameters(control_params.m, control_params.L, 
                                    control_params.b_vis, control_params.tau_ff);
                control_state.Edes = CALCULATE_ENERGY_TARGET(control_params.m, control_params.L); // From config.h
                
                // Sync energy state to prevent mismatch
                float target_energy = control_state.Edes * current_energy_ratio;
                float potential_energy = control_params.m * 9.81f * control_params.L * (1.0f - cosf(current_theta));
                float kinetic_energy = target_energy - potential_energy;
                if (kinetic_energy > 0.0f) {
                    float moment_of_inertia = control_params.m * control_params.L * control_params.L;
                    float adjusted_omega = sqrtf(2.0f * kinetic_energy / moment_of_inertia);
                    if (current_omega < 0.0f) adjusted_omega = -adjusted_omega;
                    physics.reset(current_theta, adjusted_omega);
                }
            }
        }
        
        ImGui::End();
    }
    
    void renderVisualization() {
        // Simple pendulum visualization - FIX: Use correct coordinate system
        ImGui::Begin("Pendulum Visualization");
        
        ImDrawList* draw_list = ImGui::GetWindowDrawList();
        ImVec2 canvas_pos = ImGui::GetCursorScreenPos();
        ImVec2 canvas_size = ImGui::GetContentRegionAvail();
        
        if (canvas_size.x < 50.0f) canvas_size.x = 50.0f;
        if (canvas_size.y < 50.0f) canvas_size.y = 50.0f;
        
        ImVec2 center(canvas_pos.x + canvas_size.x * 0.5f, canvas_pos.y + canvas_size.y * 0.3f);
        float scale = std::min(canvas_size.x, canvas_size.y) * 0.3f;
        
        // Draw pivot
        draw_list->AddCircleFilled(center, 5, IM_COL32(100, 100, 100, 255));
        
        // Draw pendulum using enhanced physics coordinate system
        // getTrueTheta() returns upright-referenced angle (0 = upright, PI = hanging down)
        float theta_vis = physics.getTrueTheta();  // Upright-referenced angle
        ImVec2 bob_pos(center.x + scale * sin(theta_vis), center.y + scale * cos(theta_vis));
        
        // Rod
        draw_list->AddLine(center, bob_pos, IM_COL32(200, 200, 50, 255), 3.0f);
        
        // Bob - color based on control state
        ImU32 bob_color = (control_state.state == (decltype(control_state.state))ST_BALANCE) ? 
                         IM_COL32(50, 255, 50, 255) :  // Green for balance
                         (control_state.state == (decltype(control_state.state))ST_CATCH) ?
                         IM_COL32(255, 255, 50, 255) : // Yellow for catch
                         (control_state.state == (decltype(control_state.state))ST_SWINGUP) ?
                         IM_COL32(255, 150, 50, 255) : // Orange for swing-up
                         IM_COL32(255, 50, 50, 255);   // Red for idle/other
        draw_list->AddCircleFilled(bob_pos, 8, bob_color);
        
        // Reference lines
        draw_list->AddLine(ImVec2(center.x, center.y - scale), 
                          ImVec2(center.x, center.y + scale), 
                          IM_COL32(100, 100, 100, 100), 1.0f);
        
        ImGui::End();
    }
    
    // FIX: Keep all your original data logging methods - they're fine
    void renderDataLogging() {
        ImGui::Begin("Enhanced Data Logging & Analysis");
        
        ImGui::Checkbox("Enable Logging", &logging_enabled);
        ImGui::SameLine();
        if (ImGui::Button("Clear Log")) data_log.clear();
        ImGui::SameLine();
        if (ImGui::Button("Save Enhanced CSV")) saveEnhancedDataCSV();
        
        ImGui::Text("Data Points: %zu", data_log.size());
        
        // Your original performance metrics code is fine - keep it
        if (!data_log.empty() && data_log.size() > 100) {
            ImGui::Separator();
            ImGui::Text("=== INDUSTRIAL PERFORMANCE METRICS ===");
            
            float final_angle_error = abs(std::get<1>(data_log.back()));
            float max_overshoot = 0.0f;
            
            for (size_t i = 100; i < data_log.size(); i++) {
                float angle = abs(std::get<1>(data_log[i]));
                max_overshoot = std::max(max_overshoot, angle);
            }
            
            ImGui::Text("Final Angle Error: %.2f° %s", final_angle_error,
                       final_angle_error < 5.0f ? "(✓ EXCELLENT)" : 
                       final_angle_error < 15.0f ? "(○ GOOD)" : "(✗ POOR)");
                       
            ImGui::Text("Maximum Overshoot: %.2f°", max_overshoot);
            
            // Overall system grade
            if (final_angle_error < 5.0f && control_state.state == (decltype(control_state.state))ST_BALANCE) {
                ImGui::Text("Overall Grade: A - INDUSTRIAL GRADE");
            } else if (final_angle_error < 15.0f) {
                ImGui::Text("Overall Grade: B - GOOD PERFORMANCE");
            } else {
                ImGui::Text("Overall Grade: C - NEEDS IMPROVEMENT");
            }
        }
        
        // Enhanced plotting with multiple traces
        if (!data_log.empty() && data_log.size() > 1) {
            std::vector<float> angles, energies, commands;
            for (const auto& dp : data_log) {
                angles.push_back(std::get<1>(dp));  // angle in degrees
                energies.push_back(std::get<4>(dp)); // energy
                commands.push_back(std::get<3>(dp)); // motor command
            }
            
            ImGui::Separator();
            ImGui::Text("=== ENHANCED VISUALIZATION ===");
            
            ImGui::PlotLines("Angle from Upright (°)", angles.data(), angles.size(), 
                           0, "Target: ±5° steady-state", -180.0f, 180.0f, ImVec2(500, 120));
                           
            ImGui::PlotLines("Energy (J)", energies.data(), energies.size(), 
                           0, "Target energy tracking", 0.0f, FLT_MAX, ImVec2(500, 120));
                           
            ImGui::PlotLines("Motor Command", commands.data(), commands.size(), 
                           0, "Saturation limits: ±1.0", -1.2f, 1.2f, ImVec2(500, 120));
        }
        
        ImGui::End();
    }
    
    // FIX: Keep your original save methods - they work fine
    void saveEnhancedDataCSV() {
        std::ofstream file("enhanced_pendulum_data.csv");
        file << "time,angle_deg,omega_rad_s,motor_cmd,energy_J,state,validation_error,performance_grade\n";
        
        for (size_t i = 0; i < data_log.size(); i++) {
            const auto& dp = data_log[i];
            float validation_error = physics.compareToSimpleModel(std::get<1>(dp) * M_PI/180, std::get<2>(dp));
            
            float angle_error = abs(std::get<1>(dp));
            char grade = angle_error < 5.0f ? 'A' : angle_error < 15.0f ? 'B' : 
                        angle_error < 30.0f ? 'C' : 'F';
            
            file << std::get<0>(dp) << "," << std::get<1>(dp) << "," << std::get<2>(dp) << "," 
                 << std::get<3>(dp) << "," << std::get<4>(dp) << "," << std::get<5>(dp) << ","
                 << validation_error << "," << grade << "\n";
        }
        file.close();
        printf("Enhanced data saved to enhanced_pendulum_data.csv with validation metrics\n");
    }
    
    void reset() {
        // Enhanced physics: reset to hanging down position
        physics.reset(0.0f, 0.0f);  // Start hanging down (bottom‑ref angle = 0)
        ctrl_init(&control_params, &control_state);
        control_state.Edes = CALCULATE_ENERGY_TARGET(control_params.m, control_params.L); // From config.h  // Increased energy target
        
        // Explicitly start in swing-up mode and ensure it stays there
        control_state.state = (decltype(control_state.state))ST_SWINGUP;
        ctrl_reset_integrator(&control_state);
        
        sim_time = 0;
        data_log.clear();
        
        // Reinitialise the virtual encoder to correspond to the current physical angle.  At
        // reset the pendulum is hanging down, so the upright‑referenced angle is −π (−180°).
        // Use ve_init to set the internal unwrapped angle directly.
        ve_init(&encoder, wrap_pi(physics.getTrueTheta() - (float)M_PI));

        printf("System reset: pendulum hanging down, starting swing-up control immediately\n");
    }
    
    void handleEvents() {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == SDL_KEYDOWN) {
                switch (event.key.keysym.sym) {
                    case SDLK_SPACE: 
                        paused = !paused; 
                        printf("Simulation %s\n", paused ? "PAUSED" : "RESUMED");
                        break;
                    case SDLK_r: 
                        reset(); 
                        break;
                    case SDLK_1: 
                        control_state.state = (decltype(control_state.state))ST_IDLE; 
                        printf("Control state: IDLE\n");
                        break;
                    case SDLK_2: 
                        control_state.state = (decltype(control_state.state))ST_SWINGUP; 
                        ctrl_reset_integrator(&control_state);
                        printf("Control state: SWINGUP\n");
                        break;
                    case SDLK_3:
                        control_state.state = (decltype(control_state.state))ST_BALANCE;
                        ctrl_reset_integrator(&control_state);
                        printf("Control state: BALANCE\n");
                        break;
                    case SDLK_F11: {
                        // Toggle fullscreen
                        if (is_fullscreen) {
                            SDL_SetWindowFullscreen(window, 0);
                            SDL_SetWindowSize(window, 1200, 800);
                            SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
                            printf("Switched to windowed mode\n");
                        } else {
                            SDL_SetWindowFullscreen(window, SDL_WINDOW_FULLSCREEN_DESKTOP);
                            printf("Switched to fullscreen mode\n");
                        }
                        is_fullscreen = !is_fullscreen;
                        break;
                    }
                    case SDLK_f:
                        // Toggle FPS display
                        show_fps_display = !show_fps_display;
                        if (!show_fps_display) {
                            SDL_SetWindowTitle(window, "Pendulum Control System - Enhanced Physics");
                        }
                        printf("FPS display %s\n", show_fps_display ? "enabled" : "disabled");
                        break;
                }
            }
        }
    }
    
    void run() {
        printf("=== Pendulum Control System - Enhanced Physics ===\n");
        printf("Advanced pendulum simulation with industrial-grade physics\n");
        printf("Coordinate system: 0=upright, PI=hanging down\n");
        printf("Controls: Space=Pause/Resume, R=Reset, 1=Idle, 2=Swingup, 3=Balance\n");
        printf("F11=Toggle Fullscreen, F=Show FPS\n\n");
        
        // Performance optimization: Target 60 FPS rendering
        const float target_frame_time = 1.0f / 60.0f; // 60 FPS target
        
        // Performance tracking
        int frame_count = 0;
        float fps = 0.0f;
        auto fps_timer = std::chrono::steady_clock::now();
        
        while (running) {
            auto frame_start = std::chrono::steady_clock::now();
            
            handleEvents();
            
            // Only update simulation logic at reasonable rate
            static auto last_update = std::chrono::steady_clock::now();
            auto now = std::chrono::steady_clock::now();
            float update_dt = std::chrono::duration<float>(now - last_update).count();
            
            // Limit update rate to reduce CPU load
            if (update_dt >= 1.0f / 120.0f) { // Max 120Hz update rate
                update();
                last_update = now;
            }
            
            // Always render but limit frame rate
            render();
            
            // FPS calculation
            frame_count++;
            auto fps_elapsed = std::chrono::duration<float>(now - fps_timer).count();
            if (fps_elapsed >= 1.0f) {
                fps = frame_count / fps_elapsed;
                frame_count = 0;
                fps_timer = now;
            }
            
            // Show FPS if enabled
            if (show_fps_display) {
                char title[256];
                snprintf(title, sizeof(title), "Pendulum Control System - Enhanced Physics [%.1f FPS]", fps);
                SDL_SetWindowTitle(window, title);
            }
            
            // Frame rate limiting to prevent excessive GPU load
            auto frame_end = std::chrono::steady_clock::now();
            float frame_time = std::chrono::duration<float>(frame_end - frame_start).count();
            
            if (frame_time < target_frame_time) {
                float sleep_time = target_frame_time - frame_time;
                std::this_thread::sleep_for(std::chrono::duration<float>(sleep_time));
            }
        }
    }
    
    // Keep your destructor - it's fine
    ~PCDebugSimulator() {
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplSDL2_Shutdown();
        ImGui::DestroyContext();
        
        if (gl_context) SDL_GL_DeleteContext(gl_context);
        if (window) SDL_DestroyWindow(window);
        SDL_Quit();
    }
};

// FIX: Main function with better error reporting
int main(int argc, char* argv[]) {
    (void)argc; (void)argv;
    printf("Enhanced Pendulum Control System - PC Debug Mode\n");
    printf("This version fixes the bugs in the original enhanced physics simulation\n");
    printf("DEBUG: MOTOR_THERMAL_CAPACITY = %.1fJ, MOTOR_EMERGENCY_CAPACITY = %.1fJ\n", 
           MOTOR_THERMAL_CAPACITY, MOTOR_EMERGENCY_CAPACITY);
    printf("DEBUG: Testing hardcoded thermal values in motor_protection.c\n");
    printf("\n");
    
    try {
        PCDebugSimulator simulator;
        simulator.run();
    } catch (const std::exception& e) {
        printf("Error: %s\n", e.what());
        printf("Make sure you have:\n");
        printf("1. SDL2 development libraries installed\n");
        printf("2. OpenGL development libraries installed\n");
        printf("3. ImGui and GL3W in external/ directory\n");
        return 1;
    }
    
    return 0;
}

#else

// ============================================================================
// RASPBERRY PI PICO IMPLEMENTATION - ACTUAL WORKING CODE
// ============================================================================

#define LED_PIN 25
#define I2C_SDA 2
#define I2C_SCL 3
#define PWM_IN1 14
#define PWM_IN2 15

// Optional: set to true to invert motor polarity if needed
bool MOTOR_INVERT = false;

// Optional: set to true to invert encoder direction if needed
bool SENSOR_INVERT = false;

// Global variables needed by debug module
volatile bool enc_found = false;
bool thermal_protection_active = false;

// Add missing function declarations and constants for Pico
static inline float wrap_pi(float x) {
    while (x > M_PI) x -= 2*M_PI;
    while (x <= -M_PI) x += 2*M_PI;
    return x;
}

// Define state constants to match the enum in control.h
#define ST_IDLE 0
#define ST_CALIB 1
#define ST_SWINGUP 2
#define ST_CATCH 3
#define ST_BALANCE 4
#define ST_FAULT 5

// Globals
as5600_t enc;              
drv8833_t drv;             
ctrl_params_t P;           
ctrl_state_t  S;           
static KalmanFilter KF;
lpf1_t lp_angle;           
diff_lpf_t dtheta;         

volatile bool control_enabled = false;
static bool in_cb = false;
// enc_found is now declared as global variable above

// Note: unwrap_angle is already defined in as5600.h, so we don't redefine it here

// Control loop timer callback - ACTUAL WORKING IMPLEMENTATION
bool repeating_timer_cb(struct repeating_timer *t) {
    (void)t;
    
    // Prevent re-entry
    if (in_cb || !control_enabled) {
        return true;
    }
    in_cb = true;
    
    // Record start time for accurate timing
    absolute_time_t start = get_absolute_time();
    
    // 1) Read encoder
    float theta_raw;
    bool encoder_ok = false;
    
    if (enc_found) {
        encoder_ok = as5600_read_angle_rad(&enc, &theta_raw);
        
        if (!encoder_ok) {
            static uint32_t error_count = 0;
            error_count++;
            // Only print error message every 1000 failures if debug is enabled
            if (error_count % 1000 == 0 && debug_is_output_enabled()) {
                printf("Encoder read failed! (count: %lu)\n", error_count);
            }
            if (error_count > 5000) {
                enc_found = false;  // Mark encoder as failed after many attempts
            }
        }
    }
    
    if (!encoder_ok) {
        // Use fallback or safe mode
        in_cb = false;
        return true;
    }
    
    // Unwrap and compute bottom/upright frames
    static float angle_prev = 0.0f;
    float angle_unwrapped = unwrap_angle(angle_prev, theta_raw);
    angle_prev = angle_unwrapped;

    // Apply sensor inversion if needed (to fix encoder direction mismatch)
    if (SENSOR_INVERT) {
        angle_unwrapped = -angle_unwrapped;
    }

    // Calculate theta_b as angle from bottom: theta_b=0 when hanging down, theta_b=π when upright
    float theta_b = wrap_pi(angle_unwrapped - enc.angle_offset_rad);
    float theta_u = wrap_pi(theta_b - (float)M_PI); // upright = 0

    // 2) Estimate omega with dynamic dt calculation
    static absolute_time_t last_time;
    static bool first_iteration = true;
    
    float dt;
    if (first_iteration) {
        dt = 1.0f / CONTROL_HZ;  // Use nominal dt for first iteration
        last_time = start;
        first_iteration = false;
    } else {
        // Calculate actual elapsed time for more accurate derivative
        int64_t elapsed_us = absolute_time_diff_us(last_time, start);
        dt = elapsed_us / 1000000.0f;  // Convert to seconds
        
        // Sanity check: clamp dt to reasonable bounds
        if (dt < 0.0005f || dt > 0.005f) {  // 0.5ms to 5ms
            dt = 1.0f / CONTROL_HZ;  // Fall back to nominal if too far off
        }
        last_time = start;
    }
    
    float theta_meas = lpf1_step(&lp_angle, theta_u);
    float omega_meas = diff_lpf_step(&dtheta, theta_meas, dt);

    // Kalman predict/update
    kalman_predict(&KF, S.u);
    kalman_update(&KF, theta_meas);

    S.theta_b = theta_b;
    S.theta_u = kalman_get_theta(&KF);
    S.omega   = kalman_get_omega(&KF);

    // 3) Update control state machine
    float u = ctrl_step(&P, &S);
    
    // Check if debug system has motor control override
    float debug_motor_value = 0.0f;
    if (debug_has_motor_control(&debug_motor_value)) {
        u = debug_motor_value;
        
        // If emergency stop, ensure control loop knows to go to IDLE
        debug_motor_state_t* motor_state = debug_get_motor_state();
        if (motor_state->emergency_stop) {
            S.state = (decltype(S.state))ST_IDLE;
            S.u = 0.0f;
        }
    }
    
    // Update the state with the final command for Kalman prediction
    S.u = u;
    
    // Command the motor
    drv8833_cmd(&drv, u);
    
    in_cb = false;
    return true;
}

// HELPER FUNCTION: Update motor scaling for different pendulum configurations
// Call this function after changing P.m or P.L to automatically scale motor parameters
void update_pico_motor_scaling(ctrl_params_t *params) {
    // ULTRA-ULTRA-conservative motor scaling for very light masses
    float mass_ratio = params->m / 0.004f; // Ratio to default 4g mass (fixed from 0.1f)
    float scaled_ratio = sqrtf(mass_ratio); // Square root for gentler scaling
    
    // Apply very low minimum thresholds for tiny masses (much lower than before)
    float min_tau_factor = 0.05f;  // Minimum 5% of default power (was 15%)
    float min_u_to_tau_factor = 0.08f; // Minimum 8% of default voltage scaling (was 20%)
    
    float tau_factor = fmaxf(scaled_ratio, min_tau_factor);
    float u_to_tau_factor = fmaxf(scaled_ratio, min_u_to_tau_factor);
    
    // Scale maximum torque extremely conservatively with lower base values
    params->tau_max = 0.02f * tau_factor; // Much lower base: 0.02 instead of 0.05
    
    // Scale voltage-to-torque conversion extremely conservatively  
    params->u_to_tau = 0.005f * u_to_tau_factor; // Much lower base: 0.005 instead of 0.01
    
    // Scale energy gain based on mass (moderate scaling)
    params->k_energy = 1.5f * sqrtf(mass_ratio);
    
    if (debug_is_output_enabled()) {
        printf("MOTOR SCALING UPDATED: m=%.3f kg, L=%.3f m\n", params->m, params->L);
        printf("  -> tau_max=%.4f Nm, u_to_tau=%.4f, k_energy=%.2f\n", 
               params->tau_max, params->u_to_tau, params->k_energy);
        printf("  -> mass_ratio=%.2f, scaled_ratio=%.2f (ultra-ultra-conservative scaling)\n", mass_ratio, scaled_ratio);
    }
}

// Main function for Pico - ACTUAL WORKING IMPLEMENTATION
int main() {
    stdio_init_all();
    
    // Enable watchdog with a long timeout during initialization
    watchdog_enable(30000, 1);  // 30 second timeout during init
    
    sleep_ms(2000);  // Allow time for USB serial to initialize
    
    // Initialize LED
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 1);  // Turn on LED
    
    // Initialize I2C
    i2c_init(i2c1, 400 * 1000);  // 400 kHz
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);
    
    // Initialize DRV8833
    drv8833_init(&drv, PWM_IN1, PWM_IN2, 25000.0f);  // 25 kHz PWM - optimal for brushed motors
    
    // Initialize debug system first
    debug_init();
    
    if (debug_is_output_enabled()) {
        printf("Pico Pendulum Control System - Enhanced Braking\n");
        printf("Initializing...\n");
    }
    
    // Try to find AS5600 encoder
    enc_found = as5600_init(&enc, i2c1, I2C_SDA, I2C_SCL, 400000);
    
    if (enc_found) {
        if (debug_is_output_enabled()) {
            printf("AS5600 encoder found\n");
        }
    } else {
        if (debug_is_output_enabled()) {
            printf("AS5600 encoder not found - continuing anyway\n");
        }
    }
    
    // Initialize control system with improved braking parameters
    ctrl_init(&P, &S);
    
    // Enhanced physical parameters
    // *** TO CHANGE PENDULUM CONFIGURATION: ***
    // 1. Modify P.m (mass in kg) and P.L (length in meters) below
    // 2. The motor scaling will be automatically updated by update_pico_motor_scaling()
    // 3. Example configurations:
    //    - Light rod: P.m = 0.05f, P.L = 0.08f (50g, 8cm)
    //    - Standard: P.m = 0.1f,  P.L = 0.1f  (100g, 10cm) 
    //    - Heavy rod: P.m = 0.2f, P.L = 0.12f (200g, 12cm)
    P.m = 0.1f;         // 100g rod (default for Pico) *** CHANGE THIS FOR YOUR PENDULUM ***
    P.L = 0.1f;         // 10cm rod (default for Pico)  *** CHANGE THIS FOR YOUR PENDULUM ***
    P.Jm = 1e-5f;       // Small motor inertia
    P.b_vis = 0.0002f;  // Light damping
    P.tau_ff = 0.001f;  // Friction
    P.dt = 1.0f / CONTROL_HZ;
    
    // CRITICAL FIX: Scale motor parameters based on pendulum mass and length (same as PC)
    // Calculate pendulum moment of inertia: I = m * L^2 (point mass approximation)
    float pendulum_inertia = P.m * P.L * P.L;
    
    // ULTRA-ULTRA-conservative scaling for very light masses
    float mass_ratio = P.m / 0.004f; // Ratio to default 4g (corrected from 0.1f)
    float scaled_ratio = sqrtf(mass_ratio); // Square root for gentler scaling
    
    // Apply very low minimum thresholds for tiny masses (much lower than before)
    float min_tau_factor = 0.05f;  // Minimum 5% of default power (was 15%)
    float min_u_to_tau_factor = 0.08f; // Minimum 8% of default voltage scaling (was 20%)
    
    float tau_factor = fmaxf(scaled_ratio, min_tau_factor);
    float u_to_tau_factor = fmaxf(scaled_ratio, min_u_to_tau_factor);
    
    // Scale maximum torque extremely conservatively with lower base values
    P.tau_max = 0.02f * tau_factor; // Much lower base: 0.02 instead of 0.05
    
    // Scale voltage-to-torque conversion extremely conservatively  
    P.u_to_tau = 0.005f * u_to_tau_factor; // Much lower base: 0.005 instead of 0.01
    
    // Scale energy gain based on mass - heavier pendulums need different energy pumping
    P.k_energy = ENERGY_PUMP_GAIN_MULTIPLIER * sqrtf(mass_ratio); // From config.h
    P.swing_sat = SWING_SATURATION; // From config.h
    
    // Enhanced PID gains for better stability with improved braking
    P.Kp = 12.0f;       // Adequate proportional gain
    P.Kd = 2.0f;        // Good derivative gain for damping
    P.Ki = 0.5f;        // Moderate integral gain
    P.ui_max = 0.2f;    // Reasonable integral windup limit
    P.balance_sat = 0.5f; // PC uses slightly higher balance authority than Pico
    
    // Enhanced catch thresholds
    // Use a tighter catch threshold for the Pico build as well.  A 20° catch zone
    // matches the PC simulator configuration and prevents premature balancing.
    P.theta_catch = THETA_CATCH_RADIANS; // From config.h
    P.omega_catch = OMEGA_CATCH_PC;      // PC-specific value from config.h
    
    // Braking parameters - tunable emergency braking strength
    // Braking parameters from config.h
    P.brake_light = BRAKE_LIGHT;       // Light braking for swing-up transitions
    P.brake_moderate = BRAKE_MODERATE; // Moderate braking for high speeds
    P.brake_strong = BRAKE_STRONG;     // Strong braking for very high speeds
    P.brake_maximum = BRAKE_MAXIMUM;   // Maximum braking for extreme speeds
    
    // Set target energy for swing-up
    S.Edes = P.m * GRAVITY_ACCEL * (P.L * 0.5f) * 2.0f;
    
    // CRITICAL: Apply motor scaling after setting mass and length
    update_pico_motor_scaling(&P);
    
    // Initialize filters
    lpf1_init(&lp_angle, 0.95f, 0.0f);  // Light filtering
    diff_lpf_init(&dtheta, 0.8f); // Moderate filtering on derivative
    
    // Initialize Kalman filter  
    float a = 1.0f - P.b_vis * P.dt / (CALCULATE_MOMENT_OF_INERTIA(P.m, P.L, P.Jm));
    float b = P.dt / (CALCULATE_MOMENT_OF_INERTIA(P.m, P.L, P.Jm));
    kalman_init(&KF, P.dt, a, b, 1e-4f, 5e-3f, 3e-3f);
    
    if (debug_is_output_enabled()) {
        printf("Control system initialized with enhanced braking\n");
    }
    
    // Calibration
    if (enc_found) {
        if (debug_is_output_enabled()) {
            printf("Starting calibration...\n");
        }
        
        // Robust calibration with circular averaging
        const int cal_steps = 20;
        float sum_sin = 0.0f, sum_cos = 0.0f;
        int cal_count = 0;
        
        for (int i = 0; i < cal_steps; i++) {
            float raw_angle;
            if (as5600_read_angle_rad(&enc, &raw_angle)) {
                // Apply sensor inversion during calibration too
                if (SENSOR_INVERT) {
                    raw_angle = -raw_angle;
                }
                
                sum_sin += sinf(raw_angle);
                sum_cos += cosf(raw_angle);
                cal_count++;
                
                if (debug_is_output_enabled() && (i % 5 == 0)) {
                    printf("Calibration step %d/%d: angle=%.3f\n", i+1, cal_steps, raw_angle);
                }
            }
            sleep_ms(50);
        }
        
        if (cal_count > 3) {
            enc.angle_offset_rad = atan2f(sum_sin, sum_cos);
            
            if (debug_is_output_enabled()) {
                printf("Calibration OK: offset = %.3f rad\n", enc.angle_offset_rad);
            }
            debug_set_state(DBG_FLAG_CALIB_DONE);
            S.state = (decltype(S.state))ST_IDLE;
        } else {
            if (debug_is_output_enabled()) {
                printf("Calibration failed\n");
            }
            enc.angle_offset_rad = 0.0f;
            S.state = (decltype(S.state))ST_IDLE;
        }
    } else {
        S.state = (decltype(S.state))ST_IDLE;
    }
    
    // Main control loop
    if (debug_is_output_enabled()) {
        printf("Starting main loop...\n");
    }
    
    struct repeating_timer timer;
    bool timer_running = add_repeating_timer_ms(-1, repeating_timer_cb, NULL, &timer);
    
    if (!timer_running) {
        if (debug_is_output_enabled()) {
            printf("ERROR: Failed to create timer!\n");
        }
    }
    
    sleep_ms(500);
    control_enabled = true;
    debug_set_state(DBG_FLAG_RUNNING);
    
    // Reduce watchdog timeout now that initialization is complete
    watchdog_enable(8000, 1);  // 8 second timeout during normal operation
    
    if (debug_is_output_enabled()) {
        printf("Control enabled with enhanced braking system\n");
        printf("Press 'h' for help.\n");
    }
    
    // Main loop
    uint32_t loop_count = 0;
    uint32_t last_heartbeat = 0;
    
    while (1) {
        watchdog_update();
        debug_process();
        
        // Heartbeat every 60 seconds
        if (loop_count % 6000 == 0 && loop_count != last_heartbeat && debug_is_output_enabled()) {
            printf("Loop %lu, enc=%d, ctrl=%d\n", 
                   loop_count, enc_found, control_enabled);
            last_heartbeat = loop_count;
        }
        
        loop_count++;
        sleep_ms(10);
    }
    
    return 0;
}

#endif // PLATFORM_PC
