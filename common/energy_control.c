/*
 * Energy Control Implementation
 *
 * This file encapsulates the swing‑up energy pumping logic used by
 * both the embedded controller and the PC simulator.  It computes
 * a motor command based on the current pendulum energy relative to
 * the desired upright energy.  A breakaway routine kicks the
 * pendulum out of the bottom rest, and a drive level is adjusted
 * based on peak tracking.  A unified virtual encoder is used for
 * spin‑out detection.  The function updates fields in the
 * ctrl_state_t structure to maintain state across invocations.
 */

#include "common/energy_control.h"
#include "embedded/config.h"
#include "unified_virtual_encoder.h"
#include "common/control_utils.h"
#include <math.h>
#include <stdio.h>

/* Implementation of the energy control strategy.  This function
 * mirrors the logic found in the original embedded controller but
 * factors it out into a common module.  See control.c for
 * documentation on the algorithm. */
float energy_control(const ctrl_params_t *p, ctrl_state_t *s) {
    float u = 0.0f;
    
    // Special case: detect if pendulum is stuck near the bottom point
    // This happens when theta_u is close to ±π radians
    if (fabsf(fabsf(s->theta_u) - M_PI) < 0.8f && fabsf(s->omega) < 0.5f) {
        // We're near the bottom with low velocity - likely stuck
        
        // Count how long we've been stuck
        static int stuck_counter = 0;
        stuck_counter++;
        
        // After a short delay to confirm we're stuck
        if (stuck_counter > 20) {
            // Apply a strong push in a consistent direction to escape
            // Choose direction based on which side of bottom we're on
            float escape_direction = (s->theta_u > 0) ? 1.0f : -1.0f;
            
            // Reset counter if we've moved significantly
            if (fabsf(s->omega) > 2.0f) {
                stuck_counter = 0;
                printf("ESCAPED local minimum, resuming normal control\n");
            } else {
                // Print diagnostic once when we detect stuck condition
                if (stuck_counter == 21) {
                    printf("STUCK at bottom: theta_u=%f, omega=%f, applying escape push\n", 
                           s->theta_u, s->omega);
                }
                
                // Return strong push in escape direction
                return 0.8f * escape_direction;
            }
        }
    } else {
        // We're not stuck at the bottom, reset counter
        static int stuck_counter = 0;
        stuck_counter = 0;
    }
    
    // Convert upright‑referenced angle to bottom‑referenced angle.
    float theta_b = ctl_wrap_pi(s->theta_u + (float)M_PI);

    // Safety: guard against NaN velocity.
    if (isnan(s->omega)) {
        s->omega = 0.0f;
    }

    // Compute total energy about the bottom position using the upright
    // angle.  Equivalent to bottom‑based expression via trig identity.
    float cos_theta_u = cosf(s->theta_u);
    float J = CALCULATE_MOMENT_OF_INERTIA(p->m, p->L, p->Jm);
    s->E = 0.5f * J * s->omega * s->omega +
           p->m * GRAVITY_ACCEL * (p->L * 0.5f) * (1.0f + cos_theta_u);
    float e = s->E - s->Edes;

    // Rest detection thresholds.
    const float STATIONARY_ANGLE = STATIONARY_ANGLE_THRESHOLD;
    const float STATIONARY_SPEED = STATIONARY_SPEED_THRESHOLD;
    const float MOVE_THRESHOLD = MOVEMENT_THRESHOLD;

    bool at_rest = (fabsf(theta_b) < STATIONARY_ANGLE) && (fabsf(s->omega) < STATIONARY_SPEED);

    float u_energy = 0.0f;

    if (e < 0.0f) {
        // Below target energy: need to pump energy into the system.
        if (s->kick_active) {
            // Continue the breakaway impulse until motion is detected.
            u_energy = BREAKAWAY_DUTY * (float)s->kick_direction;
            if (fabsf(theta_b) >= MOVE_THRESHOLD || fabsf(s->omega) >= STATIONARY_SPEED) {
                s->kick_active = false;
                s->drive_level = 0.4f;
            }
        } else if (at_rest) {
            // Pendulum stuck at bottom: start a kick.  Choose direction based on offset or alternate.
            if (fabsf(theta_b) > STATIONARY_ANGLE) {
                s->kick_direction = -(int)ctl_sgn(theta_b);
            } else {
                s->kick_direction = -s->kick_direction;
            }
            s->kick_active = true;
            u_energy = BREAKAWAY_DUTY * (float)s->kick_direction;
        } else {
            // Pendulum swinging: pump energy using correct pendulum dynamics.
            // Apply torque in direction that increases energy:
            // - When moving away from bottom: torque in direction of motion
            // - When moving toward bottom: torque opposite to motion
            // This is achieved by: direction = sgn(omega * sin(theta_u))
            float sin_theta_u = sinf(s->theta_u);
            float direction = ctl_sgn(s->omega * sin_theta_u);
            u_energy = s->drive_level * direction;
            // Peak tracking to adapt drive level.
            static float prev_peak = 0.0f;
            static float current_peak = 0.0f;
            static float last_omega_sign = 0.0f;
            current_peak = fmaxf(current_peak, fabsf(theta_b));
            float omega_sign = ctl_sgn(s->omega);
            if (omega_sign != last_omega_sign && last_omega_sign != 0.0f) {
                // End of half swing: compare peaks.  Increase drive if peak did not grow.
                if (current_peak < prev_peak + 0.01f) {
                    s->drive_level = fminf(s->drive_level + 0.05f, 0.8f);
                }
                prev_peak = current_peak;
                current_peak = 0.0f;
            }
            last_omega_sign = omega_sign;
            // Update unified virtual encoder for spin‑out detection.
            ve_set_velocity_global(s->omega);
            ve_update_global(p->dt);
            int ve_rotations = ve_get_completed_rotations_global();
            s->continuous_rotations = ve_rotations;
            if (ve_rotations >= SPINOUT_ROTATION_THRESHOLD) {
                ve_activate_anti_spin();
            } else {
                ve_deactivate_anti_spin();
            }
            // Override normal control if anti‑spin engaged.
            if (ve_is_anti_spin_active()) {
                u_energy = ve_get_anti_spin_output();
            }
        }
    } else {
        // At or above target energy: coast.  Terminate breakaway.
        u_energy = 0.0f;
        s->kick_active = false;
    }
    // Saturate command to configured swing‑up limit.
    if (u_energy > p->swing_sat) {
        u_energy = p->swing_sat;
    } else if (u_energy < -p->swing_sat) {
        u_energy = -p->swing_sat;
    }
    // Store command in state and return.
    s->u = u_energy;
    return u_energy;
}