#ifndef CONTROL_H
#define CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Physical params (tune to your build)
    float m;     // rod mass (kg)
    float L;     // rod length (m)
    float Jm;    // motor+hub inertia about axis (kg*m^2)
    float b_vis; // viscous friction (N*m*s)
    float tau_ff; // Coulomb friction bias (N*m), sign-dependent
    
    // Motor model (torque mapping)
    float tau_max;   // max torque available at driver current limit (N*m)
    float u_to_tau;  // torque per unit command (N*m per 1.0 PWM), used for observer (approx)
    
    // Control loop
    float dt;        // control timestep (s)
    
    // Swing-up
    float k_energy;  // gain for energy pump
    float swing_sat; // |u| saturation during swing-up [0..1]
    
    // Balance (LQR-like)
    float Kp, Kd, Ki;     // fallback PD+I
    float ui_max;         // integrator clamp
    float balance_sat;    // |u| saturation in balance
    
    // Handover thresholds
    float theta_catch;  // rad from upright
    float omega_catch;  // rad/s
    
    // Braking parameters
    float brake_light;     // light braking strength [0..1] (default 0.3)
    float brake_moderate;  // moderate braking strength [0..1] (default 0.7)
    float brake_strong;    // strong braking strength [0..1] (default 0.9)
    float brake_maximum;   // maximum braking strength [0..1] (default 1.0)
} ctrl_params_t;

typedef struct {
    float theta_b; // angle from bottom, rad (-pi..pi]
    float theta_u; // angle from upright (theta_b - pi) - RAW for energy control
    float omega;   // rad/s estimate - FILTERED for stability
    float u;       // command [-1..1]
    
    // Filtered values for balance controller
    float theta_u_filtered; // Kalman-filtered angle for precise balance control
    float omega_filtered;   // Kalman-filtered velocity for precise balance control
    
    // integrator
    float ui;
    
    // energies
    float E;   // total energy about bottom
    float Edes;
    
    // NEW: breakaway and drive state variables
    bool  kick_active;     // true if a continuous breakaway push is in progress
    int   kick_direction;  // +1 or -1, direction of the current kick
    float drive_level;     // duty cycle used for swing-up once motion is detected
    
    // NEW: Advanced energy prediction and overshoot estimation
    float energy_bleed_gain;        // adaptive gain for energy bleeding (starts at 1.0)
    float last_apex_angle;          // last measured apex angle for calibration
    float last_omega_sign;          // track zero-crossings for apex detection
    float adaptive_brake_duty;      // current adaptive braking duty during upswing
    bool  apex_detected_this_cycle; // flag to track apex detection
    int   continuous_rotations;     // count of continuous full rotations for spin-out detection
    
    // state machine
    enum { ST_IDLE=0, ST_CALIB, ST_SWINGUP, ST_CATCH, ST_BALANCE, ST_FAULT } state;
} ctrl_state_t;

void ctrl_init(ctrl_params_t *p, ctrl_state_t *s);
void ctrl_reset_integrator(ctrl_state_t *s);
float ctrl_update(ctrl_params_t *p, ctrl_state_t *s);

// Main control step: returns motor command u in [-1,1]
float ctrl_step(const ctrl_params_t *p, ctrl_state_t *s);

#ifdef __cplusplus
}
#endif

#endif
