#ifndef CONFIG_H
#define CONFIG_H

/*
 * Centralized Configuration Header
 * 
 * This header contains all common configuration constants used across both
 * the PC simulator and embedded Pico implementation. By centralizing these
 * values, we prevent configuration drift and ensure consistent behavior
 * between platforms.
 * 
 * CRITICAL: Any changes to these values should be tested on both platforms
 * to ensure consistent pendulum behavior.
 */

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// ENERGY CONTROL CONFIGURATION
// ============================================================================

// Energy target multiplier - CRITICAL PARAMETER
// This multiplier determines the target energy level for swing-up.
// Value represents multiple of minimum energy needed to reach upright position.
// Based on working Pico implementation that successfully achieves swing-up.
#define ENERGY_TARGET_MULTIPLIER    3.0f    // Working value from Pico implementation

// Energy pump gain multiplier for k_energy calculation
// Used in: k_energy = ENERGY_PUMP_GAIN_MULTIPLIER * sqrtf(mass_ratio)
#define ENERGY_PUMP_GAIN_MULTIPLIER 3.0f    // Increased from 1.5f for better energy pumping

// ============================================================================
// SWING-UP CONTROL PARAMETERS
// ============================================================================

// Saturation limits
#define SWING_SATURATION            1.0f    // Maximum motor power during swing-up [0..1] - FULL POWER
#define BALANCE_SATURATION          0.8f    // Maximum motor power during balance [0..1] - INCREASED for strong control

// Handover thresholds for switching from swing-up to balance
#define THETA_CATCH_DEGREES         20.0f   // Angle threshold in degrees from upright
#define OMEGA_CATCH_PC              3.0f    // Angular velocity threshold for PC (rad/s)
#define OMEGA_CATCH_PICO            1.5f    // Angular velocity threshold for Pico (rad/s) - more conservative

// Convert angle threshold to radians
#define THETA_CATCH_RADIANS         (THETA_CATCH_DEGREES * 3.14159265359f / 180.0f)

// ============================================================================
// BRAKING SYSTEM CONFIGURATION
// ============================================================================

// Anti-spin emergency threshold - ROTATION-BASED ONLY
// Anti-spin protection is triggered after completing SPINOUT_ROTATION_THRESHOLD rotations
// When activated, applies light braking until speed drops to target percentage of peak
#define ANTI_SPIN_COAST_TARGET_PERCENT  0.25f   // Coast down to 25% of peak speed before resuming control
#define ANTI_SPIN_BRAKE_DUTY           0.3f    // Light braking duty during coast-down to prevent energy buildup

// Progressive braking levels for overshoot protection
#define BRAKE_LIGHT                 0.3f    // Light braking for swing-up transitions
#define BRAKE_MODERATE              0.7f    // Moderate braking for high speeds  
#define BRAKE_STRONG                0.9f    // Strong braking for very high speeds
#define BRAKE_MAXIMUM               1.0f    // Maximum braking for extreme speeds

// ============================================================================
// ENERGY PREDICTION SYSTEM PARAMETERS
// ============================================================================

// Spin-out detection threshold
#define SPINOUT_ROTATION_THRESHOLD  5       // Number of continuous rotations to trigger spin-out braking

// Breakaway detection and drive parameters
#define INITIAL_DRIVE_LEVEL         0.8f    // Initial duty cycle for swing-up (80%) - INCREASED MORE for maximum power
#define BREAKAWAY_DUTY              0.8f    // Duty cycle for breakaway kick - MATCH PC VERSION

// Rest detection thresholds
#define STATIONARY_ANGLE_THRESHOLD  0.05f   // Radians (~3°) for near-rest detection
#define STATIONARY_SPEED_THRESHOLD  0.05f   // rad/s for near-rest detection
#define MOVEMENT_THRESHOLD          0.035f  // Radians (~2°) for movement detection

// Energy control thresholds
#define ENERGY_CATCH_THRESHOLD      0.80f   // Fraction of target energy for catch transition

// ============================================================================
// PHYSICAL CONSTANTS
// ============================================================================

#define GRAVITY_ACCEL               9.81f   // Gravitational acceleration (m/s²)

// ============================================================================
// UTILITY MACROS
// ============================================================================

// Calculate energy target for given parameters
#define CALCULATE_ENERGY_TARGET(mass, length) \
    ((mass) * GRAVITY_ACCEL * ((length) * 0.5f) * ENERGY_TARGET_MULTIPLIER)

// Calculate moment of inertia for rod + motor
#define CALCULATE_MOMENT_OF_INERTIA(mass, length, motor_inertia) \
    (((mass) * (length) * (length)) / 3.0f + (motor_inertia))

#ifdef __cplusplus
}
#endif

#endif // CONFIG_H
