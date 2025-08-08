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
 * Note: Any changes to these values should be validated on both PC and embedded builds
 * to ensure consistent pendulum behaviour.
 */

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// ENERGY CONTROL CONFIGURATION
// ============================================================================

// Energy target multiplier.  This factor determines the desired energy level for swing‑up
// as a multiple of the minimum energy required to reach the upright position.
// Typical values around 3.0 provide robust swing‑up performance.
#define ENERGY_TARGET_MULTIPLIER    3.0f

// Energy pump gain multiplier for k_energy calculation.
// Used in: k_energy = ENERGY_PUMP_GAIN_MULTIPLIER * sqrtf(mass_ratio).
// A higher value increases the rate of energy pumping during swing‑up.
#define ENERGY_PUMP_GAIN_MULTIPLIER 3.0f

// ============================================================================
// SWING-UP CONTROL PARAMETERS
// ============================================================================

// Saturation limits
// Maximum motor power during swing‑up and balance.  Values are in the range [0..1].
#define SWING_SATURATION            1.0f
#define BALANCE_SATURATION          0.8f

// Handover thresholds for switching from swing‑up to balance.
// THETA_CATCH_DEGREES defines the angle (in degrees) from upright at which to catch the pendulum.
// OMEGA_CATCH_PC and OMEGA_CATCH_PICO define angular velocity thresholds (rad/s) for the PC and Pico implementations respectively.
#define THETA_CATCH_DEGREES         20.0f
#define OMEGA_CATCH_PC              3.0f
#define OMEGA_CATCH_PICO            1.5f

// Convert angle threshold to radians
#define THETA_CATCH_RADIANS         (THETA_CATCH_DEGREES * 3.14159265359f / 180.0f)

// ============================================================================
// BRAKING SYSTEM CONFIGURATION
// ============================================================================

// Anti-spin emergency threshold - ROTATION-BASED ONLY
// Anti-spin protection is triggered after completing SPINOUT_ROTATION_THRESHOLD rotations
// Actual anti-spin behavior is handled by the unified virtual encoder system
#define SPINOUT_ROTATION_THRESHOLD  1       // Number of continuous rotations to trigger spin-out braking

// Progressive braking levels for overshoot protection  
#define BRAKE_LIGHT                 0.3f    // Light braking for swing-up transitions
#define BRAKE_MODERATE              0.7f    // Moderate braking for high speeds  
#define BRAKE_STRONG                0.9f    // Strong braking for very high speeds
#define BRAKE_MAXIMUM               1.0f    // Maximum braking for extreme speeds

// ============================================================================
// ENERGY PREDICTION SYSTEM PARAMETERS
// ============================================================================

// Breakaway detection and drive parameters
// Initial duty cycle for swing‑up (80%).  Determines the initial motor authority during energy pumping.
#define INITIAL_DRIVE_LEVEL         0.8f
// Duty cycle for the breakaway kick when starting swing‑up.
#define BREAKAWAY_DUTY              0.8f

// Rest detection thresholds
// Thresholds for detecting when the pendulum is near rest and when it begins moving.
#define STATIONARY_ANGLE_THRESHOLD  0.05f   // radians (~3°) for near‑rest detection
#define STATIONARY_SPEED_THRESHOLD  0.05f   // rad/s for near‑rest detection
#define MOVEMENT_THRESHOLD          0.035f  // radians (~2°) for movement detection

// Energy control thresholds
// Fraction of the target energy at which to transition to catching the pendulum.
#define ENERGY_CATCH_THRESHOLD      0.80f

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
