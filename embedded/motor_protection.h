#ifndef MOTOR_PROTECTION_H
#define MOTOR_PROTECTION_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// MOTOR THERMAL PROTECTION SYSTEM
// Well-engineered thermal protection based on industry modeling principles
// =============================================================================

// -----------------------------------------------------------------------------
// Motor Thermal Parameters
// -----------------------------------------------------------------------------
// Force our thermal capacity values by undefining any previous definitions
#ifdef MOTOR_THERMAL_CAPACITY
#undef MOTOR_THERMAL_CAPACITY
#endif
#ifdef MOTOR_EMERGENCY_CAPACITY
#undef MOTOR_EMERGENCY_CAPACITY
#endif

#define MOTOR_THERMAL_CAPACITY 200.0f     // Joules - protection threshold (increased from 80J)
#define MOTOR_EMERGENCY_CAPACITY 250.0f   // Joules - emergency brake threshold (increased from 100J)

// UNIQUE IDENTIFIER TO VERIFY THIS HEADER IS BEING USED
#define MOTOR_PROTECTION_HEADER_VERSION 2025

// Use integer constant for preprocessor check
#define MOTOR_THERMAL_CAPACITY_INT 200

// COMPILATION TEST: Force compile error if wrong header used
#if MOTOR_THERMAL_CAPACITY_INT != 200
#error "Wrong motor_protection.h included! Expected MOTOR_THERMAL_CAPACITY = 200J"
#endif

#define MOTOR_THERMAL_TIME_CONSTANT 30.0f // Seconds - thermal decay time constant
#define MOTOR_SUPPLY_VOLTAGE 5.0f         // Volts
#define MOTOR_RESISTANCE 2.0f             // Ohms (DC resistance)
#define MOTOR_BACK_EMF_CONSTANT 0.01f     // V/(rad/s) - speed-voltage constant
#define INTEGRATION_RATE_HZ 1000.0f       // 1kHz integration rate

// -----------------------------------------------------------------------------
// Type Definitions
// -----------------------------------------------------------------------------

// Thermal state enumeration for clear state management
typedef enum {
    THERMAL_NORMAL = 0,      // Normal operation
    THERMAL_ELEVATED,        // Elevated temperature but safe
    THERMAL_PROTECTION,      // Active thermal protection
    THERMAL_EMERGENCY,       // Emergency thermal shutdown
    THERMAL_HALT            // System halted waiting for order-of-magnitude decay
} thermal_state_t;

// Industry-inspired thermal model structure
typedef struct {
    // Core thermal state
    thermal_state_t state;              // Current thermal protection state
    float thermal_energy;               // Accumulated thermal energy [J]
    float thermal_power;                // Instantaneous thermal power [W]
    float peak_thermal_energy;          // Peak thermal energy for decay tracking [J]
    bool thermal_halt_active;           // System halted waiting for thermal decay
    
    // Motor operating conditions
    float current_duty_cycle;           // Current commanded duty cycle [-1, 1]
    float motor_speed_rad_s;            // Motor speed for back-EMF compensation
    
    // Thermal model parameters
    float i2r_losses;                   // Resistive heating power [W]
    float mechanical_losses;            // Friction and windage losses [W] 
    float switching_losses;             // PWM switching losses [W]
    
    // Integration and timing
    uint32_t last_timestamp_ms;         // Last integration timestamp [ms]
    float operation_time;               // Total operation time [s]
    bool initialized;                   // System initialized flag
    
    // Thermal protection thresholds with hysteresis
    float protection_threshold;         // Energy level for protection activation [J]
    float protection_clear_threshold;   // Energy level for protection deactivation [J]
    float emergency_threshold;          // Energy level for emergency shutdown [J]
    float emergency_clear_threshold;    // Energy level for emergency clearance [J]
} thermal_integrator_t;

// Motor protection interface 
typedef struct {
    thermal_integrator_t thermal;       // Thermal protection system
    float current_command;              // Current motor command [-1, 1]
    float pwm_frequency;                // PWM frequency [Hz]
    float supply_voltage;               // Motor supply voltage [V]
    bool initialized;                   // Protection system ready
} motor_protection_t;

// -----------------------------------------------------------------------------
// Core API Functions
// -----------------------------------------------------------------------------

// Initialize motor protection system
bool motor_protection_init(motor_protection_t *protection, 
                          float pwm_freq_hz, float supply_voltage_v,
                          float motor_resistance_ohms);

// Set motor command with thermal analysis
void motor_protection_set_command(motor_protection_t *protection, 
                                 float duty_cycle, uint32_t timestamp_ms, 
                                 float motor_speed_rad_s);

// Step thermal protection system forward in time
void motor_protection_step(motor_protection_t *protection, uint32_t timestamp_ms);

// Get protected motor output (may be reduced from commanded value)
float motor_protection_get_output(const motor_protection_t *protection);

// Reset protection system
void motor_protection_reset(motor_protection_t *protection);

// -----------------------------------------------------------------------------
// Status Query Functions
// -----------------------------------------------------------------------------

// Thermal state queries
bool motor_protection_is_active(const motor_protection_t *protection);
bool motor_protection_is_emergency(const motor_protection_t *protection);
bool motor_protection_is_halted(const motor_protection_t *protection);
bool motor_protection_emergency_brake_active(const motor_protection_t *protection);
thermal_state_t motor_protection_get_state(const motor_protection_t *protection);

// Thermal measurements
float motor_protection_get_thermal_energy(const motor_protection_t *protection);
float motor_protection_get_thermal_power(const motor_protection_t *protection);
float motor_protection_get_thermal_utilization(const motor_protection_t *protection);

// -----------------------------------------------------------------------------
// Diagnostic Functions
// -----------------------------------------------------------------------------

// Individual power loss components
float motor_protection_get_i2r_losses(const motor_protection_t *protection);
float motor_protection_get_mechanical_losses(const motor_protection_t *protection);
float motor_protection_get_switching_losses(const motor_protection_t *protection);

// -----------------------------------------------------------------------------
// Legacy Compatibility
// -----------------------------------------------------------------------------

// Legacy compatibility functions for old API
void motor_protection_set_command_old(motor_protection_t *protection, 
                                     float duty_cycle, uint32_t timestamp_ms);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_PROTECTION_H
