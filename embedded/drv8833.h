
#ifndef DRV8833_H
#define DRV8833_H

#include <stdbool.h>

// Conditional includes for Pico vs PC builds
#if defined(PICO_BOARD) || defined(PICO_PLATFORM)
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#define IS_PICO_BUILD 1
#else
// PC build compatibility - define missing types
#include <stdint.h>
typedef unsigned int uint;
// Include PWM simulation for PC builds
#include "pwm_simulation.h"
#define IS_PICO_BUILD 0
#endif

#include "motor_protection.h"  // Unified motor protection system

#ifdef __cplusplus
extern "C" {
#endif

// Motor control modes
typedef enum {
    DRV8833_SIGN_MAGNITUDE,    // Traditional sign-magnitude PWM (one pin PWM, other low)
    DRV8833_LOCKED_ANTIPHASE   // Locked anti-phase PWM (both pins PWM, 180° out of phase)
} drv8833_mode_t;

typedef struct {
    uint gpio_in1;
    uint gpio_in2;
    uint slice_in1;
    uint slice_in2;
    uint chan_in1;
    uint chan_in2;
    uint32_t pwm_wrap;
    float pwm_freq_hz;
    float supply_voltage;
    drv8833_mode_t control_mode;                     // PWM control mode
    motor_protection_t protection;                   // Unified motor protection system
    float current_command;                           // Current motor command
    float current_motor_speed;                       // Current motor speed for thermal calculations
    
#if !IS_PICO_BUILD
    // PC simulation components
    pwm_generator_t pwm_sim;                        // Real PWM signal generator
    motor_model_t motor_sim;                        // Physical motor model
    bool needs_pwm_update;                          // Flag for simulation stepping
#endif
} drv8833_t;

// Initialize DRV8833 two-pin control on chosen GPIOs; sets PWM at freq_hz (default 25k)
bool drv8833_init(drv8833_t *m, uint gpio_in1, uint gpio_in2, float freq_hz);

// Set motor control mode (sign-magnitude vs locked anti-phase)
void drv8833_set_mode(drv8833_t *m, drv8833_mode_t mode);

// Command normalized voltage u in [-1, 1] (sign-magnitude PWM). 0 => coast.
// Positive u: IN1 PWM, IN2 low; Negative u: IN2 PWM, IN1 low.
// Motor protection is applied automatically inside this function.
void drv8833_cmd(drv8833_t *m, float u);

// Emergency command that bypasses thermal protection for critical safety braking
// Use this only for anti-spin emergency braking - thermal protection is disabled
void drv8833_emergency_cmd(drv8833_t *m, float u);

// Optional brake (both high) for emergency
void drv8833_brake(const drv8833_t *m);

// Change PWM frequency (recompute wrap)
void drv8833_set_freq(drv8833_t *m, float freq_hz);

// Motor protection status queries - unified interface
bool drv8833_is_protection_active(const drv8833_t *m);
bool drv8833_is_emergency_brake_active(const drv8833_t *m);
bool drv8833_is_thermal_halted(const drv8833_t *m);
float drv8833_get_protection_energy_used(const drv8833_t *m);
float drv8833_get_protection_utilization(const drv8833_t *m);  // Returns 0.0-1.0

#ifndef PICO_PLATFORM
// PC simulation functions - step PWM and motor model at high frequency
float drv8833_step_simulation(drv8833_t *m, float dt_seconds, float pendulum_omega);
void drv8833_get_simulation_state(const drv8833_t *m, float *voltage, float *current, float *torque);
#endif

#ifdef __cplusplus
}
#endif

#endif
