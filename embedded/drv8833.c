#include "drv8833.h"
#include "motor_protection.h"
#include "pwm_simulation.h"
#include <math.h>
#include <string.h>
#include <stdio.h>

// Platform-specific implementations
#if IS_PICO_BUILD
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "pico/time.h"

static void pwm_init_gpio(uint gpio, float freq_hz, uint32_t *wrap_out, uint *slice_out, uint *chan_out) {
    gpio_set_function(gpio, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(gpio);
    uint chan  = pwm_gpio_to_channel(gpio);
    
    uint32_t sys_clk = 125000000; // 125 MHz default
    float clkdiv = 1.0f;
    uint32_t wrap;
    
    if (freq_hz < 2000.0f) {
        float required_wrap = (float)sys_clk / freq_hz;
        if (required_wrap > 65535.0f) {
            clkdiv = required_wrap / 65535.0f;
            wrap = 65535;
        } else {
            wrap = (uint32_t)required_wrap;
        }
    } else {
        wrap = (uint32_t)((float)sys_clk / freq_hz);
        if (wrap > 65535) { wrap = 65535; }
        if (wrap < 1) { wrap = 1; }
    }
    
    pwm_set_wrap(slice, wrap - 1);
    pwm_set_chan_level(slice, chan, 0);
    pwm_set_clkdiv(slice, clkdiv);
    pwm_set_enabled(slice, true);
    *wrap_out = wrap - 1;
    *slice_out = slice;
    *chan_out  = chan;
}

static inline void set_pwm_duty(const drv8833_t *m, uint slice, uint chan, float duty) {
    if (duty < 0.f) duty = 0.f; 
    if (duty > 1.f) duty = 1.f;
    pwm_set_chan_level(slice, chan, (uint16_t)(duty * (float)m->pwm_wrap));
}

#else
// PC build - use PWM simulation for real waveform generation
#include "pwm_simulation.h"

static void pwm_init_gpio(uint gpio, float freq_hz, uint32_t *wrap_out, uint *slice_out, uint *chan_out) {
    (void)gpio; (void)freq_hz; // Suppress unused parameter warnings
    *wrap_out = 1000; // Dummy wrap value
    *slice_out = 0;   // Dummy slice
    *chan_out = 0;    // Dummy channel
}

static inline void set_pwm_duty(const drv8833_t *m, uint slice, uint chan, float duty) {
    (void)m; (void)slice; (void)chan; (void)duty; // Suppress unused parameter warnings
    // PC build - PWM simulation handles this through the generator
}
#endif

bool drv8833_init(drv8833_t *m, uint gpio_in1, uint gpio_in2, float freq_hz) {
    if (!m) return false;
    
    memset(m, 0, sizeof(drv8833_t));
    
    m->gpio_in1 = gpio_in1;
    m->gpio_in2 = gpio_in2;
    m->pwm_freq_hz = freq_hz;
    m->supply_voltage = 5.0f;  // Default 5V supply
    m->control_mode = DRV8833_SIGN_MAGNITUDE;
    
    // Initialize PWM hardware (Pico) or simulation (PC)
    pwm_init_gpio(gpio_in1, freq_hz, &m->pwm_wrap, &m->slice_in1, &m->chan_in1);
    pwm_init_gpio(gpio_in2, freq_hz, &m->pwm_wrap, &m->slice_in2, &m->chan_in2);
    
#if !IS_PICO_BUILD
    // PC build - initialize PWM simulation and motor model
    pwm_generator_init(&m->pwm_sim, freq_hz, m->supply_voltage);
    motor_model_init(&m->motor_sim, 
                     2.0f,    // 2 ohm resistance
                     0.001f,  // 1 mH inductance  
                     0.01f,   // Back EMF constant
                     0.01f,   // Torque constant
                     1e-5f,   // Rotor inertia
                     1e-4f,   // Viscous damping
                     freq_hz, 
                     10000.0f); // 10 kHz simulation frequency
#endif
    
    // Initialize unified motor protection system
    motor_protection_init(&m->protection, freq_hz, m->supply_voltage, 2.0f);
    
    // Start with motor off
    drv8833_cmd(m, 0.0f);
    return true;
}

void drv8833_set_freq(drv8833_t *m, float freq_hz) {
    if (!m) return;
    
    pwm_init_gpio(m->gpio_in1, freq_hz, &m->pwm_wrap, &m->slice_in1, &m->chan_in1);
    pwm_init_gpio(m->gpio_in2, freq_hz, &m->pwm_wrap, &m->slice_in2, &m->chan_in2);
    m->pwm_freq_hz = freq_hz;
    
#if !IS_PICO_BUILD
    // Update PWM simulation frequency
    pwm_generator_init(&m->pwm_sim, freq_hz, m->supply_voltage);
#endif
    
    // Update protection system
    motor_protection_init(&m->protection, freq_hz, m->supply_voltage, 2.0f);
}

void drv8833_set_mode(drv8833_t *m, drv8833_mode_t mode) {
    if (!m) return;
    
    m->control_mode = mode;
    // Reset to safe state when changing modes
    drv8833_cmd(m, 0.0f);
}

void drv8833_cmd(drv8833_t *m, float u) {
    if (!m) return;
    
    // Clamp input command
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    
    // Store raw command for protection sampling
    m->current_command = u;
    
#if IS_PICO_BUILD
    // Real hardware - integration-based protection
    // CRITICAL FIX: Use real timestamp for proper thermal integration
    // This must match the timestamp system used in main.c motor_protection_step()
    uint32_t current_timestamp_ms = to_ms_since_boot(get_absolute_time());
    
    // Use stored motor speed if available, otherwise assume zero for Pico build
    float motor_speed = m->current_motor_speed;
    motor_protection_set_command(&m->protection, fabsf(u), current_timestamp_ms, motor_speed);
    
    float final_duty = motor_protection_get_output(&m->protection);
    if (u < 0.0f) final_duty = -final_duty; // Preserve sign
    
    // Apply PWM based on control mode
    switch (m->control_mode) {
        case DRV8833_SIGN_MAGNITUDE:
            if (final_duty >= 0.0f) {
                set_pwm_duty(m, m->slice_in1, m->chan_in1, final_duty);
                set_pwm_duty(m, m->slice_in2, m->chan_in2, 0.0f);
            } else {
                set_pwm_duty(m, m->slice_in1, m->chan_in1, 0.0f);
                set_pwm_duty(m, m->slice_in2, m->chan_in2, -final_duty);
            }
            break;
            
        case DRV8833_LOCKED_ANTIPHASE: {
            float duty1 = 0.5f + final_duty * 0.5f;
            float duty2 = 0.5f - final_duty * 0.5f;
            set_pwm_duty(m, m->slice_in1, m->chan_in1, duty1);
            set_pwm_duty(m, m->slice_in2, m->chan_in2, duty2);
            break;
        }
    }
    
#else
    // PC simulation - generate real PWM waveform and sample it
    // SYNCHRONIZATION FIX: Use shared time counter accessible from both cmd and step functions
    // Store raw command for protection sampling (timing will be handled in step_simulation)
    
    // Set PWM generator mode and command (PROTECTION APPLIED IN STEP_SIMULATION)
    switch (m->control_mode) {
        case DRV8833_SIGN_MAGNITUDE:
            pwm_generator_set_sign_magnitude(&m->pwm_sim, u);  // Use raw command here
            break;
        case DRV8833_LOCKED_ANTIPHASE:
            pwm_generator_set_locked_antiphase(&m->pwm_sim, u);  // Use raw command here
            break;
    }
    
    // This will be called by the simulation main loop to actually run the PWM
    m->needs_pwm_update = true;
#endif
}

// Emergency command that bypasses thermal protection for critical safety braking
void drv8833_emergency_cmd(drv8833_t *m, float u) {
    if (!m) return;
    
    // Clamp input command
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    
    // Store raw command for protection sampling
    m->current_command = u;
    
    // Removed arrow markers from debug output
    printf("EMERGENCY COMMAND: %.3f (BYPASSING THERMAL PROTECTION)\n", u);
    
#if IS_PICO_BUILD
    // Real hardware - apply command directly, bypassing protection
    // Apply PWM based on control mode
    switch (m->control_mode) {
        case DRV8833_SIGN_MAGNITUDE:
            if (u >= 0.0f) {
                set_pwm_duty(m, m->slice_in1, m->chan_in1, u);
                set_pwm_duty(m, m->slice_in2, m->chan_in2, 0.0f);
            } else {
                set_pwm_duty(m, m->slice_in1, m->chan_in1, 0.0f);
                set_pwm_duty(m, m->slice_in2, m->chan_in2, -u);
            }
            break;
            
        case DRV8833_LOCKED_ANTIPHASE: {
            float duty1 = 0.5f + u * 0.5f;
            float duty2 = 0.5f - u * 0.5f;
            set_pwm_duty(m, m->slice_in1, m->chan_in1, duty1);
            set_pwm_duty(m, m->slice_in2, m->chan_in2, duty2);
            break;
        }
    }
    
#else
    // PC simulation - set PWM generator directly without protection
    
    // Set PWM generator mode and command (use ORIGINAL command, bypassing protection)
    switch (m->control_mode) {
        case DRV8833_SIGN_MAGNITUDE:
            pwm_generator_set_sign_magnitude(&m->pwm_sim, u);
            break;
        case DRV8833_LOCKED_ANTIPHASE:
            pwm_generator_set_locked_antiphase(&m->pwm_sim, u);
            break;
    }
    
    // This will be called by the simulation main loop to actually run the PWM
    m->needs_pwm_update = true;
#endif
}

#if !IS_PICO_BUILD
// PC simulation - step PWM and motor model (called from main simulation loop)
float drv8833_step_simulation(drv8833_t *m, float dt_seconds, float pendulum_omega) {
    if (!m) return 0.0f;
    
    // Store current motor speed for thermal calculations
    m->current_motor_speed = pendulum_omega;
    
    // Clear the update flag
    m->needs_pwm_update = false;
    
    // SYNCHRONIZED THERMAL PROTECTION: Single time source for consistent protection
    static uint32_t unified_time_counter_ms = 0;
    unified_time_counter_ms += (uint32_t)(dt_seconds * 1000.0f); // Actual simulation time
    
    // Apply thermal protection with synchronized timing
    motor_protection_set_command(&m->protection, fabsf(m->current_command), unified_time_counter_ms, pendulum_omega);
    motor_protection_step(&m->protection, unified_time_counter_ms);
    
    // Get protected duty cycle and preserve sign
    float protected_duty = motor_protection_get_output(&m->protection);
    if (m->current_command < 0.0f) protected_duty = -protected_duty;
    
    // Generate real PWM waveform voltage using PROTECTED duty cycle
    // Update the PWM generator with protected command before stepping
    switch (m->control_mode) {
        case DRV8833_SIGN_MAGNITUDE:
            pwm_generator_set_sign_magnitude(&m->pwm_sim, protected_duty);
            break;
        case DRV8833_LOCKED_ANTIPHASE:
            pwm_generator_set_locked_antiphase(&m->pwm_sim, protected_duty);
            break;
    }
    
    float instantaneous_voltage = pwm_generator_step(&m->pwm_sim, dt_seconds);
    
    // Apply voltage to motor model (voltage is already protected by thermal system)
    motor_model_set_angular_velocity(&m->motor_sim, pendulum_omega);
    motor_model_apply_voltage(&m->motor_sim, instantaneous_voltage, dt_seconds);
    
    // Return motor torque for pendulum physics
    return motor_model_get_torque(&m->motor_sim);
}

// Get motor simulation state for debugging
void drv8833_get_simulation_state(const drv8833_t *m, float *voltage, float *current, float *torque) {
    if (!m) return;
    
    // Return the actual applied voltage (already protected by synchronized thermal system)
    if (voltage) *voltage = m->motor_sim.applied_voltage;
    if (current) *current = motor_model_get_current(&m->motor_sim);
    if (torque) *torque = motor_model_get_torque(&m->motor_sim);
}
#endif

void drv8833_brake(const drv8833_t *m) {
    if (!m) return;
    
    // Both pins high for brake
    set_pwm_duty(m, m->slice_in1, m->chan_in1, 1.0f);
    set_pwm_duty(m, m->slice_in2, m->chan_in2, 1.0f);
}

// Motor protection status queries - unified interface
bool drv8833_is_protection_active(const drv8833_t *m) {
    if (!m) return false;
    return motor_protection_is_active(&m->protection);
}

bool drv8833_is_emergency_brake_active(const drv8833_t *m) {
    if (!m) return false;
    return motor_protection_is_emergency(&m->protection);
}

bool drv8833_is_thermal_halted(const drv8833_t *m) {
    if (!m) return false;
    return motor_protection_is_halted(&m->protection);
}

float drv8833_get_protection_energy_used(const drv8833_t *m) {
    if (!m) return 0.0f;
    return motor_protection_get_thermal_energy(&m->protection);
}

float drv8833_get_protection_utilization(const drv8833_t *m) {
    if (!m) return 0.0f;
    // Calculate utilization as percentage of protection threshold
    const float energy = motor_protection_get_thermal_energy(&m->protection);
    return energy / MOTOR_THERMAL_CAPACITY;
}
