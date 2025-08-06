#include "drv8833.h"
#include "motor_protection.h"
#include <math.h>
#include <string.h>

// Platform-specific implementations
#ifdef PICO_PLATFORM
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
// PC build - use PWM simulation
#include "pwm_simulation.h"

static void pwm_init_gpio(uint gpio, float freq_hz, uint32_t *wrap_out, uint *slice_out, uint *chan_out) {
    *wrap_out = 1000; // Dummy wrap value
    *slice_out = 0;   // Dummy slice
    *chan_out = 0;    // Dummy channel
}

static inline void set_pwm_duty(const drv8833_t *m, uint slice, uint chan, float duty) {
    // PC build - PWM simulation handles this
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
    
#ifndef PICO_PLATFORM
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
    
#ifndef PICO_PLATFORM
    // Update PWM simulation frequency
    pwm_generator_init(&m->pwm_sim, freq_hz, m->supply_voltage);
#endif
    
    // Update protection system
    motor_protection_init(&m->protection, freq_hz, m->supply_voltage, 2.0f);
}

void drv8833_set_mode(drv8833_t *m, drv8833_mode_t mode) {
    if (!m) return;
    m->control_mode = mode;
    drv8833_cmd(m, 0.0f);
}

void drv8833_cmd(drv8833_t *m, float u) {
    if (!m) return;
    
    // Clamp input command
    if (u > 1.0f) u = 1.0f;
    if (u < -1.0f) u = -1.0f;
    
    // Store raw command for protection sampling
    m->current_command = u;
    
#ifdef PICO_PLATFORM
    // Real hardware - apply protection first, then set PWM
    float protected_duty = motor_protection_apply(&m->protection, fabsf(u));
    if (u < 0.0f) protected_duty = -protected_duty;
    
    // Apply PWM based on control mode
    switch (m->control_mode) {
        case DRV8833_SIGN_MAGNITUDE:
            if (protected_duty >= 0.0f) {
                set_pwm_duty(m, m->slice_in1, m->chan_in1, protected_duty);
                set_pwm_duty(m, m->slice_in2, m->chan_in2, 0.0f);
            } else {
                set_pwm_duty(m, m->slice_in1, m->chan_in1, 0.0f);
                set_pwm_duty(m, m->slice_in2, m->chan_in2, -protected_duty);
            }
            break;
            
        case DRV8833_LOCKED_ANTIPHASE:
            float duty1 = 0.5f + protected_duty * 0.5f;
            float duty2 = 0.5f - protected_duty * 0.5f;
            set_pwm_duty(m, m->slice_in1, m->chan_in1, duty1);
            set_pwm_duty(m, m->slice_in2, m->chan_in2, duty2);
            break;
    }
    
#else
    // PC simulation - generate real PWM waveform and sample it
    
    // Set PWM generator mode and command
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

#ifndef PICO_PLATFORM
// PC simulation - step PWM and motor model (called from main simulation loop)
float drv8833_step_simulation(drv8833_t *m, float dt_seconds, float pendulum_omega) {
    if (!m || !m->needs_pwm_update) return 0.0f;
    
    // Generate real PWM waveform voltage
    float instantaneous_voltage = pwm_generator_step(&m->pwm_sim, dt_seconds);
    
    // Sample the PWM duty cycle for protection system
    // Calculate current duty cycle from the PWM generator state
    float current_duty = m->pwm_sim.duty_cycle;
    if (m->current_command < 0.0f && m->control_mode == DRV8833_SIGN_MAGNITUDE) {
        current_duty = -current_duty; // Preserve sign for protection system
    }
    
    // Apply motor protection to the duty cycle
    float protected_duty = motor_protection_apply(&m->protection, fabsf(current_duty));
    
    // Apply protection by scaling the instantaneous voltage
    float protection_factor = 1.0f;
    if (motor_protection_emergency_brake_active(&m->protection)) {
        protection_factor = 0.0f;
    } else if (motor_protection_is_active(&m->protection)) {
        protection_factor = protected_duty / fabsf(current_duty);
        if (protection_factor != protection_factor) protection_factor = 0.0f; // Handle NaN
    }
    
    float protected_voltage = instantaneous_voltage * protection_factor;
    
    // Apply voltage to motor model
    motor_model_set_angular_velocity(&m->motor_sim, pendulum_omega);
    motor_model_apply_voltage(&m->motor_sim, protected_voltage, dt_seconds);
    
    // Return motor torque for pendulum physics
    return motor_model_get_torque(&m->motor_sim);
}

// Get motor simulation state for debugging
void drv8833_get_simulation_state(const drv8833_t *m, float *voltage, float *current, float *torque) {
    if (!m) return;
    if (voltage) *voltage = m->motor_sim.applied_voltage;
    if (current) *current = motor_model_get_current(&m->motor_sim);
    if (torque) *torque = motor_model_get_torque(&m->motor_sim);
}
#endif

void drv8833_brake(const drv8833_t *m) {
    if (!m) return;
    
#ifdef PICO_PLATFORM
    // Both pins high for brake
    set_pwm_duty(m, m->slice_in1, m->chan_in1, 1.0f);
    set_pwm_duty(m, m->slice_in2, m->chan_in2, 1.0f);
#else
    // PC simulation - set both pins high
    // This creates a brake condition
    m->pwm_sim.pin1_state = true;
    m->pwm_sim.pin2_state = true;
#endif
}

// Motor protection status queries - unified interface
bool drv8833_is_protection_active(const drv8833_t *m) {
    if (!m) return false;
    return motor_protection_is_active(&m->protection);
}

bool drv8833_is_emergency_brake_active(const drv8833_t *m) {
    if (!m) return false;
    return motor_protection_emergency_brake_active(&m->protection);
}

float drv8833_get_protection_energy_used(const drv8833_t *m) {
    if (!m) return 0.0f;
    return motor_protection_get_thermal_energy(&m->protection);
}

float drv8833_get_protection_utilization(const drv8833_t *m) {
    if (!m) return 0.0f;
    return motor_protection_get_thermal_utilization(&m->protection);
}
