#include "drv8833.h"
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
    
    // compute wrap for desired frequency: f = sys_clk / ((wrap+1)*clkdiv)
    uint32_t sys_clk = 125000000; // 125 MHz default
    float clkdiv = 1.0f;
    uint32_t wrap;
    
    // For very low frequencies, we need to use clock divider to avoid wrap overflow
    if (freq_hz < 2000.0f) {
        // Calculate required divider to keep wrap reasonable
        float required_wrap = (float)sys_clk / freq_hz;
        if (required_wrap > 65535.0f) {
            clkdiv = required_wrap / 65535.0f;
            wrap = 65535;
        } else {
            wrap = (uint32_t)required_wrap;
        }
    } else {
        // Normal frequency range - compute wrap directly
        wrap = (uint32_t)((float)sys_clk / freq_hz);
        if (wrap > 65535) { wrap = 65535; }
        if (wrap < 1) { wrap = 1; }  // Only prevent zero wrap
    }
    
    pwm_set_wrap(slice, wrap - 1);
    pwm_set_chan_level(slice, chan, 0);  // Start with 0% duty cycle
    pwm_set_clkdiv(slice, clkdiv);
    pwm_set_enabled(slice, true);
    *wrap_out = wrap - 1;
    *slice_out = slice;
    *chan_out  = chan;
}

static inline void set_pwm(const drv8833_t *m, uint slice, uint chan, float duty) {
    if (duty < 0.f) duty = 0.f; if (duty > 1.f) duty = 1.f;
    pwm_set_chan_level(slice, chan, (uint16_t)(duty * (float)m->pwm_wrap));
}

#else
// PC build stubs - no actual PWM hardware
static void pwm_init_gpio(uint gpio, float freq_hz, uint32_t *wrap_out, uint *slice_out, uint *chan_out) {
    *wrap_out = 1000; // Dummy wrap value
    *slice_out = 0;   // Dummy slice
    *chan_out = 0;    // Dummy channel
}

static inline void set_pwm(const drv8833_t *m, uint slice, uint chan, float duty) {
    // PC build - no actual PWM hardware, just store the duty cycle
    // The motor protection system will sample this value
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
    
    // Initialize PWM hardware (or stubs for PC)
    pwm_init_gpio(gpio_in1, freq_hz, &m->pwm_wrap, &m->slice_in1, &m->chan_in1);
    pwm_init_gpio(gpio_in2, freq_hz, &m->pwm_wrap, &m->slice_in2, &m->chan_in2);
    
    // Initialize unified motor protection system
    // Typical small brushed motor: ~2 ohms resistance
    motor_protection_init(&m->protection, freq_hz, m->supply_voltage, 2.0f);
    
    // Start with motor off
    drv8833_cmd(m, 0.0f);
    return true;
}

void drv8833_set_freq(drv8833_t *m, float freq_hz) {
    if (!m) return;
    
    // Re-init both slices to new wrap
    pwm_init_gpio(m->gpio_in1, freq_hz, &m->pwm_wrap, &m->slice_in1, &m->chan_in1);
    pwm_init_gpio(m->gpio_in2, freq_hz, &m->pwm_wrap, &m->slice_in2, &m->chan_in2);
    m->pwm_freq_hz = freq_hz;
    
    // Update protection system with new frequency
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
    
    // Apply motor protection to get actual duty cycle
    float protected_duty = motor_protection_apply(&m->protection, fabsf(u));
    
    // Apply motor direction
    if (u < 0.0f) {
        protected_duty = -protected_duty;
    }
    
    // Apply PWM based on control mode
    switch (m->control_mode) {
        case DRV8833_SIGN_MAGNITUDE:
            if (protected_duty >= 0.0f) {
                // Forward: IN1 PWM, IN2 low
                set_pwm(m, m->slice_in1, m->chan_in1, protected_duty);
                set_pwm(m, m->slice_in2, m->chan_in2, 0.0f);
            } else {
                // Reverse: IN1 low, IN2 PWM
                set_pwm(m, m->slice_in1, m->chan_in1, 0.0f);
                set_pwm(m, m->slice_in2, m->chan_in2, -protected_duty);
            }
            break;
            
        case DRV8833_LOCKED_ANTIPHASE:
            // Both pins PWM, 180° out of phase
            // duty_cycle = 0.5 + command/2
            float duty1 = 0.5f + protected_duty * 0.5f;
            float duty2 = 0.5f - protected_duty * 0.5f;
            set_pwm(m, m->slice_in1, m->chan_in1, duty1);
            set_pwm(m, m->slice_in2, m->chan_in2, duty2);
            break;
    }
}

void drv8833_brake(const drv8833_t *m) {
    if (!m) return;
    
    // Both pins high for brake
    set_pwm(m, m->slice_in1, m->chan_in1, 1.0f);
    set_pwm(m, m->slice_in2, m->chan_in2, 1.0f);
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
