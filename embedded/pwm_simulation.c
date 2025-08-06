#include "pwm_simulation.h"
#include <math.h>
#include <string.h>

void pwm_generator_init(pwm_generator_t *pwm, float frequency_hz, float supply_voltage) {
    memset(pwm, 0, sizeof(pwm_generator_t));
    pwm->frequency_hz = frequency_hz;
    pwm->supply_voltage = supply_voltage;
    pwm->duty_cycle = 0.0f;
    pwm->time_accumulator = 0.0;
}

void pwm_generator_set_sign_magnitude(pwm_generator_t *pwm, float command) {
    // Sign-magnitude PWM: one pin PWM, other pin low
    if (command >= 0.0f) {
        pwm->duty_cycle = command;
        pwm->phase_offset = 0.0f; // Pin 1 PWM, Pin 2 low
    } else {
        pwm->duty_cycle = -command;
        pwm->phase_offset = 1.0f; // Pin 1 low, Pin 2 PWM
    }
    
    // Clamp duty cycle
    if (pwm->duty_cycle > 1.0f) pwm->duty_cycle = 1.0f;
    if (pwm->duty_cycle < 0.0f) pwm->duty_cycle = 0.0f;
}

void pwm_generator_set_locked_antiphase(pwm_generator_t *pwm, float command) {
    // Locked anti-phase: both pins PWM, 180° out of phase
    // Duty cycle = 0.5 + command/2
    pwm->duty_cycle = 0.5f + command * 0.5f;
    pwm->phase_offset = 0.5f; // 180° phase offset
    
    // Clamp duty cycle
    if (pwm->duty_cycle > 1.0f) pwm->duty_cycle = 1.0f;
    if (pwm->duty_cycle < 0.0f) pwm->duty_cycle = 0.0f;
}

float pwm_generator_step(pwm_generator_t *pwm, float dt_seconds) {
    // Update time accumulator
    pwm->time_accumulator += dt_seconds;
    
    // Calculate current phase in PWM cycle [0.0, 1.0)
    double pwm_period = 1.0 / pwm->frequency_hz;
    double phase = fmod(pwm->time_accumulator, pwm_period) / pwm_period;
    
    // Generate pin states based on duty cycle and mode
    if (pwm->phase_offset == 0.0f) {
        // Sign-magnitude mode: Pin 1 PWM, Pin 2 low
        pwm->pin1_state = (phase < pwm->duty_cycle);
        pwm->pin2_state = false;
    } else if (pwm->phase_offset == 1.0f) {
        // Sign-magnitude mode: Pin 1 low, Pin 2 PWM  
        pwm->pin1_state = false;
        pwm->pin2_state = (phase < pwm->duty_cycle);
    } else {
        // Locked anti-phase mode: complementary PWM
        pwm->pin1_state = (phase < pwm->duty_cycle);
        double phase2 = fmod(phase + 0.5, 1.0); // 180° offset
        pwm->pin2_state = (phase2 < (1.0 - pwm->duty_cycle));
    }
    
    // Calculate instantaneous motor voltage (Pin1 - Pin2)
    float pin1_voltage = pwm->pin1_state ? pwm->supply_voltage : 0.0f;
    float pin2_voltage = pwm->pin2_state ? pwm->supply_voltage : 0.0f;
    return pin1_voltage - pin2_voltage;
}

void motor_model_init(motor_model_t *motor, float resistance, float inductance,
                      float back_emf_k, float torque_k, float inertia, float damping,
                      float pwm_frequency, float simulation_frequency) {
    memset(motor, 0, sizeof(motor_model_t));
    
    motor->resistance = resistance;
    motor->inductance = inductance;
    motor->back_emf_constant = back_emf_k;
    motor->torque_constant = torque_k;
    motor->rotor_inertia = inertia;
    motor->viscous_damping = damping;
    
    // Calculate how many simulation samples per PWM cycle
    motor->samples_per_pwm_cycle = (int)(simulation_frequency / pwm_frequency);
    if (motor->samples_per_pwm_cycle > 1000) motor->samples_per_pwm_cycle = 1000;
    if (motor->samples_per_pwm_cycle < 10) motor->samples_per_pwm_cycle = 10;
}

void motor_model_apply_voltage(motor_model_t *motor, float voltage, float dt_seconds) {
    // Store voltage sample for RMS calculation
    motor->voltage_samples[motor->sample_index] = voltage;
    motor->sample_index = (motor->sample_index + 1) % motor->samples_per_pwm_cycle;
    
    // Calculate back EMF
    motor->back_emf = motor->back_emf_constant * motor->angular_velocity;
    
    // Calculate net voltage across motor
    float net_voltage = voltage - motor->back_emf;
    
    // Motor electrical equation: L * di/dt + R * i = V_net
    // Solve for current using simple Euler integration
    float di_dt = (net_voltage - motor->resistance * motor->current) / motor->inductance;
    motor->current += di_dt * dt_seconds;
    
    // Calculate output torque
    motor->torque_output = motor->torque_constant * motor->current;
    
    // Apply viscous damping to torque
    motor->torque_output -= motor->viscous_damping * motor->angular_velocity;
    
    // Store applied voltage for debugging
    motor->applied_voltage = voltage;
}

float motor_model_get_torque(const motor_model_t *motor) {
    return motor->torque_output;
}

float motor_model_get_current(const motor_model_t *motor) {
    return motor->current;
}

float motor_model_get_rms_current(const motor_model_t *motor) {
    // Calculate RMS current over the last PWM cycle
    float sum_squares = 0.0f;
    int samples_to_use = motor->samples_per_pwm_cycle;
    
    for (int i = 0; i < samples_to_use; i++) {
        int idx = (motor->sample_index - i - 1 + motor->samples_per_pwm_cycle) % motor->samples_per_pwm_cycle;
        float voltage = motor->voltage_samples[idx];
        
        // Approximate current from voltage (ignoring inductance for RMS calc)
        float approx_current = voltage / motor->resistance;
        sum_squares += approx_current * approx_current;
    }
    
    return sqrtf(sum_squares / samples_to_use);
}

void motor_model_set_angular_velocity(motor_model_t *motor, float omega) {
    motor->angular_velocity = omega;
}
