// Enhanced Physics Simulation for Inverted Pendulum Control
// Industrial-grade accuracy based on comprehensive research analysis
// File: src/physics/enhanced_physics.hpp

#ifndef ENHANCED_PHYSICS_HPP
#define ENHANCED_PHYSICS_HPP

#ifdef PC_DEBUG

#include <cmath>
#include <vector>
#include <memory>
#include <iostream>
#include <random>
#include <chrono>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Forward declarations
class EnhancedMotorModel;
class LuGreFrictionModel;
class SensorEffectsModel;

// ============================================================================
// ENHANCED MOTOR MODEL - Industrial Grade Accuracy
// ============================================================================

class EnhancedMotorModel {
private:
    // Electromagnetic parameters (±2-5% accuracy required per research)
    float Ra_;           // Armature resistance (Ω)
    float La_;           // Armature inductance (H)
    float Ke_;           // Back-EMF constant (V·s/rad)
    float Kt_;           // Torque constant (N·m/A)
    float tau_e_;        // Electrical time constant (s)
    
    // Mechanical parameters
    float Jm_;           // Motor inertia (kg·m²)
    float b_motor_;      // Motor viscous friction (N·m·s/rad)
    
    // Temperature compensation (research: ±0.4%/°C for resistance)
    float temp_coeff_R_; // Resistance temperature coefficient (/°C)
    float temp_coeff_K_; // Back-EMF temperature coefficient (/°C)
    float temperature_;  // Operating temperature (°C)
    
    // Cogging torque model (major error source per research)
    float cogging_amplitude_;  // Peak cogging torque (N·m)
    float cogging_period_;     // Cogging period (rad)
    int cogging_harmonics_;    // Number of harmonics to model
    
    // State variables
    float current_;      // Motor current (A)
    float omega_motor_;  // Motor angular velocity (rad/s)
    float theta_motor_;  // Motor position (rad)
    float applied_voltage_; // Currently applied voltage
    
public:
    EnhancedMotorModel();
    
    void setParameters(float Ra, float La, float Ke, float Kt);
    void setTemperature(float temp_celsius);
    void setCoggingParameters(float amplitude, float period, int harmonics);
    
    // Temperature-compensated parameters
    float getRaCompensated() const;
    float getKeCompensated() const;
    float getKtCompensated() const;
    
    // Cogging torque calculation (position-dependent)
    float getCoggingTorque() const;
    
    // Enhanced motor dynamics with electrical time constant
    float updateMotor(float voltage_cmd, float load_torque, float dt);
    
    // State accessors
    float getCurrent() const { return current_; }
    float getOmega() const { return omega_motor_; }
    float getTheta() const { return theta_motor_; }
    float getAppliedVoltage() const { return applied_voltage_; }
    
    // Diagnostic functions
    float getElectricalTimeConstant() const { return tau_e_; }
    float getBackEMF() const { return getKeCompensated() * omega_motor_; }
    float getPowerDissipation() const { return current_ * current_ * getRaCompensated(); }
    float getTemperature() const { return temperature_; }
};

// ============================================================================
// ADVANCED FRICTION MODEL - LuGre Model for High Accuracy
// ============================================================================

class LuGreFrictionModel {
private:
    // LuGre parameters
    float sigma0_;       // Stiffness coefficient (N·m/rad)
    float sigma1_;       // Damping coefficient (N·m·s/rad)
    float sigma2_;       // Viscous friction coefficient (N·m·s/rad)
    float Fc_;           // Coulomb friction (N·m)
    float Fs_;           // Static friction (N·m) 
    float vs_;           // Stribeck velocity (rad/s)
    
    // Internal state
    float z_;            // Average deflection of bristles
    
public:
    LuGreFrictionModel();
    
    void setParameters(float Fc, float Fs, float vs);
    void setAdvancedParameters(float sigma0, float sigma1, float sigma2);
    
    float getFrictionTorque(float velocity, float dt);
    
    // Simplified model for comparison
    static float getSimpleFriction(float velocity, float coulomb, float viscous);
    
    // Reset internal state
    void reset() { z_ = 0.0f; }
    
    // Diagnostics
    float getBristleDeflection() const { return z_; }
};

// ============================================================================
// SENSOR EFFECTS MODEL - Noise and Quantization
// ============================================================================

class SensorEffectsModel {
private:
    // Noise parameters (research-based)
    float angle_noise_std_;          // Angular measurement noise (rad)
    float velocity_noise_std_;       // Velocity measurement noise (rad/s)
    float quantization_bits_;        // ADC resolution (bits)
    float measurement_range_;        // Full-scale measurement range
    
    // Bias and drift parameters
    float angle_bias_;               // Constant bias error
    float velocity_bias_;            // Velocity bias
    float bias_drift_rate_;          // Bias drift (rad/s²)
    
    // Internal state for drift simulation
    float time_accumulated_;
    
    // Random number generation
    mutable std::mt19937 rng_;
    mutable std::normal_distribution<float> normal_dist_;
    
public:
    SensorEffectsModel();
    
    void setNoiseParameters(float angle_std, float velocity_std, int adc_bits);
    void setBiasParameters(float angle_bias, float velocity_bias, float drift_rate);
    
    float addAngleNoise(float true_angle, float dt) const;
    float addVelocityNoise(float true_velocity, float dt) const;
    
    // Reset sensor state
    void reset();
    
    // Diagnostics
    float getAngleBias() const { return angle_bias_; }
    float getVelocityBias() const { return velocity_bias_; }
};

// ============================================================================
// ENHANCED PENDULUM PHYSICS - RK2 Integration (Optimal Choice)
// ============================================================================

class EnhancedPendulumPhysics {
public:
    // Integration method selector
    enum IntegrationMethod {
        EULER,
        MODIFIED_EULER,  // Euler-Cromer (excellent stability)
        RK2,            // Heun's method (optimal for real-time)
        RK4             // Maximum accuracy
    };
    
private:
    // State variables
    float theta_;        // Angle from vertical (upright = 0)
    float omega_;        // Angular velocity
    
    // Physical parameters
    float mass_;         // Pendulum mass
    float length_;       // Pendulum length
    float gravity_;      // Gravity constant
    float dt_;           // Integration timestep
    
    // Enhanced models
    std::unique_ptr<EnhancedMotorModel> motor_;
    std::unique_ptr<LuGreFrictionModel> friction_;
    std::unique_ptr<SensorEffectsModel> sensor_;
    
    // Integration method
    IntegrationMethod integration_method_;
    
    // Applied torque storage
    float applied_torque_;
    float applied_voltage_; // Store motor voltage separately
    
    // Performance tracking
    mutable float validation_error_;
    
    // Dynamics calculation function
    std::pair<float, float> calculateDynamics(float theta, float omega) const;
    
public:
    EnhancedPendulumPhysics();
    ~EnhancedPendulumPhysics() = default;
    
    // Configuration
    void setParameters(float mass, float length, float b_vis, float tau_ff);
    void setIntegrationMethod(IntegrationMethod method);
    void setTimestep(float dt) { dt_ = dt; }
    
    // Motor interface
    void setTorque(float torque) { applied_torque_ = torque; }
    void setMotorVoltage(float voltage);
    
    // Simulation step
    void step();
    
    // State accessors with sensor effects
    float getTheta() const;
    float getOmega() const;
    float getThetaBottom() const;
    
    // Pure state (no sensor effects) for validation
    float getTrueTheta() const { return theta_; }
    float getTrueOmega() const { return omega_; }
    
    // Energy calculation
    float getEnergy() const;
    
    // Configuration
    void reset(float initial_theta = M_PI, float initial_omega = 0);
    void resetToBottom();  // Explicitly reset to bottom position
    void setNoise(float meas_noise, float proc_noise);
    void setTemperature(float temp_celsius);
    
    // Enhanced diagnostics
    void printDiagnostics() const;
    float compareToSimpleModel(float simple_theta, float simple_omega) const;
    
    // Model component access
    const EnhancedMotorModel& getMotorModel() const { return *motor_; }
    const LuGreFrictionModel& getFrictionModel() const { return *friction_; }
    const SensorEffectsModel& getSensorModel() const { return *sensor_; }
    
    // Performance metrics
    float getValidationError() const { return validation_error_; }
    IntegrationMethod getIntegrationMethod() const { return integration_method_; }
    const char* getIntegrationMethodName() const;
};

// ============================================================================
// VALIDATION AND BENCHMARKING UTILITIES
// ============================================================================

class PhysicsValidator {
private:
    std::vector<float> reference_angles_;
    std::vector<float> reference_velocities_;
    std::vector<float> test_angles_;
    std::vector<float> test_velocities_;
    
public:
    void addReferencePoint(float angle, float velocity);
    void addTestPoint(float angle, float velocity);
    
    float calculateRMSError() const;
    float calculateMaxError() const;
    float calculateSettlingTimeError(float target_settling_time) const;
    
    void generateValidationReport(const std::string& filename) const;
    void clear();
};

// ============================================================================
// INDUSTRIAL PERFORMANCE METRICS
// ============================================================================

struct PerformanceMetrics {
    float settling_time;        // Time to reach ±5° (research target)
    float max_overshoot;        // Maximum angle deviation
    float steady_state_error;   // Final steady-state error
    float rms_error;           // RMS tracking error
    float validation_error;     // Model validation error
    char performance_grade;     // A/B/C/F grade
    
    bool meetsIndustrialTargets() const;
    void print() const;
};

PerformanceMetrics calculatePerformanceMetrics(
    const std::vector<std::pair<float, float>>& time_angle_data,
    float target_angle = 0.0f);

// ============================================================================
// FACTORY FUNCTIONS
// ============================================================================

// Create enhanced physics with industrial-grade parameters
std::unique_ptr<EnhancedPendulumPhysics> createIndustrialGradePhysics();

// Create enhanced physics with research validation setup
std::unique_ptr<EnhancedPendulumPhysics> createResearchValidationPhysics();

// Create simplified physics for comparison
std::unique_ptr<EnhancedPendulumPhysics> createSimplifiedPhysics();

#endif // PC_DEBUG

#endif // ENHANCED_PHYSICS_HPP
