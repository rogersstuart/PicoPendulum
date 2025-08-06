// Simple console-based pendulum simulator
// Compile with: g++ -std=c++17 -O2 -DPC_SIMULATION simple_simulator.cpp -o simple_simulator

#include <iostream>
#include <cmath>
#include <vector>
#include <chrono>
#include <thread>
#include <fstream>
#include <iomanip>

#define PC_SIMULATION
#define M_PI 3.14159265358979323846

// Mock hardware types for PC compilation
typedef float float32_t;
typedef unsigned int uint32_t;
typedef int int32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;
typedef bool bool_t;

// Mock Pico SDK functions
inline uint32_t to_ms_since_boot(void* time) {
    static auto start = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
}

inline void* get_absolute_time() { return nullptr; }
inline void watchdog_update() {}
inline void sleep_ms(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// Embedded control system structures and functions
extern "C" {
    typedef struct {
        float m, L, Jm, b_vis, tau_ff;
        float tau_max, u_to_tau, dt;
        float k_energy, swing_sat;
        float Kp, Kd, Ki, ui_max, balance_sat;
        float theta_catch, omega_catch;
    } ctrl_params_t;

    typedef struct {
        float theta_b, theta_u, omega, u, ui, E, Edes;
        bool kick_active;
        int kick_direction;
        float drive_level;
        enum { ST_IDLE=0, ST_CALIB, ST_SWINGUP, ST_CATCH, ST_BALANCE, ST_FAULT } state;
    } ctrl_state_t;

    static inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
    static inline float sgn(float x) { return x > 0 ? 1.0f : (x < 0 ? -1.0f : 0.0f); }
    static inline float wrap_pi(float x) {
        while (x > M_PI) x -= 2*M_PI;
        while (x <= -M_PI) x += 2*M_PI;
        return x;
    }

    void ctrl_init(ctrl_params_t *p, ctrl_state_t *s) {
        s->theta_b = 0.f; s->theta_u = 0.f; s->omega = 0.f; s->u = 0.f;
        s->ui = 0.f; s->E = 0.f; s->Edes = 0.f; s->state = 0;
        s->kick_active = false; s->kick_direction = 1; s->drive_level = 0.4f;
    }

    void ctrl_reset_integrator(ctrl_state_t *s) { 
        s->ui = 0.f; s->kick_active = false; s->drive_level = 0.4f;
    }

    static float energy_control(const ctrl_params_t *p, ctrl_state_t *s) {
        static const float STATIONARY_ANGLE = 0.05f, STATIONARY_SPEED = 0.05f;
        static const float MOVE_THRESHOLD = 0.035f, BREAKAWAY_DUTY = 0.8f;
        
        float cos_theta = cosf(s->theta_b);
        float J = (p->m * p->L * p->L) / 3.0f + p->Jm;
        s->E = 0.5f * J * s->omega * s->omega + p->m * 9.81f * (p->L * 0.5f) * (1.0f + cos_theta);
        float e = s->E - s->Edes;

        bool at_rest = (fabsf(s->theta_b) < STATIONARY_ANGLE) && (fabsf(s->omega) < STATIONARY_SPEED);

        if (e < 0.0f) {
            if (s->kick_active) {
                if (fabsf(s->theta_b) >= MOVE_THRESHOLD || fabsf(s->omega) >= STATIONARY_SPEED) {
                    s->kick_active = false; s->drive_level = 0.4f;
                }
                return BREAKAWAY_DUTY * s->kick_direction;
            } else if (at_rest) {
                s->kick_direction = fabsf(s->theta_b) > STATIONARY_ANGLE ? -sgn(s->theta_b) : -s->kick_direction;
                s->kick_active = true;
                return BREAKAWAY_DUTY * s->kick_direction;
            } else {
                return s->drive_level * sgn(s->omega);
            }
        }
        return 0.0f;
    }

    float ctrl_update(ctrl_params_t *p, ctrl_state_t *s) {
        s->theta_u = wrap_pi(s->theta_u);
        bool is_upright = (fabsf(s->theta_u) < p->theta_catch) && (fabsf(s->omega) < p->omega_catch);

        if (s->state == 4 && !is_upright) s->state = 2; // BALANCE -> SWINGUP

        float u = 0.0f;
        switch (s->state) {
            case 0: // IDLE
                if (!is_upright) s->state = 2;
                break;
            case 2: // SWINGUP
                u = energy_control(p, s);
                if (is_upright) { s->state = 4; ctrl_reset_integrator(s); }
                break;
            case 4: // BALANCE
                if (!is_upright) { s->state = 2; break; }
                float theta_u_wrapped = wrap_pi(s->theta_u);
                u = -p->Kp * theta_u_wrapped - p->Kd * s->omega;
                s->ui += p->Ki * theta_u_wrapped;
                s->ui = clampf(s->ui, -p->ui_max, p->ui_max);
                u = clampf(u + s->ui, -p->balance_sat, p->balance_sat);
                break;
        }
        return clampf(u, -1.0f, 1.0f);
    }

    float ctrl_step(const ctrl_params_t *p, ctrl_state_t *s) {
        return s->u = ctrl_update((ctrl_params_t*)p, s);
    }
}

// Simple physics simulation
class SimplePendulum {
private:
    float theta, omega, mass, length, damping, friction, dt;
    
public:
    SimplePendulum() : theta(M_PI), omega(0), mass(0.2f), length(0.3048f), 
                      damping(0.001f), friction(0.002f), dt(0.001f) {}
    
    void setParams(float m, float L, float b, float f) { mass = m; length = L; damping = b; friction = f; }
    void setTorque(float tau) {
        float I = mass * length * length / 3.0f;
        float gravity_torque = -mass * 9.81f * (length / 2.0f) * sin(theta);
        float damping_torque = -damping * omega;
        float friction_torque = -friction * sgn(omega);
        float alpha = (gravity_torque + damping_torque + friction_torque + tau) / I;
        
        omega += alpha * dt;
        theta += omega * dt;
        while (theta > M_PI) theta -= 2*M_PI;
        while (theta <= -M_PI) theta += 2*M_PI;
    }
    
    float getTheta() const { return theta; }
    float getOmega() const { return omega; }
    float getThetaBottom() const { return wrap_pi(theta + M_PI); }
    float getEnergy() const {
        float I = mass * length * length / 3.0f;
        return 0.5f * I * omega * omega + mass * 9.81f * (length / 2.0f) * (1.0f + cos(theta));
    }
    void reset(float t = M_PI, float w = 0) { theta = t; omega = w; }
};

// Console-based simulator
class ConsoleSimulator {
private:
    SimplePendulum physics;
    ctrl_params_t params;
    ctrl_state_t state;
    float sim_time;
    std::vector<std::tuple<float, float, float, float, float, int>> log_data;
    
public:
    ConsoleSimulator() : sim_time(0) {
        // Initialize control parameters
        ctrl_init(&params, &state);
        params.m = 0.2f; params.L = 0.3048f; params.Jm = 1e-5f;
        params.b_vis = 0.0002f; params.tau_ff = 0.001f;
        params.tau_max = 0.05f; params.u_to_tau = 0.05f; params.dt = 0.001f;
        params.k_energy = 1.0f; params.swing_sat = 1.0f;
        params.Kp = 5.0f; params.Kd = 1.0f; params.Ki = 0.1f;
        params.ui_max = 0.2f; params.balance_sat = 0.4f;
        params.theta_catch = 20.0f * M_PI / 180.0f; params.omega_catch = 1.5f;
        state.Edes = params.m * 9.81f * (params.L * 0.5f) * 2.2f;
        
        physics.setParams(params.m, params.L, params.b_vis, params.tau_ff);
    }
    
    void run(float duration = 10.0f, bool verbose = false) {
        std::cout << "=== Pendulum Control Simulation ===\n";
        std::cout << "Duration: " << duration << "s, Control: " << 
                     (state.state == 0 ? "IDLE" : state.state == 2 ? "SWINGUP" : "BALANCE") << "\n";
        std::cout << "Parameters: m=" << params.m << "kg, L=" << params.L << "m, Kp=" << params.Kp << "\n\n";
        
        if (verbose) {
            std::cout << std::setw(8) << "Time" << std::setw(10) << "Theta(°)" << std::setw(10) << "Omega" 
                      << std::setw(10) << "Motor" << std::setw(10) << "Energy" << std::setw(10) << "State\n";
            std::cout << std::string(60, '-') << "\n";
        }
        
        auto start_time = std::chrono::steady_clock::now();
        int step = 0;
        
        while (sim_time < duration) {
            // Update control with physics measurements
            state.theta_b = physics.getThetaBottom();
            state.theta_u = state.theta_b - M_PI;
            state.omega = physics.getOmega();
            
            // Run control algorithm
            float motor_cmd = ctrl_step(&params, &state);
            
            // Apply to physics
            physics.setTorque(motor_cmd * params.u_to_tau);
            
            // Log data every 10ms
            if (step % 10 == 0) {
                log_data.push_back(std::make_tuple(sim_time, state.theta_u * 180/M_PI, 
                                                 state.omega, motor_cmd, physics.getEnergy(), state.state));
                
                if (verbose && step % 100 == 0) { // Print every 100ms
                    const char* state_names[] = {"IDLE", "CALIB", "SWINGUP", "CATCH", "BALANCE", "FAULT"};
                    std::cout << std::setw(8) << std::fixed << std::setprecision(2) << sim_time
                              << std::setw(10) << state.theta_u * 180/M_PI
                              << std::setw(10) << std::setprecision(3) << state.omega
                              << std::setw(10) << motor_cmd
                              << std::setw(10) << physics.getEnergy()
                              << std::setw(10) << state_names[state.state] << "\n";
                }
            }
            
            sim_time += params.dt;
            step++;
            
            // Check for real-time execution
            auto elapsed = std::chrono::steady_clock::now() - start_time;
            if (std::chrono::duration<float>(elapsed).count() > sim_time + 1.0f) {
                std::cout << "\nSimulation running slower than real-time...\n";
            }
        }
        
        // Print summary
        auto final_angle = state.theta_u * 180 / M_PI;
        auto final_energy = physics.getEnergy();
        std::cout << "\n=== Simulation Complete ===\n";
        std::cout << "Final angle: " << std::fixed << std::setprecision(1) << final_angle << "° from upright\n";
        std::cout << "Final energy: " << std::setprecision(4) << final_energy << "J (target: " << state.Edes << "J)\n";
        std::cout << "Final state: " << (state.state == 0 ? "IDLE" : state.state == 2 ? "SWINGUP" : 
                                         state.state == 4 ? "BALANCE" : "OTHER") << "\n";
        
        if (state.state == 4 && fabsf(final_angle) < 5.0f) {
            std::cout << "SUCCESS: Pendulum balanced upright!\n";
        } else if (state.state == 2 && final_energy > state.Edes * 0.8f) {
            std::cout << "PROGRESS: Swing-up building energy...\n";
        } else {
            std::cout << "NOTE: May need parameter tuning\n";
        }
    }
    
    void saveCSV(const std::string& filename = "simulation_data.csv") {
        std::ofstream file(filename);
        file << "time,angle_deg,omega,motor_cmd,energy,state\n";
        for (const auto& data : log_data) {
            file << std::get<0>(data) << "," << std::get<1>(data) << "," << std::get<2>(data) 
                 << "," << std::get<3>(data) << "," << std::get<4>(data) << "," << std::get<5>(data) << "\n";
        }
        std::cout << "Data saved to " << filename << " (" << log_data.size() << " data points)\n";
    }
    
    void setControlState(int new_state) {
        state.state = new_state;
        if (new_state == 2) { // SWINGUP
            std::cout << "Starting swing-up mode...\n";
        } else if (new_state == 0) { // IDLE
            std::cout << "Setting to idle mode...\n";
        }
    }
    
    void tuneParameters(float kp, float kd, float ki) {
        params.Kp = kp; params.Kd = kd; params.Ki = ki;
        std::cout << "Updated gains: Kp=" << kp << ", Kd=" << kd << ", Ki=" << ki << "\n";
    }
    
    void addDisturbance(float angle_kick = 0.2f, float velocity_kick = 1.0f) {
        physics.reset(physics.getTheta() + angle_kick, physics.getOmega() + velocity_kick);
        std::cout << "Applied disturbance: +" << (angle_kick * 180/M_PI) << "° angle, +" << velocity_kick << " rad/s velocity\n";
    }
    
    void reset() {
        physics.reset(M_PI + 0.1f, 0); // Start slightly off bottom
        ctrl_init(&params, &state);
        state.Edes = params.m * 9.81f * (params.L * 0.5f) * 2.2f;
        sim_time = 0;
        log_data.clear();
        std::cout << "Simulation reset\n";
    }
};

void printHelp() {
    std::cout << "\n=== Pendulum Control Simulator ===\n";
    std::cout << "A console-based simulator for testing pendulum control algorithms\n\n";
    std::cout << "Usage: ./simple_simulator [options]\n\n";
    std::cout << "Options:\n";
    std::cout << "  --help, -h          Show this help\n";
    std::cout << "  --duration, -d <s>  Simulation duration (default: 10s)\n";
    std::cout << "  --verbose, -v       Show detailed output\n";
    std::cout << "  --swing             Start in swing-up mode\n";
    std::cout << "  --kp <value>        Set proportional gain (default: 5.0)\n";
    std::cout << "  --kd <value>        Set derivative gain (default: 1.0)\n";
    std::cout << "  --ki <value>        Set integral gain (default: 0.1)\n";
    std::cout << "  --disturbance       Add initial disturbance\n";
    std::cout << "  --save <file>       Save data to CSV file\n\n";
    std::cout << "Examples:\n";
    std::cout << "  ./simple_simulator --swing --duration 15 --verbose\n";
    std::cout << "  ./simple_simulator --kp 10 --kd 2 --disturbance\n";
    std::cout << "  ./simple_simulator --swing --save results.csv\n\n";
}

int main(int argc, char* argv[]) {
    float duration = 10.0f;
    bool verbose = false;
    bool swing_mode = false;
    bool add_disturbance = false;
    float kp = 5.0f, kd = 1.0f, ki = 0.1f;
    std::string save_file = "";
    
    // Parse command line arguments
    for (int i = 1; i < argc; i++) {
        std::string arg = argv[i];
        
        if (arg == "--help" || arg == "-h") {
            printHelp();
            return 0;
        } else if (arg == "--duration" || arg == "-d") {
            if (i + 1 < argc) duration = std::stof(argv[++i]);
        } else if (arg == "--verbose" || arg == "-v") {
            verbose = true;
        } else if (arg == "--swing") {
            swing_mode = true;
        } else if (arg == "--kp") {
            if (i + 1 < argc) kp = std::stof(argv[++i]);
        } else if (arg == "--kd") {
            if (i + 1 < argc) kd = std::stof(argv[++i]);
        } else if (arg == "--ki") {
            if (i + 1 < argc) ki = std::stof(argv[++i]);
        } else if (arg == "--disturbance") {
            add_disturbance = true;
        } else if (arg == "--save") {
            if (i + 1 < argc) save_file = argv[++i];
        } else {
            std::cout << "Unknown option: " << arg << "\n";
            std::cout << "Use --help for usage information\n";
            return 1;
        }
    }
    
    try {
        ConsoleSimulator sim;
        
        // Apply command line settings
        sim.tuneParameters(kp, kd, ki);
        
        if (swing_mode) {
            sim.setControlState(2); // SWINGUP
        }
        
        if (add_disturbance) {
            sim.addDisturbance();
        }
        
        // Run simulation
        sim.run(duration, verbose);
        
        // Save data if requested
        if (!save_file.empty()) {
            sim.saveCSV(save_file);
        }
        
        std::cout << "\nPress Enter to exit...";
        std::cin.get();
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}