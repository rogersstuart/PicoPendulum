#include "MainWindow.h"

#include <QSplitter>
#include <QApplication>

#include <cmath>

// Ensure control constants are visible
extern "C" {
#include "embedded/control.h"
}

// Define state constants if not visible from C header
#ifndef ST_IDLE
enum ctrl_states { ST_IDLE=0, ST_CALIB, ST_SWINGUP, ST_CATCH, ST_BALANCE, ST_FAULT };
#endif

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      m_pendulumWidget(nullptr), m_controlPanel(nullptr),
      m_timer(nullptr), m_lastTimeNs(0), m_simTime(0.0f),
      m_paused(false)
{
    // Set a descriptive window title for the simulator.  This name
    // reflects that the application uses a Qt-based user interface.
    setWindowTitle(tr("Pendulum Simulator"));

    // Create a splitter to hold the pendulum view and the control panel.
    auto *splitter = new QSplitter(Qt::Horizontal, this);
    m_pendulumWidget = new PendulumWidget(splitter);
    m_controlPanel = new ControlPanel(splitter);
    splitter->addWidget(m_pendulumWidget);
    splitter->addWidget(m_controlPanel);
    // Ensure both panes initially occupy equal space
    splitter->setStretchFactor(0, 1);
    splitter->setStretchFactor(1, 1);
    setCentralWidget(splitter);

    // Connect control panel signals to our slots
    connect(m_controlPanel, &ControlPanel::pauseToggled,
            this, &MainWindow::onPauseToggled);
    connect(m_controlPanel, &ControlPanel::resetRequested,
            this, &MainWindow::onResetRequested);
    connect(m_controlPanel, &ControlPanel::startSwingRequested,
            this, &MainWindow::onStartSwingRequested);

    // Initialise simulation and control
    initControlSystem();
    // Provide pointers to simulation objects to the pendulum widget
    m_pendulumWidget->setSimulationObjects(&m_physics, &m_ctrlState, &m_ctrlParams);

    // Create timer for periodic updates (~60 Hz)
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &MainWindow::onUpdate);
    m_timer->start(16);

    // Start elapsed timer to measure real time between updates
    m_elapsed.start();
    m_lastTimeNs = m_elapsed.nsecsElapsed();
}

void MainWindow::initControlSystem()
{
    // Initialise control parameters and state using the embedded API
    ctrl_init(&m_ctrlParams, &m_ctrlState);
    // Default physical parameters: these match the hardware values used by the simulator
    m_ctrlParams.m     = 0.05f;     // 50 g
    m_ctrlParams.L     = 0.3048f;   // 12 inches in metres
    m_ctrlParams.Jm    = 1e-5f;     // Motor inertia
    m_ctrlParams.b_vis = 0.0002f;   // Viscous friction estimate
    m_ctrlParams.tau_ff= 0.001f;    // Coulomb friction
    m_ctrlParams.dt    = 1.0f / 1000.0f; // 1 kHz control update

    // Compute scaling factors based on the lever arm ratio.  These factors
    // mirror the logic used in the embedded controller so that the
    // simulation produces comparable torques.
    float default_lever_arm = 0.004f * 0.3048f;
    float current_lever_arm = m_ctrlParams.m * m_ctrlParams.L;
    float lever_ratio       = current_lever_arm / default_lever_arm;
    float motor_scaling_factor = std::sqrt(lever_ratio);
    float min_scaling_factor   = 0.05f;
    if (motor_scaling_factor < min_scaling_factor) motor_scaling_factor = min_scaling_factor;

    m_ctrlParams.tau_max      = 0.050f * motor_scaling_factor;
    m_ctrlParams.u_to_tau     = 0.05f * motor_scaling_factor;
    m_ctrlParams.k_energy     = ENERGY_PUMP_GAIN_MULTIPLIER * std::sqrt(lever_ratio);
    m_ctrlParams.swing_sat    = 1.0f;

    // PID gains for balance control
    m_ctrlParams.Kp      = 35.0f;
    m_ctrlParams.Kd      = 6.0f;
    m_ctrlParams.Ki      = 3.0f;
    m_ctrlParams.ui_max  = 1.0f;
    m_ctrlParams.balance_sat = 1.0f;

    // Catch thresholds
    m_ctrlParams.theta_catch = 20.0f * static_cast<float>(M_PI) / 180.0f;
    m_ctrlParams.omega_catch = OMEGA_CATCH_PC;

    // Initial energy target based on mass and length
    m_ctrlState.Edes = CALCULATE_ENERGY_TARGET(m_ctrlParams.m, m_ctrlParams.L);

    // Configure physics simulation
    m_physics.setParameters(m_ctrlParams.m, m_ctrlParams.L,
                            m_ctrlParams.b_vis, m_ctrlParams.tau_ff);
    m_physics.setTimestep(m_ctrlParams.dt);

    // Initialise motor driver simulation (pins arbitrary for PC build)
    drv8833_init(&m_motorDriver, 14, 15, 1000.0f);
    
    // Initialize all systems to consistent hanging-down position (π radians from upright)
    const float initial_theta_u = static_cast<float>(M_PI); // Hanging down
    
    // Initialize physics system to hanging down position
    m_physics.reset(initial_theta_u, 0.0f);
    
    // Initialize virtual encoder to match the physics initial position
    ve_init_global(initial_theta_u);
    // Initialise adaptive mass estimator
    adaptive_mass_init(&m_ctrlParams);

    // Reset controller state variables to match the initial physics state
    m_ctrlState.theta_u = initial_theta_u; // Start hanging downward
    m_ctrlState.omega   = 0.0f;
    m_ctrlState.state   = (decltype(m_ctrlState.state))ST_IDLE;
    m_ctrlState.ui      = 0.0f;
    m_ctrlState.E       = 0.0f;
    m_simTime = 0.0f;
}

void MainWindow::resetSimulation()
{
    initControlSystem();
    m_pendulumWidget->setSimulationObjects(&m_physics, &m_ctrlState, &m_ctrlParams);
    m_simTime = 0.0f;
}

void MainWindow::stepSimulation(float realDt)
{
    // Determine how many control steps to take based on real time
    float dt = m_ctrlParams.dt;
    int steps = static_cast<int>(realDt / dt);
    if (steps < 1) steps = 1;
    if (steps > 50) steps = 50;
    for (int i = 0; i < steps; ++i) {
        // Read angular velocity from physics
        m_ctrlState.omega = m_physics.getOmega();
        // Update virtual encoder with current velocity and time step
        ve_set_velocity_global(m_ctrlState.omega);
        ve_update_global(dt);
        
        // For debugging: use physics angle directly instead of virtual encoder
        // to eliminate encoder drift issues
        m_ctrlState.theta_u = m_physics.getTheta();
        
        // Maintain bottom referenced angle for completeness
        m_ctrlState.theta_b = m_physics.getThetaBottom();

        // Run controller step: this includes adaptive mass estimation via
        // the mass integration layer in ctrl_step().  It returns the
        // motor command in the range [-1,1].
        float u = ctrl_step(&m_ctrlParams, &m_ctrlState);
        
        // Safety limiter: prevent runaway control
        if (fabsf(u) > 1.0f) {
            printf("WARNING: Control output saturated: u=%.3f\n", u);
            u = (u > 0) ? 1.0f : -1.0f;
        }
        
        // Debug output for swing-up troubleshooting
        if (m_ctrlState.state == ST_SWINGUP && static_cast<int>(m_simTime * 1000) % 100 == 0) {
            printf("SwingUp: theta_u=%.3f, omega=%.3f, E=%.3f, Edes=%.3f, u=%.3f\n", 
                   m_ctrlState.theta_u, m_ctrlState.omega, m_ctrlState.E, m_ctrlState.Edes, u);
        }
        
        // Apply command through motor driver simulation
        drv8833_cmd(&m_motorDriver, u);
        drv8833_step_simulation(&m_motorDriver, dt, m_physics.getOmega());
        // Get resulting motor voltage and apply to physics
        float sim_voltage = 0.0f;
        float sim_current = 0.0f;
        float sim_torque  = 0.0f;
        drv8833_get_simulation_state(&m_motorDriver, &sim_voltage, &sim_current, &sim_torque);
        m_physics.setMotorVoltage(sim_voltage);
        // Advance physics simulation one time step
        m_physics.step();
        // Advance simulated time
        m_simTime += dt;
    }
}

void MainWindow::onUpdate()
{
    // Compute real elapsed time since last update
    qint64 now = m_elapsed.nsecsElapsed();
    float realDt = (now - m_lastTimeNs) / 1e9f;
    m_lastTimeNs = now;
    if (realDt < 0.0f) realDt = 0.0f;
    if (realDt > 0.1f) realDt = 0.1f;
    // Step simulation only if not paused
    if (!m_paused) {
        stepSimulation(realDt);
    }
    // Compute the current mechanical energy of the pendulum and the
    // energy consumed by the motor's thermal protection for display
    float energy        = m_physics.getEnergy();
    float thermalEnergy = drv8833_get_protection_energy_used(&m_motorDriver);
    // Update control panel with current values
    m_controlPanel->updateDisplay(m_simTime, m_ctrlState, m_ctrlParams,
                                  m_motorDriver, energy, thermalEnergy,
                                  MOTOR_THERMAL_CAPACITY, MOTOR_EMERGENCY_CAPACITY);
    // Update pendulum drawing
    m_pendulumWidget->update();
}

void MainWindow::onPauseToggled(bool paused)
{
    m_paused = paused;
}

void MainWindow::onResetRequested()
{
    resetSimulation();
}

void MainWindow::onStartSwingRequested()
{
    // Switch controller to swing‑up mode
    m_ctrlState.state = (decltype(m_ctrlState.state))ST_SWINGUP;
    m_ctrlState.ui    = 0.0f;
    // Recompute energy target for the current mass estimate
    m_ctrlState.Edes = CALCULATE_ENERGY_TARGET(m_ctrlParams.m, m_ctrlParams.L);
}