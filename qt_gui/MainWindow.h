#ifndef MAIN_WINDOW_H
#define MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QElapsedTimer>

#include "PendulumWidget.h"
#include "ControlPanel.h"
#include "enhanced_physics_simulation.hpp"

// Include additional C APIs used by the simulation
extern "C" {
#include "embedded/motor_protection.h"
#include "unified_virtual_encoder.h"
#include "adaptive_mass/mass_integration.h"
#include "common/energy_control.h"
}

#include "enhanced_physics_simulation.hpp"

/**
 * @brief Main window that hosts the pendulum simulation and control panel.
 *
 * This class sets up the GUI layout using a QSplitter, initialises the
 * control system and physics simulation, and runs the control loop on
 * a timer.  It also updates the display with current state and
 * adaptive mass/friction estimates.  User interactions are handled
 * through slots connected to the control panel signals.
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT
public:
    explicit MainWindow(QWidget *parent = nullptr);

private slots:
    /// Called periodically by the timer to update the simulation and UI.
    void onUpdate();
    /// Respond to pause/resume toggle from the control panel.
    void onPauseToggled(bool paused);
    /// Respond to reset requests from the control panel.
    void onResetRequested();
    /// Respond to start swing requests.
    void onStartSwingRequested();

private:
    /// Initialise the control system, physics simulation and related state.
    void initControlSystem();
    /// Reset the simulation to its initial state.
    void resetSimulation();
    /// Step the physics and controller forward by the given real time.
    void stepSimulation(float realDt);

    PendulumWidget *m_pendulumWidget;
    ControlPanel   *m_controlPanel;
    QTimer         *m_timer;
    QElapsedTimer   m_elapsed;
    qint64          m_lastTimeNs;
    float           m_simTime;
    bool            m_paused;

    // Simulation and control objects
    EnhancedPendulumPhysics m_physics;
    drv8833_t m_motorDriver;
    ctrl_state_t m_ctrlState;
    ctrl_params_t m_ctrlParams;
};

#endif // MAIN_WINDOW_H