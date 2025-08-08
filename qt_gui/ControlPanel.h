#ifndef CONTROL_PANEL_H
#define CONTROL_PANEL_H

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QGroupBox>
#include <QCheckBox>
#include <QTimer>

// Forward declarations of control structures from the C code.  We include
// these via extern "C" in the implementation file to avoid name mangling.
struct ctrl_state_t;
struct ctrl_params_t;
struct drv8833_t;

/**
 * @brief Simple control panel for the pendulum simulator.
 *
 * This widget aggregates a number of status displays and buttons to
 * control the simulation.  It shows the current simulation time,
 * pendulum angle, angular velocity, motor command, energy and error,
 * controller state, and the adaptive mass and friction estimates.  It
 * also includes motor protection utilisation indicators and a group
 * box for data logging controls.  Signals are emitted when the user
 * toggles pause/resume, requests a reset, or starts a swing‑up.
 */
class ControlPanel : public QWidget
{
    Q_OBJECT
public:
    explicit ControlPanel(QWidget *parent = nullptr);

    /**
     * @brief Update the display with fresh simulation data.
     *
     * @param simTime Current simulation time in seconds.
     * @param state   Reference to the controller state structure.
     * @param params  Reference to the controller parameters.
     * @param driver  Reference to the motor driver simulation state.
     * @param energy  Current total mechanical energy of the pendulum.
     * @param thermalEnergy Energy used for thermal protection.
     * @param thermalCap Total thermal capacity of the motor (for normal protection).
     * @param emergencyCap Total energy at which the emergency brake engages.
     */
    void updateDisplay(double simTime,
                       const ctrl_state_t &state,
                       const ctrl_params_t &params,
                       const drv8833_t &driver,
                       float energy, float thermalEnergy,
                       float thermalCap, float emergencyCap);

signals:
    /// Emitted when the pause/resume button is clicked.  True = paused.
    void pauseToggled(bool paused);
    /// Emitted when the reset button is clicked.
    void resetRequested();
    /// Emitted when the start swing‑up button is clicked.
    void startSwingRequested();

private slots:
    void onPauseResume();
    void onReset();
    void onStartSwing();

private:
    QPushButton *m_pauseButton;
    QPushButton *m_resetButton;
    QPushButton *m_startSwingButton;

    QLabel *m_timeLabel;
    QLabel *m_angleLabel;
    QLabel *m_velocityLabel;
    QLabel *m_commandLabel;
    QLabel *m_energyLabel;
    QLabel *m_energyErrLabel;
    QLabel *m_stateLabel;

    QLabel *m_massLabel;
    QLabel *m_frictionLabel;

    QProgressBar *m_protProgress;
    QProgressBar *m_emergencyProgress;
    QLabel *m_protStatus;

    QGroupBox *m_loggingGroup;
    QCheckBox *m_loggingEnabled;
    QPushButton *m_exportButton;
};

#endif // CONTROL_PANEL_H