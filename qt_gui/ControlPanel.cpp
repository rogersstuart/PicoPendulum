#include "ControlPanel.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QFormLayout>
#include <QGroupBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QProgressBar>
#include <cmath>

// C headers must be included with C linkage to avoid name mangling.  We
// forward declare in the header file and include here.
extern "C" {
#include "embedded/drv8833.h"
#include "embedded/motor_protection.h"
}

ControlPanel::ControlPanel(QWidget *parent)
    : QWidget(parent)
{
    // Set up a vertical layout for the entire panel
    auto *mainLayout = new QVBoxLayout(this);
    setLayout(mainLayout);

    // First row: control buttons
    auto *buttonLayout = new QHBoxLayout();
    m_pauseButton = new QPushButton(tr("Pause"), this);
    m_resetButton = new QPushButton(tr("Reset"), this);
    m_startSwingButton = new QPushButton(tr("Start Swing-Up"), this);
    buttonLayout->addWidget(m_pauseButton);
    buttonLayout->addWidget(m_resetButton);
    buttonLayout->addWidget(m_startSwingButton);
    mainLayout->addLayout(buttonLayout);

    // Connect button signals to internal slots which will emit external signals
    connect(m_pauseButton, &QPushButton::clicked, this, &ControlPanel::onPauseResume);
    connect(m_resetButton, &QPushButton::clicked, this, &ControlPanel::onReset);
    connect(m_startSwingButton, &QPushButton::clicked, this, &ControlPanel::onStartSwing);

    // Small spacer between sections
    mainLayout->addSpacing(6);

    // Status display: simulation time, angle, velocity, command, energy, etc.
    auto *statusLayout = new QFormLayout();
    m_timeLabel     = new QLabel("0.00 s", this);
    m_angleLabel    = new QLabel("0.00 rad", this);
    m_velocityLabel = new QLabel("0.00 rad/s", this);
    m_commandLabel  = new QLabel("0.00", this);
    m_energyLabel   = new QLabel("0.00 J", this);
    m_energyErrLabel= new QLabel("0.00 J", this);
    m_stateLabel    = new QLabel("IDLE", this);
    statusLayout->addRow(tr("Time:"), m_timeLabel);
    statusLayout->addRow(tr("Angle (θ_u):"), m_angleLabel);
    statusLayout->addRow(tr("Angular vel (ω):"), m_velocityLabel);
    statusLayout->addRow(tr("Motor command (u):"), m_commandLabel);
    statusLayout->addRow(tr("Energy (E):"), m_energyLabel);
    statusLayout->addRow(tr("Energy error (E-Eₙ):"), m_energyErrLabel);
    statusLayout->addRow(tr("State:"), m_stateLabel);
    mainLayout->addLayout(statusLayout);

    // Display for adaptive mass and friction estimates
    auto *massLayout = new QFormLayout();
    m_massLabel = new QLabel("0.00 g", this);
    m_frictionLabel = new QLabel("0.00000", this);
    massLayout->addRow(tr("Mass estimate (m̂):"), m_massLabel);
    massLayout->addRow(tr("Friction estimate (b̂_vis):"), m_frictionLabel);
    mainLayout->addLayout(massLayout);

    // Motor protection status: thermal utilisation and emergency brake
    auto *protLayout = new QFormLayout();
    m_protProgress = new QProgressBar(this);
    m_protProgress->setRange(0, 100);
    m_protProgress->setFormat("%p%");
    m_emergencyProgress = new QProgressBar(this);
    m_emergencyProgress->setRange(0, 100);
    m_emergencyProgress->setFormat("%p%");
    m_protStatus = new QLabel(tr("Normal"), this);
    protLayout->addRow(tr("Thermal Utilisation:"), m_protProgress);
    protLayout->addRow(tr("Emergency Utilisation:"), m_emergencyProgress);
    protLayout->addRow(tr("Protection Status:"), m_protStatus);
    mainLayout->addLayout(protLayout);

    // Data logging controls: collapsed group box that can be shown/hidden
    m_loggingGroup = new QGroupBox(tr("Data Logging"), this);
    auto *logLayout = new QVBoxLayout();
    m_loggingEnabled = new QCheckBox(tr("Enable logging"), this);
    m_exportButton = new QPushButton(tr("Export CSV"), this);
    logLayout->addWidget(m_loggingEnabled);
    logLayout->addWidget(m_exportButton);
    m_loggingGroup->setLayout(logLayout);
    mainLayout->addWidget(m_loggingGroup);

    // Stretch at end to push content to the top
    mainLayout->addStretch(1);
}

void ControlPanel::updateDisplay(double simTime,
                                 const ctrl_state_t &state,
                                 const ctrl_params_t &params,
                                 const drv8833_t &driver,
                                 float energy, float thermalEnergy,
                                 float thermalCap, float emergencyCap)
{
    // Basic state information
    m_timeLabel->setText(QString::asprintf("%.2f s", simTime));
    m_angleLabel->setText(QString::asprintf("%.3f rad (%.1f°)", state.theta_u,
                                             state.theta_u * 180.0f / M_PI));
    m_velocityLabel->setText(QString::asprintf("%.3f rad/s", state.omega));
    m_commandLabel->setText(QString::asprintf("%.3f", state.u));
    m_energyLabel->setText(QString::asprintf("%.3f J", state.E));
    m_energyErrLabel->setText(QString::asprintf("%.3f J", state.E - state.Edes));

    // Controller state name lookup
    static const char *stateNames[] = {"IDLE", "CALIB", "SWINGUP", "CATCH", "BALANCE", "FAULT"};
    int idx = (state.state >= 0 && state.state <= 5) ? state.state : 0;
    m_stateLabel->setText(stateNames[idx]);

    // Adaptive mass and friction estimates
    m_massLabel->setText(QString::asprintf("%.2f g", params.m * 1000.0f));
    m_frictionLabel->setText(QString::asprintf("%.5f", params.b_vis));

    // Protection utilisation: use driver API to query how loaded the motor is.
    float utilisation = drv8833_get_protection_utilization((drv8833_t *)&driver);
    utilisation = fminf(fmaxf(utilisation, 0.0f), 1.0f);
    m_protProgress->setValue(static_cast<int>(utilisation * 100.0f));

    // Emergency utilisation relative to energy caps
    float emergencyUtil = thermalEnergy / emergencyCap;
    emergencyUtil = fminf(fmaxf(emergencyUtil, 0.0f), 1.0f);
    m_emergencyProgress->setValue(static_cast<int>(emergencyUtil * 100.0f));

    // Determine protection status and update label colour
    bool thermalHalt = drv8833_is_thermal_halted((drv8833_t *)&driver);
    bool emergency   = drv8833_is_emergency_brake_active((drv8833_t *)&driver);
    bool active      = drv8833_is_protection_active((drv8833_t *)&driver);
    if (emergency) {
        m_protStatus->setText(tr("EMERGENCY BRAKE"));
        m_protStatus->setStyleSheet("color: red;");
    } else if (thermalHalt) {
        m_protStatus->setText(tr("THERMAL HALT"));
        m_protStatus->setStyleSheet("color: magenta;");
    } else if (active) {
        m_protStatus->setText(tr("PROTECTION ACTIVE"));
        m_protStatus->setStyleSheet("color: orange;");
    } else {
        m_protStatus->setText(tr("Normal"));
        m_protStatus->setStyleSheet("color: green;");
    }
}

void ControlPanel::onPauseResume()
{
    // Flip between pause and resume text.  When the button is clicked,
    // we emit a signal indicating the new paused state.  The caller
    // should manage the actual state variable.
    bool currentlyPaused = (m_pauseButton->text() == tr("Resume"));
    bool newPaused = !currentlyPaused;
    emit pauseToggled(newPaused);
    m_pauseButton->setText(newPaused ? tr("Resume") : tr("Pause"));
}

void ControlPanel::onReset()
{
    emit resetRequested();
    // Reset pause button text to default state
    m_pauseButton->setText(tr("Pause"));
}

void ControlPanel::onStartSwing()
{
    emit startSwingRequested();
}