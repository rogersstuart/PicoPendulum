#include "PendulumWidget.h"

#include <QPainter>
#include <cmath>

#include "enhanced_physics_simulation.hpp"

#include <QGuiApplication>

PendulumWidget::PendulumWidget(QWidget *parent)
    : QOpenGLWidget(parent), m_physics(nullptr), m_state(nullptr), m_params(nullptr), m_pixelScale(1.0f)
{
    // Request a full redraw whenever the widget is resized or the
    // device pixel ratio changes.  This is important on high DPI
    // displays where the pixel ratio can change dynamically when
    // moving the window between monitors.
    setUpdateBehavior(QOpenGLWidget::NoPartialUpdate);
}

void PendulumWidget::setSimulationObjects(EnhancedPendulumPhysics *phys,
                                          ctrl_state_t *state,
                                          ctrl_params_t *params)
{
    m_physics = phys;
    m_state = state;
    m_params = params;
}

void PendulumWidget::initializeGL()
{
    initializeOpenGLFunctions();
    glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
}

void PendulumWidget::resizeGL(int w, int h)
{
    Q_UNUSED(w);
    Q_UNUSED(h);
    // The viewport is automatically managed by Qt.
}

void PendulumWidget::paintGL()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (!m_physics || !m_state || !m_params) {
        return;
    }
    // Draw with QPainter on top of OpenGL.  This is sufficient for
    // simple 2D rendering and avoids the need for manual OpenGL
    // drawing commands.
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // Device pixel ratio for HiDPI support
    m_pixelScale = devicePixelRatioF();

    int w = width();
    int h = height();
    // Position the pivot somewhat towards the top of the widget and
    // centred horizontally.  Use a fraction of the widget height to
    // allocate space for the pendulum swing.
    float cx = w * 0.5f;
    float cy = h * 0.25f;

    // Map physical length to pixel length.  We scale by the minimum of
    // the width and height to preserve aspect ratio.
    float rod_px = 0.45f * std::min(w, h);
    
    // Get angle from physics (upright = 0, hanging down = PI)
    float theta_u = m_state->theta_u;
    
    // Calculate pendulum bob position
    // Screen coordinates: positive Y is down, positive X is right
    // theta_u = 0 means upright (pendulum pointing up, bob should be above pivot)
    // theta_u = π means hanging down (pendulum pointing down, bob should be below pivot)
    // We need to flip the Y component to match screen coordinates
    float bobX = cx + rod_px * sinf(theta_u);
    float bobY = cy - rod_px * cosf(theta_u);  // Note the minus sign!

    // Draw the rod
    QPen rodPen(QColor(200, 200, 200));
    rodPen.setWidthF(4.0f);
    painter.setPen(rodPen);
    painter.drawLine(QPointF(cx, cy), QPointF(bobX, bobY));

    // Draw the pivot as a small disc
    painter.setBrush(QColor(80, 80, 80));
    painter.setPen(Qt::NoPen);
    painter.drawEllipse(QPointF(cx, cy), 6.0f, 6.0f);

    // Draw the bob at the end of the rod
    painter.setBrush(QColor(150, 150, 250));
    float bob_radius = 10.0f;
    painter.drawEllipse(QPointF(bobX, bobY), bob_radius, bob_radius);

    // Draw a motor command indicator at the pivot.  Use a small arrow whose
    // length and colour reflect the current command.  Positive commands
    // point upward (red), negative downward (blue).
    float u = m_state->u;
    float arrow_len = 30.0f;
    float ax2 = cx;
    float ay2 = cy - arrow_len * u;
    QPen arrowPen(u >= 0 ? QColor(255, 80, 80) : QColor(80, 80, 255));
    arrowPen.setWidthF(3.0f);
    painter.setPen(arrowPen);
    painter.drawLine(QPointF(cx, cy), QPointF(ax2, ay2));

    // Display the adaptive estimates in the bottom left corner.  We
    // convert the mass to grams for readability.
    QString massText = QString("m̂ = %1 g, b̂_vis = %2")
                           .arg(m_params->m * 1000.0f, 0, 'f', 2)
                           .arg(m_params->b_vis, 0, 'f', 5);
    painter.setPen(QColor(220, 220, 220));
    painter.drawText(QPointF(10, height() - 10), massText);
}