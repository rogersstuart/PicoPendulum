#ifndef PENDULUM_WIDGET_H
#define PENDULUM_WIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>

// Forward declarations of control structures used for display
struct ctrl_state_t;
struct ctrl_params_t;

class EnhancedPendulumPhysics;

/**
 * @brief OpenGL widget for rendering the pendulum.
 *
 * This widget draws a simple 2D representation of the pendulum using
 * QPainter on top of an OpenGL surface.  It displays the rod, bob,
 * pivot, the motor command indicator, and the adaptive mass/friction
 * estimates.  The widget connects to the physics simulation and
 * controller state via pointers provided by setSimulationObjects().
 */
class PendulumWidget : public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    explicit PendulumWidget(QWidget *parent = nullptr);

    /**
     * Provide pointers to the physics and control structures.  The
     * widget does not own these objects; it simply reads their
     * members when drawing.  Pointers may be null until the
     * simulation is fully initialised.
     */
    void setSimulationObjects(EnhancedPendulumPhysics *phys,
                              ctrl_state_t *state,
                              ctrl_params_t *params);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintGL() override;

private:
    EnhancedPendulumPhysics *m_physics;
    ctrl_state_t *m_state;
    ctrl_params_t *m_params;
    float m_pixelScale;
};

#endif // PENDULUM_WIDGET_H