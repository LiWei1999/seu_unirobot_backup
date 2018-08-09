#ifndef SEU_UNIROBOT_ROBOT_GL_HPP
#define SEU_UNIROBOT_ROBOT_GL_HPP

#include <QtWidgets>
#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <GL/glut.h>
#include <list>
#include <map>
#include "robot/humanoid.hpp"

class robot_gl: public QOpenGLWidget, protected QOpenGLFunctions
{
    Q_OBJECT
public:
    robot_gl();
    ~robot_gl();
    void turn_joint(const std::map<std::string, float> &degs);

protected:
    void initializeGL();
    void paintGL();

    virtual void mousePressEvent(QMouseEvent *event);
    virtual void mouseReleaseEvent(QMouseEvent *event);
    virtual void mouseMoveEvent(QMouseEvent *event);
    virtual void wheelEvent(QWheelEvent *event);
    virtual void keyPressEvent(QKeyEvent *event);
    virtual void keyReleaseEvent(QKeyEvent *event);

private:
    void init_3d_model();
    void draw_3d_sphere(float raduis);
    void draw_3d_cylinder(float length, float r_top, float r_button);
    void draw_3d_axe(float length);
    void draw_3d_ground(float width = 3.0f, float div = 0.25f);
    void draw_3d_bone(robot::bone_ptr b);
    void setUserView();

    std::map<std::string, float> joints_deg_;
    float m_Rotate_X, m_Rotate_Y;
    float m_transX, m_transY, m_transZ;
    int m_KeyState;
    QPoint m_mousePoint;
    bool m_IsPressed;
    GLUquadricObj   *quad_obj;
};

#endif // CRobotDrawGL_H
