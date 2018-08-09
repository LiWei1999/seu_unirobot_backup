//
// Created by lcseu on 18-8-8.
//

#ifndef SEU_UNIROBOT_MATH_COMMON_HPP
#define SEU_UNIROBOT_MATH_COMMON_HPP

#include <eigen3/Eigen/Dense>
#include <cmath>

namespace robot_math
{
    #define epsilon 1E-6

    static double deg2rad(const double &x)
    {
        return x*M_PI/180.0;
    }

    static double rad2deg(const double &x)
    {
        return x*180.0/M_PI;
    }

    static bool is_zero(const double &x)
    {
        return fabs(x)<epsilon;
    }

    static int sign(const double &x)
    {
        return (x>=0)?1:-1;
    }

    void bound(const double &min, const double &max, double &x)
    {
        if(max<min) return;
        if(x<min) x = min;
        if(x>max) x = max;
    }

    Eigen::Matrix3d RotY(const double &deg)
    {
        double x = deg2rad(deg);
        Eigen::Matrix3d m;
        m<< cos(x),     0.0, sin(x),
                0.0,        1.0, 0.0,
                -sin(x),    0.0, cos(x);
        return m;
    }

    Eigen::Matrix3d RotX(const double &deg)
    {
        double x = deg2rad(deg);
        Eigen::Matrix3d m;
        m<< 1.0, 0.0,       0.0,
                0.0, cos(x),    -sin(x),
                0.0, sin(x),    cos(x);
        return m;
    }

    Eigen::Matrix3d RotZ(const double &deg)
    {
        double x = deg2rad(deg);
        Eigen::Matrix3d m;
        m<< cos(x), -sin(x),    0.0,
                sin(x), cos(x),     0.0,
                0.0,    0.0,        1.0;
        return m;
    }
}

#endif //SEU_UNIROBOT_MATH_COMMON_HPP
