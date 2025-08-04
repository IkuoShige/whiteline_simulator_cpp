#ifndef WHITELINE_SIMULATOR_CPP__POSE_HPP_
#define WHITELINE_SIMULATOR_CPP__POSE_HPP_

#include <cmath>

struct Pose {
    double x;      // x position in meters
    double y;      // y position in meters  
    double theta;  // orientation in radians

    Pose() : x(0.0), y(0.0), theta(0.0) {}
    Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

/**
 * Normalize angle to [-π, π] range
 * @param angle Input angle in radians
 * @return Normalized angle in [-π, π] range
 */
inline double toPInPI(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

/**
 * Coordinate transformation function (compatible with robocup_demo_shige brain)
 * Transforms coordinates from source frame to target frame
 * @param xs, ys, thetas Source coordinate (x, y, theta)
 * @param xst, yst, thetast Transformation (x, y, theta) 
 * @param xt, yt, thetat Output transformed coordinate (x, y, theta)
 */
inline void transCoord(const double &xs, const double &ys, const double &thetas, 
                      const double &xst, const double &yst, const double &thetast, 
                      double &xt, double &yt, double &thetat) {
    thetat = toPInPI(thetas + thetast);
    xt = xst + xs * cos(thetast) - ys * sin(thetast);
    yt = yst + xs * sin(thetast) + ys * cos(thetast);
}

#endif  // WHITELINE_SIMULATOR_CPP__POSE_HPP_ 