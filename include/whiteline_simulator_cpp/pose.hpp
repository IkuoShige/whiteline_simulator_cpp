#ifndef WHITELINE_SIMULATOR_CPP__POSE_HPP_
#define WHITELINE_SIMULATOR_CPP__POSE_HPP_

struct Pose {
    double x;      // x position in meters
    double y;      // y position in meters  
    double theta;  // orientation in radians

    Pose() : x(0.0), y(0.0), theta(0.0) {}
    Pose(double x_, double y_, double theta_) : x(x_), y(y_), theta(theta_) {}
};

#endif  // WHITELINE_SIMULATOR_CPP__POSE_HPP_ 