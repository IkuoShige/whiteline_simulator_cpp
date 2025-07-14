#ifndef WHITELINE_SIMULATOR_CPP__PREPROCESSOR_HPP_
#define WHITELINE_SIMULATOR_CPP__PREPROCESSOR_HPP_

#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <sstream>
#include <cmath>
#include "whiteline_simulator_cpp/pose.hpp"

// Simple Point2f structure to replace OpenCV dependency
struct Point2f {
    float x, y;
    Point2f() : x(0.0f), y(0.0f) {}
    Point2f(float x_, float y_) : x(x_), y(y_) {}
};

// Landmark types
enum class LandmarkType {
    T_JUNCTION = 0,      // T字型交点
    L_CORNER = 1,        // L字型交点（コーナー）
    X_CROSS = 2,         // X字型交点（十字）
    PENALTY_MARK = 3,    // ペナルティマーク
    GOAL_POST = 4        // ゴールポスト
};

// Landmark structure
struct Landmark {
    Point2f position;
    LandmarkType type;
    float orientation;  // ランドマークの向き（ラジアン）
    
    Landmark() : position(0.0f, 0.0f), type(LandmarkType::T_JUNCTION), orientation(0.0f) {}
    Landmark(const Point2f& pos, LandmarkType t, float ori = 0.0f) 
        : position(pos), type(t), orientation(ori) {}
};

// Soccer field dimensions structure
struct SoccerFieldDimensions {
    double field_length = 14.0;      // A: Field length
    double field_width = 9.0;        // B: Field width
    double goal_depth = 0.6;         // C: Goal depth
    double goal_width = 2.6;         // D: Goal width
    double goal_area_length = 1.0;   // E: Goal area length
    double goal_area_width = 4.0;    // F: Goal area width
    double penalty_mark_distance = 2.1;  // G: Penalty mark distance
    double centre_circle_diameter = 3.0;  // H: Centre circle diameter
    double border_strip_width = 1.0;     // I: Border strip width (minimum)
    double penalty_area_length = 3.0;    // J: Penalty area length
    double penalty_area_width = 6.0;     // K: Penalty area width
};

class Preprocessor {
public:
    Preprocessor() = default;
    ~Preprocessor() = default;

    /**
     * Load white line points from a text file
     * @param filename Path to the file containing white line points
     * @param wlpos Vector to store the loaded points
     */
    void load_white_line_points(const std::string& filename, std::vector<Point2f>& wlpos);

    /**
     * Generate soccer field white line points
     * @param dimensions Soccer field dimensions
     * @param point_spacing Spacing between points in meters
     * @return Vector of white line points
     */
    std::vector<Point2f> generate_soccer_field_lines(const SoccerFieldDimensions& dimensions, double point_spacing = 0.05);

    /**
     * Generate soccer field landmarks
     * @param dimensions Soccer field dimensions
     * @return Vector of landmarks
     */
    std::vector<Landmark> generate_soccer_field_landmarks(const SoccerFieldDimensions& dimensions);

    /**
     * Save white line points to a text file
     * @param filename Path to the output file
     * @param wlpos Vector of white line points to save
     */
    void save_white_line_points(const std::string& filename, const std::vector<Point2f>& wlpos);

    /**
     * Save landmarks to a text file
     * @param filename Path to the output file
     * @param landmarks Vector of landmarks to save
     */
    void save_landmarks(const std::string& filename, const std::vector<Landmark>& landmarks);

    /**
     * Simulate white line observation from robot's perspective
     * @param wlpos All white line points in the world
     * @param robot_pose Current robot pose
     * @param fov Field of view in radians
     * @param max_distance Maximum detection distance in meters
     * @return Vector of visible white line points
     */
    std::vector<Point2f> simulate_white_line(
        const std::vector<Point2f>& wlpos, 
        const Pose& robot_pose, 
        double fov, 
        double max_distance
    );

    /**
     * Simulate landmark observation from robot's perspective
     * @param landmarks All landmarks in the world
     * @param robot_pose Current robot pose
     * @param fov Field of view in radians
     * @param max_distance Maximum detection distance in meters
     * @return Vector of visible landmarks
     */
    std::vector<Landmark> simulate_landmarks(
        const std::vector<Landmark>& landmarks,
        const Pose& robot_pose,
        double fov,
        double max_distance
    );

    /**
     * Get landmark type name as string
     * @param type Landmark type
     * @return String representation of landmark type
     */
    std::string get_landmark_type_name(LandmarkType type) const;

private:
    /**
     * Generate points along a line segment
     * @param start Start point
     * @param end End point
     * @param spacing Point spacing
     * @return Vector of points along the line
     */
    std::vector<Point2f> generate_line_points(const Point2f& start, const Point2f& end, double spacing);
    
    /**
     * Generate points along a circle
     * @param center Center point
     * @param radius Circle radius
     * @param spacing Point spacing (angular)
     * @return Vector of points along the circle
     */
    std::vector<Point2f> generate_circle_points(const Point2f& center, double radius, double spacing);
    
    /**
     * Generate points for a rectangular area
     * @param center Center point
     * @param width Rectangle width
     * @param height Rectangle height
     * @param spacing Point spacing
     * @return Vector of points forming the rectangle
     */
    std::vector<Point2f> generate_rectangle_points(const Point2f& center, double width, double height, double spacing);
};

#endif  // WHITELINE_SIMULATOR_CPP__PREPROCESSOR_HPP_ 