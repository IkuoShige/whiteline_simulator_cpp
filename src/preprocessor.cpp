#include "whiteline_simulator_cpp/preprocessor.hpp"
#include <random>
#include <algorithm>
#include <cmath>
#include <iomanip>

void Preprocessor::load_white_line_points(const std::string& filename, std::vector<Point2f>& wlpos) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for reading: " << filename << std::endl;
        return;
    }

    wlpos.clear();
    for (std::string line; std::getline(file, line);) {
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            continue;
        }
        
        std::istringstream iss(line);
        double x, y;
        if (!(iss >> x >> y)) {
            std::cerr << "Error reading line: " << line << std::endl;
            continue;
        }
        wlpos.emplace_back(static_cast<float>(x), static_cast<float>(y));
    }
    
    std::cout << "Loaded " << wlpos.size() << " white line points from " << filename << std::endl;
}

std::vector<Point2f> Preprocessor::generate_soccer_field_lines(const SoccerFieldDimensions& dimensions, double point_spacing) {
    std::vector<Point2f> field_lines;
    
    // Field boundaries (14m x 9m, centered at origin)
    double half_length = dimensions.field_length / 2.0;
    double half_width = dimensions.field_width / 2.0;
    
    // Field boundary lines
    auto boundary_lines = generate_rectangle_points(Point2f(0, 0), dimensions.field_length, dimensions.field_width, point_spacing);
    field_lines.insert(field_lines.end(), boundary_lines.begin(), boundary_lines.end());
    
    // Center line (dividing the field lengthwise)
    auto center_line = generate_line_points(Point2f(0, -half_width), Point2f(0, half_width), point_spacing);
    field_lines.insert(field_lines.end(), center_line.begin(), center_line.end());
    
    // Center circle (diameter 3m, radius 1.5m)
    double center_circle_radius = dimensions.centre_circle_diameter / 2.0;
    auto center_circle = generate_circle_points(Point2f(0, 0), center_circle_radius, point_spacing);
    field_lines.insert(field_lines.end(), center_circle.begin(), center_circle.end());
    
    // Goal areas (1m x 4m, both ends)
    // Left goal area
    auto left_goal_area = generate_rectangle_points(
        Point2f(-half_length + dimensions.goal_area_length / 2.0, 0), 
        dimensions.goal_area_length, 
        dimensions.goal_area_width, 
        point_spacing
    );
    field_lines.insert(field_lines.end(), left_goal_area.begin(), left_goal_area.end());
    
    // Right goal area
    auto right_goal_area = generate_rectangle_points(
        Point2f(half_length - dimensions.goal_area_length / 2.0, 0), 
        dimensions.goal_area_length, 
        dimensions.goal_area_width, 
        point_spacing
    );
    field_lines.insert(field_lines.end(), right_goal_area.begin(), right_goal_area.end());
    
    // Penalty areas (3m x 6m, both ends)
    // Left penalty area
    auto left_penalty_area = generate_rectangle_points(
        Point2f(-half_length + dimensions.penalty_area_length / 2.0, 0), 
        dimensions.penalty_area_length, 
        dimensions.penalty_area_width, 
        point_spacing
    );
    field_lines.insert(field_lines.end(), left_penalty_area.begin(), left_penalty_area.end());
    
    // Right penalty area
    auto right_penalty_area = generate_rectangle_points(
        Point2f(half_length - dimensions.penalty_area_length / 2.0, 0), 
        dimensions.penalty_area_length, 
        dimensions.penalty_area_width, 
        point_spacing
    );
    field_lines.insert(field_lines.end(), right_penalty_area.begin(), right_penalty_area.end());
    
    // Penalty marks (small circles at penalty mark distance)
    double penalty_mark_radius = 0.05;  // 5cm radius for penalty marks
    // Left penalty mark
    auto left_penalty_mark = generate_circle_points(
        Point2f(-half_length + dimensions.penalty_mark_distance, 0), 
        penalty_mark_radius, 
        point_spacing
    );
    field_lines.insert(field_lines.end(), left_penalty_mark.begin(), left_penalty_mark.end());
    
    // Right penalty mark
    auto right_penalty_mark = generate_circle_points(
        Point2f(half_length - dimensions.penalty_mark_distance, 0), 
        penalty_mark_radius, 
        point_spacing
    );
    field_lines.insert(field_lines.end(), right_penalty_mark.begin(), right_penalty_mark.end());
    
    // Goals (goal posts, 2.6m width, 0.6m depth)
    double goal_half_width = dimensions.goal_width / 2.0;
    
    // Left goal line (only the goal line on the field boundary)
    auto left_goal_line = generate_line_points(
        Point2f(-half_length, -goal_half_width), 
        Point2f(-half_length, goal_half_width), 
        point_spacing
    );
    field_lines.insert(field_lines.end(), left_goal_line.begin(), left_goal_line.end());
    
    // Right goal line (only the goal line on the field boundary)
    auto right_goal_line = generate_line_points(
        Point2f(half_length, -goal_half_width), 
        Point2f(half_length, goal_half_width), 
        point_spacing
    );
    field_lines.insert(field_lines.end(), right_goal_line.begin(), right_goal_line.end());
    
    std::cout << "Generated " << field_lines.size() << " soccer field white line points" << std::endl;
    return field_lines;
}

std::vector<Landmark> Preprocessor::generate_soccer_field_landmarks(const SoccerFieldDimensions& dimensions) {
    std::vector<Landmark> landmarks;
    
    double half_length = dimensions.field_length / 2.0;
    double half_width = dimensions.field_width / 2.0;
    double penalty_half_width = dimensions.penalty_area_width / 2.0;
    double goal_area_half_width = dimensions.goal_area_width / 2.0;
    double center_circle_radius = dimensions.centre_circle_diameter / 2.0;
    
    // Reference: robocup_demo_shige/src/brain/src/locator.cpp calcFieldMarkers()
    // 合計26個のランドマークを生成
    
    // 1. Xマーク（センターライン上）- 2個
    landmarks.emplace_back(Point2f(0.0, -center_circle_radius), LandmarkType::X_CROSS, 0.0f);
    landmarks.emplace_back(Point2f(0.0, center_circle_radius), LandmarkType::X_CROSS, 0.0f);
    
    // 2. ペナルティスポット - 2個
    landmarks.emplace_back(Point2f(half_length - dimensions.penalty_mark_distance, 0.0), LandmarkType::PENALTY_MARK, 0.0f);
    landmarks.emplace_back(Point2f(-half_length + dimensions.penalty_mark_distance, 0.0), LandmarkType::PENALTY_MARK, M_PI);
    
    // 3. サイドライン中央 - 2個
    landmarks.emplace_back(Point2f(0.0, half_width), LandmarkType::T_JUNCTION, M_PI);
    landmarks.emplace_back(Point2f(0.0, -half_width), LandmarkType::T_JUNCTION, 0.0f);
    
    // 4. ペナルティエリア - 8個
    // L-corners: 4個（ペナルティエリアの角）
    landmarks.emplace_back(Point2f(half_length - dimensions.penalty_area_length, penalty_half_width), LandmarkType::L_CORNER, M_PI);
    landmarks.emplace_back(Point2f(half_length - dimensions.penalty_area_length, -penalty_half_width), LandmarkType::L_CORNER, -M_PI/2);
    landmarks.emplace_back(Point2f(-half_length + dimensions.penalty_area_length, penalty_half_width), LandmarkType::L_CORNER, M_PI/2);
    landmarks.emplace_back(Point2f(-half_length + dimensions.penalty_area_length, -penalty_half_width), LandmarkType::L_CORNER, 0.0f);
    
    // T-junctions: 4個（ペナルティエリアの端）
    landmarks.emplace_back(Point2f(half_length, penalty_half_width), LandmarkType::T_JUNCTION, M_PI/2);
    landmarks.emplace_back(Point2f(half_length, -penalty_half_width), LandmarkType::T_JUNCTION, -M_PI/2);
    landmarks.emplace_back(Point2f(-half_length, penalty_half_width), LandmarkType::T_JUNCTION, M_PI/2);
    landmarks.emplace_back(Point2f(-half_length, -penalty_half_width), LandmarkType::T_JUNCTION, -M_PI/2);
    
    // 5. ゴールエリア - 8個
    // L-corners: 4個（ゴールエリアの角）
    landmarks.emplace_back(Point2f(half_length - dimensions.goal_area_length, goal_area_half_width), LandmarkType::L_CORNER, M_PI);
    landmarks.emplace_back(Point2f(half_length - dimensions.goal_area_length, -goal_area_half_width), LandmarkType::L_CORNER, -M_PI/2);
    landmarks.emplace_back(Point2f(-half_length + dimensions.goal_area_length, goal_area_half_width), LandmarkType::L_CORNER, M_PI/2);
    landmarks.emplace_back(Point2f(-half_length + dimensions.goal_area_length, -goal_area_half_width), LandmarkType::L_CORNER, 0.0f);
    
    // T-junctions: 4個（ゴールエリアの端）
    landmarks.emplace_back(Point2f(half_length, goal_area_half_width), LandmarkType::T_JUNCTION, M_PI/2);
    landmarks.emplace_back(Point2f(half_length, -goal_area_half_width), LandmarkType::T_JUNCTION, -M_PI/2);
    landmarks.emplace_back(Point2f(-half_length, goal_area_half_width), LandmarkType::T_JUNCTION, M_PI/2);
    landmarks.emplace_back(Point2f(-half_length, -goal_area_half_width), LandmarkType::T_JUNCTION, -M_PI/2);
    
    // 6. フィールドの四隅 - 4個
    landmarks.emplace_back(Point2f(half_length, half_width), LandmarkType::L_CORNER, M_PI);
    landmarks.emplace_back(Point2f(half_length, -half_width), LandmarkType::L_CORNER, -M_PI/2);
    landmarks.emplace_back(Point2f(-half_length, half_width), LandmarkType::L_CORNER, M_PI/2);
    landmarks.emplace_back(Point2f(-half_length, -half_width), LandmarkType::L_CORNER, 0.0f);
    
    std::cout << "Generated " << landmarks.size() << " soccer field landmarks (reference: robocup_demo_shige)" << std::endl;
    return landmarks;
}

void Preprocessor::save_white_line_points(const std::string& filename, const std::vector<Point2f>& wlpos) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    
    file << "# Soccer field white line points (AdultSize)\n";
    file << "# Generated automatically - x y coordinates in meters\n";
    file << "# Field dimensions: 14m x 9m\n";
    file << "# Format: x y\n\n";
    
    for (const auto& point : wlpos) {
        file << point.x << " " << point.y << "\n";
    }
    
    std::cout << "Saved " << wlpos.size() << " white line points to " << filename << std::endl;
}

void Preprocessor::save_landmarks(const std::string& filename, const std::vector<Landmark>& landmarks) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file for writing: " << filename << std::endl;
        return;
    }
    
    file << "# Soccer field landmarks (AdultSize)\n";
    file << "# Generated automatically - x y type orientation\n";
    file << "# Field dimensions: 14m x 9m\n";
    file << "# Format: x y type_id orientation_rad\n";
    file << "# Types: 0=T_JUNCTION, 1=L_CORNER, 2=X_CROSS, 3=PENALTY_MARK, 4=GOAL_POST\n\n";
    
    for (const auto& landmark : landmarks) {
        file << landmark.position.x << " " << landmark.position.y << " " 
             << static_cast<int>(landmark.type) << " " << landmark.orientation << "\n";
    }
    
    std::cout << "Saved " << landmarks.size() << " landmarks to " << filename << std::endl;
}

std::vector<Landmark> Preprocessor::simulate_landmarks(
    const std::vector<Landmark>& landmarks,
    const Pose& robot_pose,
    double fov,
    double max_distance) {
    
    std::vector<Landmark> visible_landmarks;
    double half_fov = fov / 2.0;

    for (const auto& landmark : landmarks) {
        double dx = landmark.position.x - robot_pose.x;
        double dy = landmark.position.y - robot_pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Skip landmarks beyond maximum detection distance
        if (distance > max_distance) {
            continue;
        }

        // Calculate angle relative to robot's orientation
        double angle = std::atan2(dy, dx) - robot_pose.theta;
        
        // Normalize angle to [-pi, pi]
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }

        // Check if landmark is within field of view
        if (std::abs(angle) <= half_fov) {
            // Transform landmark to robot-local coordinates
            Landmark local_landmark;
            
            // Transform position to robot-local coordinates (ROS convention: +x forward, +y left)
            // Apply inverse rotation: R^T * (p_global - p_robot)
            double cos_theta = std::cos(robot_pose.theta);
            double sin_theta = std::sin(robot_pose.theta);
            local_landmark.position.x = cos_theta * dx + sin_theta * dy;   // Forward distance
            local_landmark.position.y = -sin_theta * dx + cos_theta * dy;  // Left distance
            
            // Transform orientation to robot-local coordinates
            local_landmark.orientation = landmark.orientation - robot_pose.theta;
            
            // Normalize orientation to [-pi, pi]
            while (local_landmark.orientation > M_PI) {
                local_landmark.orientation -= 2 * M_PI;
            }
            while (local_landmark.orientation < -M_PI) {
                local_landmark.orientation += 2 * M_PI;
            }
            
            // Keep the same landmark type
            local_landmark.type = landmark.type;
            
            visible_landmarks.push_back(local_landmark);
        }
    }

    return visible_landmarks;
}

std::string Preprocessor::get_landmark_type_name(LandmarkType type) const {
    switch (type) {
        case LandmarkType::T_JUNCTION: return "T_JUNCTION";
        case LandmarkType::L_CORNER: return "L_CORNER";
        case LandmarkType::X_CROSS: return "X_CROSS";
        case LandmarkType::PENALTY_MARK: return "PENALTY_MARK";
        case LandmarkType::GOAL_POST: return "GOAL_POST";
        default: return "UNKNOWN";
    }
}

std::vector<Point2f> Preprocessor::generate_line_points(const Point2f& start, const Point2f& end, double spacing) {
    std::vector<Point2f> points;
    
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double distance = std::sqrt(dx * dx + dy * dy);
    
    if (distance < spacing) {
        points.push_back(start);
        return points;
    }
    
    int num_points = static_cast<int>(distance / spacing) + 1;
    
    for (int i = 0; i < num_points; ++i) {
        double t = static_cast<double>(i) / (num_points - 1);
        Point2f point;
        point.x = start.x + t * dx;
        point.y = start.y + t * dy;
        points.push_back(point);
    }
    
    return points;
}

std::vector<Point2f> Preprocessor::generate_circle_points(const Point2f& center, double radius, double spacing) {
    std::vector<Point2f> points;
    
    double circumference = 2 * M_PI * radius;
    int num_points = static_cast<int>(circumference / spacing);
    
    for (int i = 0; i < num_points; ++i) {
        double angle = 2 * M_PI * i / num_points;
        Point2f point;
        point.x = center.x + radius * std::cos(angle);
        point.y = center.y + radius * std::sin(angle);
        points.push_back(point);
    }
    
    return points;
}

std::vector<Point2f> Preprocessor::generate_rectangle_points(const Point2f& center, double width, double height, double spacing) {
    std::vector<Point2f> points;
    
    double half_width = width / 2.0;
    double half_height = height / 2.0;
    
    // Top edge
    auto top_edge = generate_line_points(
        Point2f(center.x - half_width, center.y + half_height),
        Point2f(center.x + half_width, center.y + half_height),
        spacing
    );
    points.insert(points.end(), top_edge.begin(), top_edge.end());
    
    // Right edge
    auto right_edge = generate_line_points(
        Point2f(center.x + half_width, center.y + half_height),
        Point2f(center.x + half_width, center.y - half_height),
        spacing
    );
    points.insert(points.end(), right_edge.begin(), right_edge.end());
    
    // Bottom edge
    auto bottom_edge = generate_line_points(
        Point2f(center.x + half_width, center.y - half_height),
        Point2f(center.x - half_width, center.y - half_height),
        spacing
    );
    points.insert(points.end(), bottom_edge.begin(), bottom_edge.end());
    
    // Left edge
    auto left_edge = generate_line_points(
        Point2f(center.x - half_width, center.y - half_height),
        Point2f(center.x - half_width, center.y + half_height),
        spacing
    );
    points.insert(points.end(), left_edge.begin(), left_edge.end());
    
    return points;
}

std::vector<Point2f> Preprocessor::simulate_white_line(
    const std::vector<Point2f>& wlpos, 
    const Pose& robot_pose, 
    double fov, 
    double max_distance) {
    
    std::vector<Point2f> simulated_points;
    double half_fov = fov / 2.0;
    int total_points = 0;
    int distance_filtered = 0;
    int fov_filtered = 0;

    for (const auto& point : wlpos) {
        total_points++;
        double dx = point.x - robot_pose.x;
        double dy = point.y - robot_pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Skip points beyond maximum detection distance
        if (distance > max_distance) {
            distance_filtered++;
            continue;
        }

        // Calculate angle relative to robot's orientation
        double angle = std::atan2(dy, dx) - robot_pose.theta;
        
        // Normalize angle to [-pi, pi]
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }

        // Check if point is within field of view
        if (std::abs(angle) <= half_fov) {
            // Transform point to robot-local coordinates (ROS convention: +x forward, +y left)
            // Apply inverse rotation: R^T * (p_global - p_robot)
            Point2f local_point;
            double cos_theta = std::cos(robot_pose.theta);
            double sin_theta = std::sin(robot_pose.theta);
            local_point.x = cos_theta * dx + sin_theta * dy;   // Forward distance
            local_point.y = -sin_theta * dx + cos_theta * dy;  // Left distance
            
            simulated_points.push_back(local_point);
        } else {
            fov_filtered++;
        }
    }

    // Debug: Show sample points with coordinate transformations
    if (!simulated_points.empty()) {
        std::cout << "\n=== WHITE LINE DETECTION DEBUG ===" << std::endl;
        std::cout << "Robot pose: (" << robot_pose.x << ", " << robot_pose.y << ", " << robot_pose.theta << " rad)" << std::endl;
        std::cout << "Robot heading: " << (robot_pose.theta * 180.0 / M_PI) << " degrees" << std::endl;
        std::cout << "Detected " << simulated_points.size() << " points in robot-local coordinates:" << std::endl;
        
        for (int i = 0; i < std::min(3, static_cast<int>(simulated_points.size())); ++i) {
            const auto& local_point = simulated_points[i];
            
            // Convert back to global coordinates for verification (ROS convention)
            double cos_theta_fwd = std::cos(robot_pose.theta);
            double sin_theta_fwd = std::sin(robot_pose.theta);
            double global_x = robot_pose.x + local_point.x * cos_theta_fwd - local_point.y * sin_theta_fwd;
            double global_y = robot_pose.y + local_point.x * sin_theta_fwd + local_point.y * cos_theta_fwd;
            
            // Calculate distance and angle from robot
            double distance = std::sqrt(local_point.x * local_point.x + local_point.y * local_point.y);
            double angle_rad = std::atan2(local_point.y, local_point.x);
            double angle_deg = angle_rad * 180.0 / M_PI;
            
            std::cout << "  [" << i << "] Robot-local: (" << std::fixed << std::setprecision(3) 
                      << local_point.x << ", " << local_point.y << ") "
                      << "dist=" << distance << "m, angle=" << angle_deg << "° "
                      << "-> Global: (" << global_x << ", " << global_y << ")" << std::endl;
        }
        std::cout << "===============================" << std::endl;
    }

    std::cout << "White line detection: total=" << total_points 
              << ", distance_filtered=" << distance_filtered 
              << ", fov_filtered=" << fov_filtered 
              << ", visible=" << simulated_points.size() << std::endl;

    return simulated_points;
} 

std::vector<Point2f> Preprocessor::generate_white_line_areas(
    const SoccerFieldDimensions& dimensions,
    double line_width,
    double point_density) {
    
    std::vector<Point2f> area_points;
    double half_length = dimensions.field_length / 2.0;
    double half_width = dimensions.field_width / 2.0;
    double center_circle_radius = dimensions.centre_circle_diameter / 2.0;

    double goal_half_width = dimensions.goal_width / 2.0;
    
    // Calculate points per line segment based on density
    double points_per_meter = std::sqrt(point_density);
    
    // 1. Field boundary lines (with width)
    std::vector<std::pair<Point2f, Point2f>> boundary_lines = {
        {{static_cast<float>(-half_length), static_cast<float>(-half_width)}, {static_cast<float>(half_length), static_cast<float>(-half_width)}},  // Bottom
        {{static_cast<float>(-half_length), static_cast<float>(half_width)}, {static_cast<float>(half_length), static_cast<float>(half_width)}},    // Top
        {{static_cast<float>(-half_length), static_cast<float>(-half_width)}, {static_cast<float>(-half_length), static_cast<float>(half_width)}},  // Left
        {{static_cast<float>(half_length), static_cast<float>(-half_width)}, {static_cast<float>(half_length), static_cast<float>(half_width)}}     // Right
    };
    
    for (const auto& line : boundary_lines) {
        auto line_points = generate_line_area_points(line.first, line.second, line_width, points_per_meter);
        area_points.insert(area_points.end(), line_points.begin(), line_points.end());
        std::cout << "Generated " << line_points.size() << " boundary line points" << std::endl;
    }
    
    // 2. Center line
    auto center_line_points = generate_line_area_points(
        Point2f(0, -half_width), Point2f(0, half_width), line_width, points_per_meter);
    area_points.insert(area_points.end(), center_line_points.begin(), center_line_points.end());
    std::cout << "Generated " << center_line_points.size() << " center line points" << std::endl;
    
    // 3. Center circle
    auto center_circle_points = generate_circle_area_points(
        Point2f(0, 0), center_circle_radius, line_width, points_per_meter);
    area_points.insert(area_points.end(), center_circle_points.begin(), center_circle_points.end());
    std::cout << "Generated " << center_circle_points.size() << " center circle points" << std::endl;
    
    // 4. Goal areas
    auto left_goal_points = generate_rectangle_area_points(
        Point2f(-half_length + dimensions.goal_area_length / 2.0, 0),
        dimensions.goal_area_length, dimensions.goal_area_width, line_width, points_per_meter);
    area_points.insert(area_points.end(), left_goal_points.begin(), left_goal_points.end());
    std::cout << "Generated " << left_goal_points.size() << " left goal area points" << std::endl;
    
    auto right_goal_points = generate_rectangle_area_points(
        Point2f(half_length - dimensions.goal_area_length / 2.0, 0),
        dimensions.goal_area_length, dimensions.goal_area_width, line_width, points_per_meter);
    area_points.insert(area_points.end(), right_goal_points.begin(), right_goal_points.end());
    std::cout << "Generated " << right_goal_points.size() << " right goal area points" << std::endl;
    
    // 5. Penalty areas
    auto left_penalty_points = generate_rectangle_area_points(
        Point2f(-half_length + dimensions.penalty_area_length / 2.0, 0),
        dimensions.penalty_area_length, dimensions.penalty_area_width, line_width, points_per_meter);
    area_points.insert(area_points.end(), left_penalty_points.begin(), left_penalty_points.end());
    std::cout << "Generated " << left_penalty_points.size() << " left penalty area points" << std::endl;
    
    auto right_penalty_points = generate_rectangle_area_points(
        Point2f(half_length - dimensions.penalty_area_length / 2.0, 0),
        dimensions.penalty_area_length, dimensions.penalty_area_width, line_width, points_per_meter);
    area_points.insert(area_points.end(), right_penalty_points.begin(), right_penalty_points.end());
    std::cout << "Generated " << right_penalty_points.size() << " right penalty area points" << std::endl;
    
    // 6. Penalty marks (small circles)
    double penalty_mark_radius = 0.05;
    auto left_penalty_mark_points = generate_circle_area_points(
        Point2f(-half_length + dimensions.penalty_mark_distance, 0),
        penalty_mark_radius, line_width, points_per_meter);
    area_points.insert(area_points.end(), left_penalty_mark_points.begin(), left_penalty_mark_points.end());
    std::cout << "Generated " << left_penalty_mark_points.size() << " left penalty mark points" << std::endl;
    
    auto right_penalty_mark_points = generate_circle_area_points(
        Point2f(half_length - dimensions.penalty_mark_distance, 0),
        penalty_mark_radius, line_width, points_per_meter);
    area_points.insert(area_points.end(), right_penalty_mark_points.begin(), right_penalty_mark_points.end());
    std::cout << "Generated " << right_penalty_mark_points.size() << " right penalty mark points" << std::endl;
    
    // 7. Goal lines
    auto left_goal_line_points = generate_line_area_points(
        Point2f(-half_length, -goal_half_width), Point2f(-half_length, goal_half_width),
        line_width, points_per_meter);
    area_points.insert(area_points.end(), left_goal_line_points.begin(), left_goal_line_points.end());
    std::cout << "Generated " << left_goal_line_points.size() << " left goal line points" << std::endl;
    
    auto right_goal_line_points = generate_line_area_points(
        Point2f(half_length, -goal_half_width), Point2f(half_length, goal_half_width),
        line_width, points_per_meter);
    area_points.insert(area_points.end(), right_goal_line_points.begin(), right_goal_line_points.end());
    std::cout << "Generated " << right_goal_line_points.size() << " right goal line points" << std::endl;
    
    std::cout << "Generated " << area_points.size() << " white line area points" << std::endl;
    
    // Debug: Check coordinate ranges
    if (!area_points.empty()) {
        float min_x = area_points[0].x, max_x = area_points[0].x;
        float min_y = area_points[0].y, max_y = area_points[0].y;
        
        for (const auto& point : area_points) {
            min_x = std::min(min_x, point.x);
            max_x = std::max(max_x, point.x);
            min_y = std::min(min_y, point.y);
            max_y = std::max(max_y, point.y);
        }
        
        std::cout << "White line coordinate ranges: x=[" << min_x << ", " << max_x 
                  << "], y=[" << min_y << ", " << max_y << "]" << std::endl;
    }
    
    return area_points;
}

std::vector<Point2f> Preprocessor::simulate_camera_white_line_detection(
    const std::vector<Point2f>& wlpos,
    const Pose& robot_pose,
    const CameraSimulationParams& camera_params) {
    
    std::vector<Point2f> detected_points;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<> noise_dist(0.0, camera_params.noise_std);
    std::uniform_real_distribution<> prob_dist(0.0, 1.0);
    
    double half_fov = camera_params.fov / 2.0;
    
    // Statistics for debugging
    int total_points = 0;
    int distance_filtered = 0;
    int fov_filtered = 0;
    int probability_filtered = 0;
    
    for (const auto& point : wlpos) {
        total_points++;
        // Calculate distance and angle from robot
        double dx = point.x - robot_pose.x;
        double dy = point.y - robot_pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);
        
        // Skip points beyond maximum detection distance
        if (distance > camera_params.max_distance) {
            distance_filtered++;
            continue;
        }
        
        // Calculate angle relative to robot's orientation
        double angle = std::atan2(dy, dx) - robot_pose.theta;
        
        // Normalize angle to [-pi, pi]
        while (angle > M_PI) angle -= 2 * M_PI;
        while (angle < -M_PI) angle += 2 * M_PI;
        
        // Check if point is within field of view
        if (std::abs(angle) <= half_fov) {
            // Apply detection probability
            if (prob_dist(gen) <= camera_params.detection_probability) {
                // Transform point to robot-local coordinates (ROS convention: +x forward, +y left)
                Point2f local_point;
                double cos_theta = std::cos(robot_pose.theta);
                double sin_theta = std::sin(robot_pose.theta);
                local_point.x = cos_theta * dx + sin_theta * dy;   // Forward distance
                local_point.y = -sin_theta * dx + cos_theta * dy;  // Left distance
                
                // Add noise to the detected point in local coordinates
                Point2f noisy_local_point;
                noisy_local_point.x = local_point.x + noise_dist(gen);
                noisy_local_point.y = local_point.y + noise_dist(gen);
                
                detected_points.push_back(noisy_local_point);
            } else {
                probability_filtered++;
            }
        } else {
            fov_filtered++;
        }
    }
    
    // Debug: Show sample points with realistic camera simulation
    if (!detected_points.empty()) {
        std::cout << "\n=== REALISTIC CAMERA DETECTION DEBUG ===" << std::endl;
        std::cout << "Robot pose: (" << robot_pose.x << ", " << robot_pose.y << ", " << robot_pose.theta << " rad)" << std::endl;
        std::cout << "Robot heading: " << (robot_pose.theta * 180.0 / M_PI) << " degrees" << std::endl;
        std::cout << "Camera params: noise_std=" << camera_params.noise_std 
                  << "m, detection_prob=" << camera_params.detection_probability << std::endl;
        std::cout << "Realistic detection: total=" << total_points 
                  << ", distance_filtered=" << distance_filtered 
                  << ", fov_filtered=" << fov_filtered 
                  << ", probability_filtered=" << probability_filtered 
                  << ", detected=" << detected_points.size() << std::endl;
        
        for (int i = 0; i < std::min(3, static_cast<int>(detected_points.size())); ++i) {
            const auto& noisy_point = detected_points[i];
            
            // Convert back to global coordinates for verification (ROS convention)
            double cos_theta_fwd = std::cos(robot_pose.theta);
            double sin_theta_fwd = std::sin(robot_pose.theta);
            double global_x = robot_pose.x + noisy_point.x * cos_theta_fwd - noisy_point.y * sin_theta_fwd;
            double global_y = robot_pose.y + noisy_point.x * sin_theta_fwd + noisy_point.y * cos_theta_fwd;
            
            // Calculate distance and angle from robot
            double distance = std::sqrt(noisy_point.x * noisy_point.x + noisy_point.y * noisy_point.y);
            double angle_rad = std::atan2(noisy_point.y, noisy_point.x);
            double angle_deg = angle_rad * 180.0 / M_PI;
            
            std::cout << "  [" << i << "] Robot-local: (" << std::fixed << std::setprecision(3) 
                      << noisy_point.x << ", " << noisy_point.y << ") "
                      << "dist=" << distance << "m, angle=" << angle_deg << "° "
                      << "-> Global: (" << global_x << ", " << global_y << ") [with noise]" << std::endl;
        }
        std::cout << "==========================================" << std::endl;
    }
    
    return detected_points;
}

// Helper function to generate points along a line with width
std::vector<Point2f> Preprocessor::generate_line_area_points(
    const Point2f& start, const Point2f& end, double width, double points_per_meter) {
    
    std::vector<Point2f> points;
    double dx = end.x - start.x;
    double dy = end.y - start.y;
    double length = std::sqrt(dx * dx + dy * dy);
    
    if (length < 0.001) return points;
    
    // Normalize direction vector
    double dir_x = dx / length;
    double dir_y = dy / length;
    
    // Perpendicular vector for width
    double perp_x = -dir_y;
    double perp_y = dir_x;
    
    // Calculate number of points along the line
    int num_points_along = static_cast<int>(length * points_per_meter);
    int num_points_across = static_cast<int>(width * points_per_meter);
    
    // Ensure minimum number of points
    num_points_along = std::max(num_points_along, 1);
    num_points_across = std::max(num_points_across, 1);
    
    std::cout << "Line from (" << start.x << ", " << start.y << ") to (" << end.x << ", " << end.y 
              << "), length=" << length << ", width=" << width 
              << ", points_per_meter=" << points_per_meter
              << ", num_points_along=" << num_points_along 
              << ", num_points_across=" << num_points_across << std::endl;
    
    for (int i = 0; i <= num_points_along; ++i) {
        double t = static_cast<double>(i) / num_points_along;
        double center_x = start.x + t * dx;
        double center_y = start.y + t * dy;
        
        for (int j = 0; j <= num_points_across; ++j) {
            double w = (static_cast<double>(j) / num_points_across - 0.5) * width;
            Point2f point;
            point.x = center_x + w * perp_x;
            point.y = center_y + w * perp_y;
            points.push_back(point);
        }
    }
    
    std::cout << "Generated " << points.size() << " points for line" << std::endl;
    return points;
}

// Helper function to generate points in a circle area
std::vector<Point2f> Preprocessor::generate_circle_area_points(
    const Point2f& center, double radius, double width, double points_per_meter) {
    
    std::vector<Point2f> points;
    double outer_radius = radius + width / 2.0;
    double inner_radius = std::max(0.0, radius - width / 2.0);
    
    // Calculate area and number of points
    double area = M_PI * (outer_radius * outer_radius - inner_radius * inner_radius);
    int num_points = static_cast<int>(area * points_per_meter * points_per_meter);
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> radius_dist(inner_radius, outer_radius);
    std::uniform_real_distribution<> angle_dist(0.0, 2.0 * M_PI);
    
    for (int i = 0; i < num_points; ++i) {
        double r = radius_dist(gen);
        double angle = angle_dist(gen);
        
        Point2f point;
        point.x = center.x + r * std::cos(angle);
        point.y = center.y + r * std::sin(angle);
        points.push_back(point);
    }
    
    return points;
}

// Helper function to generate points in a rectangle area
std::vector<Point2f> Preprocessor::generate_rectangle_area_points(
    const Point2f& center, double width, double height, double line_width, double points_per_meter) {
    
    std::vector<Point2f> points;
    
    // Generate points for each edge of the rectangle
    double half_w = width / 2.0;
    double half_h = height / 2.0;
    
    // Top edge
    auto top_points = generate_line_area_points(
        Point2f(center.x - half_w, center.y + half_h),
        Point2f(center.x + half_w, center.y + half_h),
        line_width, points_per_meter);
    points.insert(points.end(), top_points.begin(), top_points.end());
    
    // Bottom edge
    auto bottom_points = generate_line_area_points(
        Point2f(center.x - half_w, center.y - half_h),
        Point2f(center.x + half_w, center.y - half_h),
        line_width, points_per_meter);
    points.insert(points.end(), bottom_points.begin(), bottom_points.end());
    
    // Left edge
    auto left_points = generate_line_area_points(
        Point2f(center.x - half_w, center.y - half_h),
        Point2f(center.x - half_w, center.y + half_h),
        line_width, points_per_meter);
    points.insert(points.end(), left_points.begin(), left_points.end());
    
    // Right edge
    auto right_points = generate_line_area_points(
        Point2f(center.x + half_w, center.y - half_h),
        Point2f(center.x + half_w, center.y + half_h),
        line_width, points_per_meter);
    points.insert(points.end(), right_points.begin(), right_points.end());
    
    return points;
} 