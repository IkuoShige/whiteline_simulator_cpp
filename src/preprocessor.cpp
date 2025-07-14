#include "whiteline_simulator_cpp/preprocessor.hpp"

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
    double goal_half_width = dimensions.goal_width / 2.0;
    double center_circle_radius = dimensions.centre_circle_diameter / 2.0;
    
    // 1. L字型コーナー（フィールドの四隅）
    landmarks.emplace_back(Point2f(-half_length, -half_width), LandmarkType::L_CORNER, 0.0f);        // 左下
    landmarks.emplace_back(Point2f(-half_length, half_width), LandmarkType::L_CORNER, M_PI/2);       // 左上
    landmarks.emplace_back(Point2f(half_length, half_width), LandmarkType::L_CORNER, M_PI);          // 右上
    landmarks.emplace_back(Point2f(half_length, -half_width), LandmarkType::L_CORNER, -M_PI/2);      // 右下
    
    // 2. L字型コーナー（ペナルティエリアの角）
    // 左側ペナルティエリア
    landmarks.emplace_back(Point2f(-half_length + dimensions.penalty_area_length, -penalty_half_width), LandmarkType::L_CORNER, 0.0f);
    landmarks.emplace_back(Point2f(-half_length + dimensions.penalty_area_length, penalty_half_width), LandmarkType::L_CORNER, M_PI/2);
    
    // 右側ペナルティエリア
    landmarks.emplace_back(Point2f(half_length - dimensions.penalty_area_length, -penalty_half_width), LandmarkType::L_CORNER, -M_PI/2);
    landmarks.emplace_back(Point2f(half_length - dimensions.penalty_area_length, penalty_half_width), LandmarkType::L_CORNER, M_PI);
    
    // 3. L字型コーナー（ゴールエリアの角）
    // 左側ゴールエリア
    landmarks.emplace_back(Point2f(-half_length + dimensions.goal_area_length, -goal_area_half_width), LandmarkType::L_CORNER, 0.0f);
    landmarks.emplace_back(Point2f(-half_length + dimensions.goal_area_length, goal_area_half_width), LandmarkType::L_CORNER, M_PI/2);
    
    // 右側ゴールエリア
    landmarks.emplace_back(Point2f(half_length - dimensions.goal_area_length, -goal_area_half_width), LandmarkType::L_CORNER, -M_PI/2);
    landmarks.emplace_back(Point2f(half_length - dimensions.goal_area_length, goal_area_half_width), LandmarkType::L_CORNER, M_PI);
    
    // 4. T字型交点（中央線とタッチラインの交点）- 2個のみ
    landmarks.emplace_back(Point2f(0, -half_width), LandmarkType::T_JUNCTION, 0.0f);
    landmarks.emplace_back(Point2f(0, half_width), LandmarkType::T_JUNCTION, M_PI);
    
    // 5. T字型交点（ゴールラインとペナルティエリアの交点）- 4個
    landmarks.emplace_back(Point2f(-half_length, -penalty_half_width), LandmarkType::T_JUNCTION, -M_PI/2);
    landmarks.emplace_back(Point2f(-half_length, penalty_half_width), LandmarkType::T_JUNCTION, M_PI/2);
    landmarks.emplace_back(Point2f(half_length, -penalty_half_width), LandmarkType::T_JUNCTION, -M_PI/2);
    landmarks.emplace_back(Point2f(half_length, penalty_half_width), LandmarkType::T_JUNCTION, M_PI/2);
    
    // 6. T字型交点（ゴールラインとゴールエリアの交点）- 4個
    landmarks.emplace_back(Point2f(-half_length, -goal_area_half_width), LandmarkType::T_JUNCTION, -M_PI/2);
    landmarks.emplace_back(Point2f(-half_length, goal_area_half_width), LandmarkType::T_JUNCTION, M_PI/2);
    landmarks.emplace_back(Point2f(half_length, -goal_area_half_width), LandmarkType::T_JUNCTION, -M_PI/2);
    landmarks.emplace_back(Point2f(half_length, goal_area_half_width), LandmarkType::T_JUNCTION, M_PI/2);
    
    // 7. X字型交点（中央点）
    landmarks.emplace_back(Point2f(0, 0), LandmarkType::X_CROSS, 0.0f);
    
    // 8. X字型交点（センターサークルとセンターラインの交点）
    landmarks.emplace_back(Point2f(0, -center_circle_radius), LandmarkType::X_CROSS, 0.0f);
    landmarks.emplace_back(Point2f(0, center_circle_radius), LandmarkType::X_CROSS, 0.0f);
    
    // 9. ペナルティマーク
    landmarks.emplace_back(Point2f(-half_length + dimensions.penalty_mark_distance, 0), LandmarkType::PENALTY_MARK, 0.0f);
    landmarks.emplace_back(Point2f(half_length - dimensions.penalty_mark_distance, 0), LandmarkType::PENALTY_MARK, M_PI);
    
    // 10. ゴールポスト（ゴールラインとゴールの交点）
    landmarks.emplace_back(Point2f(-half_length, -goal_half_width), LandmarkType::GOAL_POST, 0.0f);
    landmarks.emplace_back(Point2f(-half_length, goal_half_width), LandmarkType::GOAL_POST, 0.0f);
    landmarks.emplace_back(Point2f(half_length, -goal_half_width), LandmarkType::GOAL_POST, M_PI);
    landmarks.emplace_back(Point2f(half_length, goal_half_width), LandmarkType::GOAL_POST, M_PI);
    
    std::cout << "Generated " << landmarks.size() << " soccer field landmarks" << std::endl;
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
            visible_landmarks.push_back(landmark);
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

    for (const auto& point : wlpos) {
        double dx = point.x - robot_pose.x;
        double dy = point.y - robot_pose.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Skip points beyond maximum detection distance
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

        // Check if point is within field of view
        if (std::abs(angle) <= half_fov) {
            simulated_points.push_back(point);
        }
    }

    return simulated_points;
} 