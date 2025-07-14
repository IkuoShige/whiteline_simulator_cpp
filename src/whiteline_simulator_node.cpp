#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "whiteline_simulator_cpp/preprocessor.hpp"
#include "whiteline_simulator_cpp/pose.hpp"

using namespace std::chrono_literals;

class WhitelineSimulatorNode : public rclcpp::Node
{
public:
    WhitelineSimulatorNode()
    : Node("whiteline_simulator_node"), 
      preprocessor_(std::make_unique<Preprocessor>()),
      robot_pose_(0.0, 0.0, 0.0),  // Start at origin
      time_step_(0.0)
    {
        // Declare parameters
        this->declare_parameter<std::string>("whiteline_points_file", "soccer_field_points.txt");
        this->declare_parameter<std::string>("landmarks_file", "soccer_field_landmarks.txt");
        this->declare_parameter<bool>("generate_soccer_field", true);
        this->declare_parameter<double>("point_spacing", 0.05);
        this->declare_parameter<double>("fov", M_PI / 2);  // 90 degrees field of view
        this->declare_parameter<double>("max_distance", 8.0);  // 8 meters maximum detection distance
        this->declare_parameter<double>("robot_speed", 0.1);  // Robot movement speed
        this->declare_parameter<double>("field_length", 14.0);
        this->declare_parameter<double>("field_width", 9.0);
        
        // Publishers
        whiteline_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "whiteline_points", 10);
        landmarks_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "landmarks", 10);
        robot_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>(
            "robot_pose", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "whiteline_markers", 10);
        
        // Get parameters
        std::string whiteline_file = this->get_parameter("whiteline_points_file").as_string();
        std::string landmarks_file = this->get_parameter("landmarks_file").as_string();
        bool generate_soccer_field = this->get_parameter("generate_soccer_field").as_bool();
        double point_spacing = this->get_parameter("point_spacing").as_double();
        fov_ = this->get_parameter("fov").as_double();
        max_distance_ = this->get_parameter("max_distance").as_double();
        robot_speed_ = this->get_parameter("robot_speed").as_double();
        
        // Load or generate white line points and landmarks
        if (generate_soccer_field) {
            RCLCPP_INFO(this->get_logger(), "Generating soccer field white line points and landmarks...");
            
            // Set up soccer field dimensions
            SoccerFieldDimensions dimensions;
            dimensions.field_length = this->get_parameter("field_length").as_double();
            dimensions.field_width = this->get_parameter("field_width").as_double();
            
            // Generate soccer field lines and landmarks
            all_whiteline_points_ = preprocessor_->generate_soccer_field_lines(dimensions, point_spacing);
            all_landmarks_ = preprocessor_->generate_soccer_field_landmarks(dimensions);
            
            // Save to files for future use
            preprocessor_->save_white_line_points(whiteline_file, all_whiteline_points_);
            preprocessor_->save_landmarks(landmarks_file, all_landmarks_);
            
            RCLCPP_INFO(this->get_logger(), "Generated and saved %zu soccer field white line points", all_whiteline_points_.size());
            RCLCPP_INFO(this->get_logger(), "Generated and saved %zu soccer field landmarks", all_landmarks_.size());
        } else {
            RCLCPP_INFO(this->get_logger(), "Loading white line points from file: %s", whiteline_file.c_str());
            preprocessor_->load_white_line_points(whiteline_file, all_whiteline_points_);
            
            // Note: For now, we'll always generate landmarks since we don't have a load function
            // In a real implementation, you might want to add a load_landmarks function
            SoccerFieldDimensions dimensions;
            dimensions.field_length = this->get_parameter("field_length").as_double();
            dimensions.field_width = this->get_parameter("field_width").as_double();
            all_landmarks_ = preprocessor_->generate_soccer_field_landmarks(dimensions);
            RCLCPP_INFO(this->get_logger(), "Generated %zu soccer field landmarks", all_landmarks_.size());
        }
        
        // Timer for simulation updates
        timer_ = this->create_wall_timer(
            100ms, std::bind(&WhitelineSimulatorNode::timer_callback, this));
            
        RCLCPP_INFO(this->get_logger(), "Whiteline Simulator Node started");
        RCLCPP_INFO(this->get_logger(), "Loaded %zu white line points", all_whiteline_points_.size());
        RCLCPP_INFO(this->get_logger(), "Loaded %zu landmarks", all_landmarks_.size());
        RCLCPP_INFO(this->get_logger(), "FOV: %.2f rad (%.1f deg), Max distance: %.2f m", 
                    fov_, fov_ * 180.0 / M_PI, max_distance_);
    }

private:
    void timer_callback()
    {
        // Simulate robot movement (figure-8 pattern on the soccer field)
        time_step_ += robot_speed_;
        
        // Figure-8 pattern that covers different areas of the field
        double t = time_step_ * 0.2;
        robot_pose_.x = 4.0 * std::sin(t);
        robot_pose_.y = 2.0 * std::sin(2 * t);
        robot_pose_.theta = std::atan2(4.0 * std::cos(t), 4.0 * std::cos(2 * t));
        
        // Publish current robot pose
        publish_robot_pose();
        
        // Simulate white line detection
        std::vector<Point2f> visible_points = preprocessor_->simulate_white_line(
            all_whiteline_points_, robot_pose_, fov_, max_distance_);
        
        // Simulate landmark detection
        std::vector<Landmark> visible_landmarks = preprocessor_->simulate_landmarks(
            all_landmarks_, robot_pose_, fov_, max_distance_);
        
        // Publish visible white line points
        publish_whiteline_points(visible_points);
        
        // Publish visible landmarks
        publish_landmarks(visible_landmarks);
        
        // Publish markers for rviz2 visualization
        publish_markers(visible_points, all_whiteline_points_, visible_landmarks, all_landmarks_);
        
        // Log some information
        if (static_cast<int>(time_step_ * 10) % 50 == 0) {  // Log every 5 seconds
            RCLCPP_INFO(this->get_logger(), 
                "Robot at (%.2f, %.2f, %.2f rad), visible points: %zu, landmarks: %zu", 
                robot_pose_.x, robot_pose_.y, robot_pose_.theta, visible_points.size(), visible_landmarks.size());
        }
    }
    
    void publish_robot_pose()
    {
        auto pose_msg = geometry_msgs::msg::Pose2D();
        pose_msg.x = robot_pose_.x;
        pose_msg.y = robot_pose_.y;
        pose_msg.theta = robot_pose_.theta;
        robot_pose_publisher_->publish(pose_msg);
    }
    
    void publish_whiteline_points(const std::vector<Point2f>& points)
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        
        // Format: [x1, y1, x2, y2, x3, y3, ...]
        msg.data.clear();
        msg.data.reserve(points.size() * 2);
        
        for (const auto& point : points) {
            msg.data.push_back(point.x);
            msg.data.push_back(point.y);
        }
        
        // Set layout information
        msg.layout.dim.resize(2);
        msg.layout.dim[0].label = "points";
        msg.layout.dim[0].size = points.size();
        msg.layout.dim[0].stride = points.size() * 2;
        msg.layout.dim[1].label = "coordinates";
        msg.layout.dim[1].size = 2;
        msg.layout.dim[1].stride = 2;
        msg.layout.data_offset = 0;
        
        whiteline_publisher_->publish(msg);
    }
    
    void publish_landmarks(const std::vector<Landmark>& landmarks)
    {
        auto msg = std_msgs::msg::Float32MultiArray();
        
        // Format: [x1, y1, type1, orientation1, x2, y2, type2, orientation2, ...]
        msg.data.clear();
        msg.data.reserve(landmarks.size() * 4);
        
        for (const auto& landmark : landmarks) {
            msg.data.push_back(landmark.position.x);
            msg.data.push_back(landmark.position.y);
            msg.data.push_back(static_cast<float>(landmark.type));
            msg.data.push_back(landmark.orientation);
        }
        
        // Set layout information
        msg.layout.dim.resize(2);
        msg.layout.dim[0].label = "landmarks";
        msg.layout.dim[0].size = landmarks.size();
        msg.layout.dim[0].stride = landmarks.size() * 4;
        msg.layout.dim[1].label = "properties";
        msg.layout.dim[1].size = 4; // x, y, type, orientation
        msg.layout.dim[1].stride = 4;
        msg.layout.data_offset = 0;
        
        landmarks_publisher_->publish(msg);
    }
    
    void publish_markers(const std::vector<Point2f>& visible_points, 
                        const std::vector<Point2f>& all_points,
                        const std::vector<Landmark>& visible_landmarks,
                        const std::vector<Landmark>& all_landmarks)
    {
        auto marker_array = visualization_msgs::msg::MarkerArray();
        
        // Clear all previous markers
        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);
        
        // Create marker for all white line points (gray)
        visualization_msgs::msg::Marker all_points_marker;
        all_points_marker.header.frame_id = "map";
        all_points_marker.header.stamp = this->get_clock()->now();
        all_points_marker.ns = "all_whiteline_points";
        all_points_marker.id = 0;
        all_points_marker.type = visualization_msgs::msg::Marker::POINTS;
        all_points_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set marker properties for all points (gray)
        all_points_marker.pose.orientation.w = 1.0;
        all_points_marker.scale.x = 0.03;  // Smaller point width for dense field
        all_points_marker.scale.y = 0.03;  // Smaller point height for dense field
        all_points_marker.color.r = 0.7;  // Light gray color
        all_points_marker.color.g = 0.7;  // Light gray color
        all_points_marker.color.b = 0.7;  // Light gray color
        all_points_marker.color.a = 0.4;  // More transparent
        
        // Add all points to marker
        for (const auto& point : all_points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            all_points_marker.points.push_back(p);
        }
        
        // Create marker for visible white line points (white)
        visualization_msgs::msg::Marker visible_points_marker;
        visible_points_marker.header.frame_id = "map";
        visible_points_marker.header.stamp = this->get_clock()->now();
        visible_points_marker.ns = "visible_whiteline_points";
        visible_points_marker.id = 1;
        visible_points_marker.type = visualization_msgs::msg::Marker::POINTS;
        visible_points_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set marker properties for visible points (white)
        visible_points_marker.pose.orientation.w = 1.0;
        visible_points_marker.scale.x = 0.08;  // Larger point width
        visible_points_marker.scale.y = 0.08;  // Larger point height
        visible_points_marker.color.r = 1.0;  // White color
        visible_points_marker.color.g = 1.0;  // White color  
        visible_points_marker.color.b = 1.0;  // White color
        visible_points_marker.color.a = 1.0;  // Fully opaque
        
        // Add visible points to marker
        for (const auto& point : visible_points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            visible_points_marker.points.push_back(p);
        }
        
        // Create markers for all landmarks
        for (size_t i = 0; i < all_landmarks.size(); ++i) {
            visualization_msgs::msg::Marker landmark_marker;
            landmark_marker.header.frame_id = "map";
            landmark_marker.header.stamp = this->get_clock()->now();
            landmark_marker.ns = "all_landmarks";
            landmark_marker.id = i + 100;  // Offset to avoid ID conflicts
            landmark_marker.type = visualization_msgs::msg::Marker::CUBE;
            landmark_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set position
            landmark_marker.pose.position.x = all_landmarks[i].position.x;
            landmark_marker.pose.position.y = all_landmarks[i].position.y;
            landmark_marker.pose.position.z = 0.1;
            
            // Set orientation
            double half_theta = all_landmarks[i].orientation / 2.0;
            landmark_marker.pose.orientation.x = 0.0;
            landmark_marker.pose.orientation.y = 0.0;
            landmark_marker.pose.orientation.z = std::sin(half_theta);
            landmark_marker.pose.orientation.w = std::cos(half_theta);
            
            // Set size and color based on landmark type
            set_landmark_marker_properties(landmark_marker, all_landmarks[i].type, false);
            
            marker_array.markers.push_back(landmark_marker);
        }
        
        // Create markers for visible landmarks
        for (size_t i = 0; i < visible_landmarks.size(); ++i) {
            visualization_msgs::msg::Marker landmark_marker;
            landmark_marker.header.frame_id = "map";
            landmark_marker.header.stamp = this->get_clock()->now();
            landmark_marker.ns = "visible_landmarks";
            landmark_marker.id = i + 200;  // Offset to avoid ID conflicts
            landmark_marker.type = visualization_msgs::msg::Marker::CUBE;
            landmark_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set position
            landmark_marker.pose.position.x = visible_landmarks[i].position.x;
            landmark_marker.pose.position.y = visible_landmarks[i].position.y;
            landmark_marker.pose.position.z = 0.2;
            
            // Set orientation
            double half_theta = visible_landmarks[i].orientation / 2.0;
            landmark_marker.pose.orientation.x = 0.0;
            landmark_marker.pose.orientation.y = 0.0;
            landmark_marker.pose.orientation.z = std::sin(half_theta);
            landmark_marker.pose.orientation.w = std::cos(half_theta);
            
            // Set size and color based on landmark type
            set_landmark_marker_properties(landmark_marker, visible_landmarks[i].type, true);
            
            marker_array.markers.push_back(landmark_marker);
        }
        
        // Create marker for robot pose
        visualization_msgs::msg::Marker robot_marker;
        robot_marker.header.frame_id = "map";
        robot_marker.header.stamp = this->get_clock()->now();
        robot_marker.ns = "robot_pose";
        robot_marker.id = 2;
        robot_marker.type = visualization_msgs::msg::Marker::ARROW;
        robot_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set robot pose
        robot_marker.pose.position.x = robot_pose_.x;
        robot_marker.pose.position.y = robot_pose_.y;
        robot_marker.pose.position.z = 0.0;
        
        // Convert theta to quaternion
        double half_theta = robot_pose_.theta / 2.0;
        robot_marker.pose.orientation.x = 0.0;
        robot_marker.pose.orientation.y = 0.0;
        robot_marker.pose.orientation.z = std::sin(half_theta);
        robot_marker.pose.orientation.w = std::cos(half_theta);
        
        // Set robot marker properties
        robot_marker.scale.x = 0.5;  // Length
        robot_marker.scale.y = 0.1;  // Width
        robot_marker.scale.z = 0.1;  // Height
        robot_marker.color.r = 1.0;  // Red
        robot_marker.color.g = 0.0;  // Green
        robot_marker.color.b = 0.0;  // Blue
        robot_marker.color.a = 1.0;  // Alpha
        
        // Create FOV visualization marker
        visualization_msgs::msg::Marker fov_marker;
        fov_marker.header.frame_id = "map";
        fov_marker.header.stamp = this->get_clock()->now();
        fov_marker.ns = "robot_fov";
        fov_marker.id = 3;
        fov_marker.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
        fov_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set FOV marker properties
        fov_marker.pose.orientation.w = 1.0;
        fov_marker.scale.x = 1.0;
        fov_marker.scale.y = 1.0;
        fov_marker.scale.z = 1.0;
        fov_marker.color.r = 0.0;
        fov_marker.color.g = 1.0;
        fov_marker.color.b = 0.0;
        fov_marker.color.a = 0.2;  // Transparent
        
        // Create FOV triangle
        geometry_msgs::msg::Point robot_point;
        robot_point.x = robot_pose_.x;
        robot_point.y = robot_pose_.y;
        robot_point.z = 0.0;
        
        geometry_msgs::msg::Point fov_left;
        fov_left.x = robot_pose_.x + max_distance_ * std::cos(robot_pose_.theta + fov_/2);
        fov_left.y = robot_pose_.y + max_distance_ * std::sin(robot_pose_.theta + fov_/2);
        fov_left.z = 0.0;
        
        geometry_msgs::msg::Point fov_right;
        fov_right.x = robot_pose_.x + max_distance_ * std::cos(robot_pose_.theta - fov_/2);
        fov_right.y = robot_pose_.y + max_distance_ * std::sin(robot_pose_.theta - fov_/2);
        fov_right.z = 0.0;
        
        fov_marker.points.push_back(robot_point);
        fov_marker.points.push_back(fov_left);
        fov_marker.points.push_back(fov_right);
        
        // Add markers to array
        marker_array.markers.push_back(all_points_marker);
        marker_array.markers.push_back(visible_points_marker);
        marker_array.markers.push_back(robot_marker);
        marker_array.markers.push_back(fov_marker);
        
        marker_publisher_->publish(marker_array);
    }
    
    void set_landmark_marker_properties(visualization_msgs::msg::Marker& marker, LandmarkType type, bool is_visible)
    {
        switch (type) {
            case LandmarkType::T_JUNCTION:
                marker.scale.x = 0.15;
                marker.scale.y = 0.15;
                marker.scale.z = 0.15;
                marker.color.r = 1.0;  // Red
                marker.color.g = 0.0;
                marker.color.b = 0.0;
                break;
            case LandmarkType::L_CORNER:
                marker.scale.x = 0.12;
                marker.scale.y = 0.12;
                marker.scale.z = 0.12;
                marker.color.r = 0.0;  // Blue
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                break;
            case LandmarkType::X_CROSS:
                marker.scale.x = 0.20;
                marker.scale.y = 0.20;
                marker.scale.z = 0.20;
                marker.color.r = 1.0;  // Yellow
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                break;
            case LandmarkType::PENALTY_MARK:
                marker.scale.x = 0.10;
                marker.scale.y = 0.10;
                marker.scale.z = 0.10;
                marker.color.r = 1.0;  // Magenta
                marker.color.g = 0.0;
                marker.color.b = 1.0;
                break;
            case LandmarkType::GOAL_POST:
                marker.scale.x = 0.18;
                marker.scale.y = 0.18;
                marker.scale.z = 0.25;
                marker.color.r = 0.0;  // Cyan
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                break;
        }
        
        // Set alpha based on visibility
        marker.color.a = is_visible ? 1.0 : 0.3;
    }
    
    // ROS2 publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr whiteline_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr landmarks_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr robot_pose_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Simulation components
    std::unique_ptr<Preprocessor> preprocessor_;
    std::vector<Point2f> all_whiteline_points_;
    std::vector<Landmark> all_landmarks_;
    Pose robot_pose_;
    
    // Simulation parameters
    double fov_;
    double max_distance_;
    double robot_speed_;
    double time_step_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhitelineSimulatorNode>());
    rclcpp::shutdown();
    return 0;
} 