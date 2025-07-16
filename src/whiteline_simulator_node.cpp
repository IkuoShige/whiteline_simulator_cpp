#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
      robot_pose_(-5.5, -1.8, 0.0),  // Start at touchline-penalty area intersection facing field center
      odom_pose_(-5.5, -1.8, 0.0),   // Initialize odometry to same position
      time_step_(0.0),
      movement_state_(0),
      state_duration_(0.0),
      prev_robot_pose_(-5.5, -1.8, 0.0)
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
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "whiteline_markers", 10);
            
        // TF broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
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
        // Slow forward movement for particle tracking experiment
        updateSlowForwardMovement();
        
        // Increment time step for logging
        time_step_ += 0.1;
        
        // Publish current robot pose and odometry
        publish_robot_pose();
        publish_odometry();
        publish_tf();
        
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
        
        // Log odometry drift every 10 seconds for debugging
        if (static_cast<int>(time_step_ * 10) % 100 == 0) {
            double dx_error = robot_pose_.x - odom_pose_.x;
            double dy_error = robot_pose_.y - odom_pose_.y;
            double total_error = std::sqrt(dx_error * dx_error + dy_error * dy_error);
            RCLCPP_INFO(this->get_logger(),
                "Odometry vs Ground Truth - Error: %.3fm, True: (%.3f, %.3f), Odom: (%.3f, %.3f)",
                total_error, robot_pose_.x, robot_pose_.y, odom_pose_.x, odom_pose_.y);
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
    
    void publish_odometry()
    {
        auto odom_msg = nav_msgs::msg::Odometry();
        auto current_time = this->now();
        
        // Header
        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";
        odom_msg.child_frame_id = "base_link";
        
        // Position (use odometry estimate, not true pose)
        odom_msg.pose.pose.position.x = odom_pose_.x;
        odom_msg.pose.pose.position.y = odom_pose_.y;
        odom_msg.pose.pose.position.z = 0.0;
        
        // Orientation (convert theta to quaternion)
        double half_theta = odom_pose_.theta / 2.0;
        odom_msg.pose.pose.orientation.x = 0.0;
        odom_msg.pose.pose.orientation.y = 0.0;
        odom_msg.pose.pose.orientation.z = std::sin(half_theta);
        odom_msg.pose.pose.orientation.w = std::cos(half_theta);
        
        // Calculate velocity (simple difference using odometry)
        static auto last_time = current_time;
        static Pose prev_odom_pose = odom_pose_;
        double dt = (current_time - last_time).seconds();
        if (dt > 0.0) {
            double dx = odom_pose_.x - prev_odom_pose.x;
            double dy = odom_pose_.y - prev_odom_pose.y;
            double dtheta = odom_pose_.theta - prev_odom_pose.theta;
            
            // Normalize angle difference
            while (dtheta > M_PI) dtheta -= 2 * M_PI;
            while (dtheta < -M_PI) dtheta += 2 * M_PI;
            
            odom_msg.twist.twist.linear.x = dx / dt;
            odom_msg.twist.twist.linear.y = dy / dt;
            odom_msg.twist.twist.angular.z = dtheta / dt;
        } else {
            odom_msg.twist.twist.linear.x = 0.0;
            odom_msg.twist.twist.linear.y = 0.0;
            odom_msg.twist.twist.angular.z = 0.0;
        }
        
        // Simple covariance (diagonal matrix with small values)
        odom_msg.pose.covariance[0] = 0.01;   // x
        odom_msg.pose.covariance[7] = 0.01;   // y
        odom_msg.pose.covariance[35] = 0.02;  // theta
        
        odom_msg.twist.covariance[0] = 0.01;   // vx
        odom_msg.twist.covariance[7] = 0.01;   // vy
        odom_msg.twist.covariance[35] = 0.02;  // vtheta
        
        odom_publisher_->publish(odom_msg);
        
        // Update previous poses for next iteration
        prev_robot_pose_ = robot_pose_;
        prev_odom_pose = odom_pose_;
        last_time = current_time;
    }
    
    void publish_tf()
    {
        geometry_msgs::msg::TransformStamped t;
        auto current_time = this->now();
        
        // Publish odom -> base_link transform
        t.header.stamp = current_time;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";
        
        t.transform.translation.x = odom_pose_.x;
        t.transform.translation.y = odom_pose_.y;
        t.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_pose_.theta);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(t);
        
        // Also publish base_link -> base_footprint transform (identity)
        geometry_msgs::msg::TransformStamped footprint_tf;
        footprint_tf.header.stamp = current_time;
        footprint_tf.header.frame_id = "base_link";
        footprint_tf.child_frame_id = "base_footprint";
        
        footprint_tf.transform.translation.x = 0.0;
        footprint_tf.transform.translation.y = 0.0;
        footprint_tf.transform.translation.z = 0.0;
        
        tf2::Quaternion identity_q;
        identity_q.setRPY(0, 0, 0);
        footprint_tf.transform.rotation.x = identity_q.x();
        footprint_tf.transform.rotation.y = identity_q.y();
        footprint_tf.transform.rotation.z = identity_q.z();
        footprint_tf.transform.rotation.w = identity_q.w();
        
        tf_broadcaster_->sendTransform(footprint_tf);
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
    
    void updateHumanoidMovement()
    {
        const double STATE_DURATION = 3.0;  // Each movement state lasts 3 seconds
        const double MOVE_SPEED = 0.5;      // m/s movement speed
        const double TURN_SPEED = 0.8;      // rad/s rotation speed
        
        // Check if it's time to change movement state
        if (state_duration_ >= STATE_DURATION) {
            movement_state_ = (movement_state_ + 1) % 8;  // 8 different movement states
            state_duration_ = 0.0;
            RCLCPP_INFO(this->get_logger(), "Switching to movement state: %d", movement_state_);
        }
        
        double dt = 0.1;  // Time step for movement calculation
        
        switch (movement_state_) {
            case 0:  // Move forward
                robot_pose_.x += MOVE_SPEED * dt * std::cos(robot_pose_.theta);
                robot_pose_.y += MOVE_SPEED * dt * std::sin(robot_pose_.theta);
                break;
            case 1:  // Move backward
                robot_pose_.x -= MOVE_SPEED * dt * std::cos(robot_pose_.theta);
                robot_pose_.y -= MOVE_SPEED * dt * std::sin(robot_pose_.theta);
                break;
            case 2:  // Move left (strafe)
                robot_pose_.x += MOVE_SPEED * dt * std::cos(robot_pose_.theta + M_PI/2);
                robot_pose_.y += MOVE_SPEED * dt * std::sin(robot_pose_.theta + M_PI/2);
                break;
            case 3:  // Move right (strafe)
                robot_pose_.x += MOVE_SPEED * dt * std::cos(robot_pose_.theta - M_PI/2);
                robot_pose_.y += MOVE_SPEED * dt * std::sin(robot_pose_.theta - M_PI/2);
                break;
            case 4:  // Rotate left
                robot_pose_.theta += TURN_SPEED * dt;
                if (robot_pose_.theta > M_PI) robot_pose_.theta -= 2 * M_PI;
                break;
            case 5:  // Rotate right
                robot_pose_.theta -= TURN_SPEED * dt;
                if (robot_pose_.theta < -M_PI) robot_pose_.theta += 2 * M_PI;
                break;
            case 6:  // Move forward-left diagonal
                robot_pose_.x += MOVE_SPEED * dt * std::cos(robot_pose_.theta + M_PI/4);
                robot_pose_.y += MOVE_SPEED * dt * std::sin(robot_pose_.theta + M_PI/4);
                break;
            case 7:  // Move forward-right diagonal
                robot_pose_.x += MOVE_SPEED * dt * std::cos(robot_pose_.theta - M_PI/4);
                robot_pose_.y += MOVE_SPEED * dt * std::sin(robot_pose_.theta - M_PI/4);
                break;
        }
        
        // Keep robot within field boundaries (with some margin)
        const double FIELD_X_MAX = 6.5;  // Half field length with margin
        const double FIELD_Y_MAX = 4.0;  // Half field width with margin
        
        if (robot_pose_.x > FIELD_X_MAX || robot_pose_.x < -FIELD_X_MAX ||
            robot_pose_.y > FIELD_Y_MAX || robot_pose_.y < -FIELD_Y_MAX) {
            // If robot goes out of bounds, turn towards center
            double angle_to_center = std::atan2(-robot_pose_.y, -robot_pose_.x);
            robot_pose_.theta = angle_to_center;
        }
    }
    
    void updateSlowForwardMovement()
    {
        const double SLOW_SPEED = 0.2;  // Very slow forward speed: 0.2 m/s
        double dt = 0.1;  // Time step for movement calculation
        
        // Move forward slowly in current direction (true robot pose)
        robot_pose_.x += SLOW_SPEED * dt * std::cos(robot_pose_.theta);
        robot_pose_.y += SLOW_SPEED * dt * std::sin(robot_pose_.theta);
        
        // Update odometry with some drift/error (reduced for more realistic simulation)
        const double ODOM_DRIFT_RATE = 0.002;  // 0.2% drift per update (reduced from 2%)
        const double ODOM_NOISE = 0.0005;      // Small random noise (reduced)
        
        // Add some systematic drift and noise to odometry
        double dx_true = SLOW_SPEED * dt * std::cos(robot_pose_.theta);
        double dy_true = SLOW_SPEED * dt * std::sin(robot_pose_.theta);
        
        // Simulate odometry drift (slightly different from true movement)
        double dx_odom = dx_true * (1.0 + ODOM_DRIFT_RATE * ((rand() % 100) / 100.0 - 0.5));
        double dy_odom = dy_true * (1.0 + ODOM_DRIFT_RATE * ((rand() % 100) / 100.0 - 0.5));
        
        // Add small random noise
        dx_odom += ODOM_NOISE * ((rand() % 100) / 100.0 - 0.5);
        dy_odom += ODOM_NOISE * ((rand() % 100) / 100.0 - 0.5);
        
        odom_pose_.x += dx_odom;
        odom_pose_.y += dy_odom;
        odom_pose_.theta = robot_pose_.theta;  // Assume angle sensor is accurate
        
        // Keep robot within field boundaries (with some margin)
        const double FIELD_X_MAX = 6.0;  // Half field length with margin
        const double FIELD_Y_MAX = 3.5;  // Half field width with margin
        
        if (robot_pose_.x > FIELD_X_MAX || robot_pose_.x < -FIELD_X_MAX ||
            robot_pose_.y > FIELD_Y_MAX || robot_pose_.y < -FIELD_Y_MAX) {
            // If robot goes out of bounds, turn towards center
            double angle_to_center = std::atan2(-robot_pose_.y, -robot_pose_.x);
            robot_pose_.theta = angle_to_center;
            odom_pose_.theta = robot_pose_.theta;  // Update odometry angle too
        }
    }
    
    // ROS2 publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr whiteline_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr landmarks_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Pose2D>::SharedPtr robot_pose_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    
    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Simulation components
    std::unique_ptr<Preprocessor> preprocessor_;
    std::vector<Point2f> all_whiteline_points_;
    std::vector<Landmark> all_landmarks_;
    Pose robot_pose_;         // True robot pose (ground truth)
    Pose odom_pose_;          // Odometry estimate with accumulated error
    
    // Simulation parameters
    double fov_;
    double max_distance_;
    double robot_speed_;
    double time_step_;
    int movement_state_;      // Current movement state (0-7)
    double state_duration_;   // How long in current state
    Pose prev_robot_pose_;    // Previous pose for odometry calculation
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhitelineSimulatorNode>());
    rclcpp::shutdown();
    return 0;
} 