#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "booster_interface/msg/odometer.hpp"
#include "whiteline_simulator_cpp/preprocessor.hpp"
#include "whiteline_simulator_cpp/pose.hpp"

using namespace std::chrono_literals;

class WhitelineSimulatorNode : public rclcpp::Node
{
public:
    WhitelineSimulatorNode()
    : Node("whiteline_simulator_node"), 
      preprocessor_(std::make_unique<Preprocessor>()),
      robot_pose_(-4.0, -5.0, 1.5708),  // Start at field coordinate (-4.0, -5.0) facing north (π/2)
      odom_pose_(-4.0, -5.0, 1.5708),   // Initialize odometry to same position
      time_step_(0.0),
      movement_state_(0),
      state_duration_(0.0),
      prev_robot_pose_(-4.0, -5.0, 1.5708),
      cmd_vel_linear_(0.0),
      cmd_vel_angular_(0.0),
      current_timestamp_(this->now())
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
        this->declare_parameter<bool>("keyboard_control", false);
        
        // Camera simulation parameters
        this->declare_parameter<double>("white_line_width", 0.05);      // White line width (5cm)
        this->declare_parameter<double>("pixel_density", 0.01);         // Pixel density in meters (1cm per pixel)
        this->declare_parameter<double>("noise_std", 0.005);            // Noise standard deviation (5mm)
        this->declare_parameter<double>("detection_probability", 0.8);   // Probability of detecting a white line pixel
        this->declare_parameter<double>("min_contrast_threshold", 0.3); // Minimum contrast threshold for detection
        this->declare_parameter<bool>("use_realistic_simulation", true); // Use realistic camera simulation
        
        // Real robot compatibility parameters
        this->declare_parameter<double>("robot.odom_factor", 1.0);  // Robot odometry correction factor
        this->declare_parameter<bool>("publish_odometer_state", true); // Enable /odometer_state publishing
        
        // Publishers
        whiteline_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "whiteline_points", 10);
        landmarks_publisher_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
            "landmarks", 10);
        robot_pose_publisher_ = this->create_publisher<geometry_msgs::msg::Pose2D>(
            "robot_pose", 10);
        odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>(
            "odom", 10);
        odometer_state_publisher_ = this->create_publisher<booster_interface::msg::Odometer>(
            "odometer_state", 10);
        marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "whiteline_markers", 10);
            
        // Twist subscriber for keyboard control
        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, 
            std::bind(&WhitelineSimulatorNode::cmd_vel_callback, this, std::placeholders::_1));
            
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
        keyboard_control_ = this->get_parameter("keyboard_control").as_bool();
        
        // Get camera simulation parameters
        camera_params_.white_line_width = this->get_parameter("white_line_width").as_double();
        camera_params_.pixel_density = this->get_parameter("pixel_density").as_double();
        camera_params_.noise_std = this->get_parameter("noise_std").as_double();
        camera_params_.detection_probability = this->get_parameter("detection_probability").as_double();
        camera_params_.min_contrast_threshold = this->get_parameter("min_contrast_threshold").as_double();
        camera_params_.max_distance = max_distance_;
        camera_params_.fov = fov_;
        use_realistic_simulation_ = this->get_parameter("use_realistic_simulation").as_bool();
        
        // Get real robot compatibility parameters  
        robot_odom_factor_ = this->get_parameter("robot.odom_factor").as_double();
        publish_odometer_state_ = this->get_parameter("publish_odometer_state").as_bool();
        
        // Load or generate white line points and landmarks
        if (generate_soccer_field) {
            RCLCPP_INFO(this->get_logger(), "Generating soccer field white line points and landmarks...");
            
            // Set up soccer field dimensions
            SoccerFieldDimensions dimensions;
            dimensions.field_length = this->get_parameter("field_length").as_double();
            dimensions.field_width = this->get_parameter("field_width").as_double();
            
            // Generate soccer field lines and landmarks
            if (use_realistic_simulation_) {
                // Use realistic white line areas with width
                all_whiteline_points_ = preprocessor_->generate_white_line_areas(
                    dimensions, camera_params_.white_line_width, 1000.0);  // 1000 points per square meter for better coverage
                RCLCPP_INFO(this->get_logger(), "Generated realistic white line areas with width %.3fm", camera_params_.white_line_width);
            } else {
                // Use original line-based generation
                all_whiteline_points_ = preprocessor_->generate_soccer_field_lines(dimensions, point_spacing);
                RCLCPP_INFO(this->get_logger(), "Generated line-based white line points");
            }
            
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
        RCLCPP_INFO(this->get_logger(), "Keyboard control: %s", keyboard_control_ ? "enabled" : "disabled");
        if (keyboard_control_) {
            RCLCPP_INFO(this->get_logger(), "Listening for cmd_vel messages for keyboard control");
        }
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_vel_linear_ = msg->linear.x;
        cmd_vel_angular_ = msg->angular.z;
    }
    
    void timer_callback()
    {
        // Update robot movement based on control mode
        if (keyboard_control_) {
            updateKeyboardMovement();
        } else {
            // Slow forward movement for particle tracking experiment
            updateSlowForwardMovement();
        }
        
        // Increment time step for logging
        time_step_ += 0.1;
        
        // Get current time once for all messages to ensure synchronization
        current_timestamp_ = this->now();
        
        // Publish TF transforms first, then odometry to avoid timing issues
        publish_tf();
        publish_robot_pose();
        publish_odometry();
        
        // Simulate white line detection from robot's perspective
        std::vector<Point2f> visible_points;
        if (use_realistic_simulation_) {
            // Use realistic camera-based detection with noise and detection probability
            visible_points = preprocessor_->simulate_camera_white_line_detection(
                all_whiteline_points_, robot_pose_, camera_params_);
            RCLCPP_INFO(this->get_logger(), "Robot at (%.2f, %.2f, %.2f), Realistic camera detected %zu white line points", 
                        robot_pose_.x, robot_pose_.y, robot_pose_.theta, visible_points.size());
        } else {
            // Use original simple detection
            visible_points = preprocessor_->simulate_white_line(
                all_whiteline_points_, robot_pose_, fov_, max_distance_);
            RCLCPP_INFO(this->get_logger(), "Robot at (%.2f, %.2f, %.2f), Simple detection found %zu white line points", 
                        robot_pose_.x, robot_pose_.y, robot_pose_.theta, visible_points.size());
        }
        
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
        // Use synchronized timestamp
        auto current_time = current_timestamp_;
        
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
        
        // Set velocity based on control mode
        if (keyboard_control_) {
            // For keyboard control, use the limited velocities (apply same limits as in updateKeyboardMovement)
            const double ROBOT_MAX_SPEED_X_PLUS = 4.0 / 4.138;
            const double ROBOT_MAX_SPEED_X_MINUS = -4.0 / 4.322;
            const double ROBOT_MAX_SPEED_PI = 2 * M_PI / 4.492;
            
            double limited_linear_x = cmd_vel_linear_;
            double limited_angular_z = cmd_vel_angular_;
            
            // Apply same speed limits
            if (limited_linear_x > 0) {
                limited_linear_x = std::min(limited_linear_x, ROBOT_MAX_SPEED_X_PLUS);
            } else {
                limited_linear_x = std::max(limited_linear_x, ROBOT_MAX_SPEED_X_MINUS);
            }
            limited_angular_z = std::max(-ROBOT_MAX_SPEED_PI, std::min(limited_angular_z, ROBOT_MAX_SPEED_PI));
            
            odom_msg.twist.twist.linear.x = limited_linear_x;
            odom_msg.twist.twist.linear.y = 0.0;  // No lateral movement for differential drive
            odom_msg.twist.twist.angular.z = limited_angular_z;
        } else {
            // For automatic movement, calculate velocity from position difference
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
                
                // Convert global velocity to local velocity (robot frame)
                double v_global_x = dx / dt;
                double v_global_y = dy / dt;
                
                // Transform to robot's local frame using odometry angle
                double cos_theta = std::cos(odom_pose_.theta);
                double sin_theta = std::sin(odom_pose_.theta);
                
                odom_msg.twist.twist.linear.x = v_global_x * cos_theta + v_global_y * sin_theta;
                odom_msg.twist.twist.linear.y = -v_global_x * sin_theta + v_global_y * cos_theta;
                odom_msg.twist.twist.angular.z = dtheta / dt;
            } else {
                odom_msg.twist.twist.linear.x = 0.0;
                odom_msg.twist.twist.linear.y = 0.0;
                odom_msg.twist.twist.angular.z = 0.0;
            }
            prev_odom_pose = odom_pose_;
            last_time = current_time;
        }
        
        // Simple covariance (diagonal matrix with small values)
        odom_msg.pose.covariance[0] = 0.01;   // x
        odom_msg.pose.covariance[7] = 0.01;   // y
        odom_msg.pose.covariance[35] = 0.02;  // theta
        
        odom_msg.twist.covariance[0] = 0.01;   // vx
        odom_msg.twist.covariance[7] = 0.01;   // vy
        odom_msg.twist.covariance[35] = 0.02;  // vtheta
        
        odom_publisher_->publish(odom_msg);
        
        // Publish /odometer_state for real robot compatibility 
        if (publish_odometer_state_) {
            auto odometer_msg = booster_interface::msg::Odometer();
            
            // Apply robot odometry factor (real robot calibration)
            odometer_msg.x = static_cast<float>(odom_pose_.x * robot_odom_factor_);
            odometer_msg.y = static_cast<float>(odom_pose_.y * robot_odom_factor_);
            odometer_msg.theta = static_cast<float>(odom_pose_.theta);
            
            odometer_state_publisher_->publish(odometer_msg);
        }
        
        // Update previous poses for next iteration
        prev_robot_pose_ = robot_pose_;
    }
    
    void publish_tf()
    {
        // Use synchronized timestamp
        auto current_time = current_timestamp_;
        std::vector<geometry_msgs::msg::TransformStamped> transforms;
        
        // Publish map -> odom transform (identity, since we don't have SLAM)
        geometry_msgs::msg::TransformStamped map_to_odom;
        map_to_odom.header.stamp = current_time;
        map_to_odom.header.frame_id = "map";
        map_to_odom.child_frame_id = "odom";
        map_to_odom.transform.translation.x = 0.0;
        map_to_odom.transform.translation.y = 0.0;
        map_to_odom.transform.translation.z = 0.0;
        tf2::Quaternion map_q;
        map_q.setRPY(0, 0, 0);
        map_to_odom.transform.rotation.x = map_q.x();
        map_to_odom.transform.rotation.y = map_q.y();
        map_to_odom.transform.rotation.z = map_q.z();
        map_to_odom.transform.rotation.w = map_q.w();
        transforms.push_back(map_to_odom);
        
        // Publish odom -> base_link transform
        geometry_msgs::msg::TransformStamped odom_to_base;
        odom_to_base.header.stamp = current_time;
        odom_to_base.header.frame_id = "odom";
        odom_to_base.child_frame_id = "base_link";
        
        odom_to_base.transform.translation.x = odom_pose_.x;
        odom_to_base.transform.translation.y = odom_pose_.y;
        odom_to_base.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_pose_.theta);
        odom_to_base.transform.rotation.x = q.x();
        odom_to_base.transform.rotation.y = q.y();
        odom_to_base.transform.rotation.z = q.z();
        odom_to_base.transform.rotation.w = q.w();
        transforms.push_back(odom_to_base);
        
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
        transforms.push_back(footprint_tf);
        
        // Send all transforms together to ensure consistency
        tf_broadcaster_->sendTransform(transforms);
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
        
        // Create marker for visible white line points in robot frame (blue)
        visualization_msgs::msg::Marker visible_points_robot_marker;
        visible_points_robot_marker.header.frame_id = "base_link";
        visible_points_robot_marker.header.stamp = this->get_clock()->now();
        visible_points_robot_marker.ns = "visible_whiteline_points_robot";
        visible_points_robot_marker.id = 1;
        visible_points_robot_marker.type = visualization_msgs::msg::Marker::POINTS;
        visible_points_robot_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set marker properties for robot frame points (blue)
        visible_points_robot_marker.pose.orientation.w = 1.0;
        visible_points_robot_marker.scale.x = 0.08;  // Larger point width
        visible_points_robot_marker.scale.y = 0.08;  // Larger point height
        visible_points_robot_marker.color.r = 0.0;  // Blue color
        visible_points_robot_marker.color.g = 0.0;  // Blue color  
        visible_points_robot_marker.color.b = 1.0;  // Blue color
        visible_points_robot_marker.color.a = 1.0;  // Fully opaque
        
        // Add visible points in robot frame
        for (const auto& point : visible_points) {
            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            visible_points_robot_marker.points.push_back(p);
        }
        
        // Create marker for visible white line points transformed to global frame (white)
        visualization_msgs::msg::Marker visible_points_global_marker;
        visible_points_global_marker.header.frame_id = "map";
        visible_points_global_marker.header.stamp = this->get_clock()->now();
        visible_points_global_marker.ns = "visible_whiteline_points_global";
        visible_points_global_marker.id = 2;
        visible_points_global_marker.type = visualization_msgs::msg::Marker::POINTS;
        visible_points_global_marker.action = visualization_msgs::msg::Marker::ADD;
        
        // Set marker properties for global frame points (white)
        visible_points_global_marker.pose.orientation.w = 1.0;
        visible_points_global_marker.scale.x = 0.10;  // Slightly larger point width
        visible_points_global_marker.scale.y = 0.10;  // Slightly larger point height
        visible_points_global_marker.color.r = 1.0;  // White color
        visible_points_global_marker.color.g = 1.0;  // White color  
        visible_points_global_marker.color.b = 1.0;  // White color
        visible_points_global_marker.color.a = 1.0;  // Fully opaque
        
        // Transform robot frame points back to global frame for visualization
        // Forward transformation: R * p_local + p_robot (ROS convention)
        double cos_theta = std::cos(robot_pose_.theta);
        double sin_theta = std::sin(robot_pose_.theta);
        for (const auto& local_point : visible_points) {
            geometry_msgs::msg::Point p;
            p.x = robot_pose_.x + local_point.x * cos_theta - local_point.y * sin_theta;
            p.y = robot_pose_.y + local_point.x * sin_theta + local_point.y * cos_theta;
            p.z = 0.0;
            visible_points_global_marker.points.push_back(p);
        }
        
        // Create marker for sample debug points (red) - first 5 visible points transformed to global
        if (!visible_points.empty()) {
            visualization_msgs::msg::Marker sample_points_marker;
            sample_points_marker.header.frame_id = "map";
            sample_points_marker.header.stamp = this->get_clock()->now();
            sample_points_marker.ns = "sample_debug_points";
            sample_points_marker.id = 10;  // Changed ID to avoid conflicts
            sample_points_marker.type = visualization_msgs::msg::Marker::POINTS;
            sample_points_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set marker properties for sample points (red)
            sample_points_marker.pose.orientation.w = 1.0;
            sample_points_marker.scale.x = 0.15;  // Larger point width for visibility
            sample_points_marker.scale.y = 0.15;  // Larger point height for visibility
            sample_points_marker.color.r = 1.0;  // Red color
            sample_points_marker.color.g = 0.0;  // Red color
            sample_points_marker.color.b = 0.0;  // Red color
            sample_points_marker.color.a = 1.0;  // Fully opaque
            
            // Transform first 5 visible points from robot frame to global frame
            int num_sample_points = std::min(5, static_cast<int>(visible_points.size()));
            for (int i = 0; i < num_sample_points; ++i) {
                geometry_msgs::msg::Point p;
                // Transform robot-local point to global coordinates
                p.x = robot_pose_.x + visible_points[i].x * cos_theta - visible_points[i].y * sin_theta;
                p.y = robot_pose_.y + visible_points[i].x * sin_theta + visible_points[i].y * cos_theta;
                p.z = 0.1;  // Slightly elevated for visibility
                sample_points_marker.points.push_back(p);
            }
            
            marker_array.markers.push_back(sample_points_marker);
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
        
        // Create markers for visible landmarks in robot frame
        for (size_t i = 0; i < visible_landmarks.size(); ++i) {
            visualization_msgs::msg::Marker landmark_marker;
            landmark_marker.header.frame_id = "base_link";
            landmark_marker.header.stamp = this->get_clock()->now();
            landmark_marker.ns = "visible_landmarks_robot";
            landmark_marker.id = i + 200;  // Offset to avoid ID conflicts
            landmark_marker.type = visualization_msgs::msg::Marker::CUBE;
            landmark_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Set position in robot frame
            landmark_marker.pose.position.x = visible_landmarks[i].position.x;
            landmark_marker.pose.position.y = visible_landmarks[i].position.y;
            landmark_marker.pose.position.z = 0.2;
            
            // Set orientation in robot frame
            double half_theta = visible_landmarks[i].orientation / 2.0;
            landmark_marker.pose.orientation.x = 0.0;
            landmark_marker.pose.orientation.y = 0.0;
            landmark_marker.pose.orientation.z = std::sin(half_theta);
            landmark_marker.pose.orientation.w = std::cos(half_theta);
            
            // Set size and color based on landmark type (darker for robot frame)
            set_landmark_marker_properties(landmark_marker, visible_landmarks[i].type, false);  // Use darker colors
            
            marker_array.markers.push_back(landmark_marker);
        }
        
        // Create markers for visible landmarks transformed to global frame
        for (size_t i = 0; i < visible_landmarks.size(); ++i) {
            visualization_msgs::msg::Marker landmark_marker;
            landmark_marker.header.frame_id = "map";
            landmark_marker.header.stamp = this->get_clock()->now();
            landmark_marker.ns = "visible_landmarks_global";
            landmark_marker.id = i + 300;  // Offset to avoid ID conflicts
            landmark_marker.type = visualization_msgs::msg::Marker::CUBE;
            landmark_marker.action = visualization_msgs::msg::Marker::ADD;
            
            // Transform position to global frame (ROS convention)
            // Forward transformation: R * p_local + p_robot
            double cos_theta = std::cos(robot_pose_.theta);
            double sin_theta = std::sin(robot_pose_.theta);
            landmark_marker.pose.position.x = robot_pose_.x + visible_landmarks[i].position.x * cos_theta - visible_landmarks[i].position.y * sin_theta;
            landmark_marker.pose.position.y = robot_pose_.y + visible_landmarks[i].position.x * sin_theta + visible_landmarks[i].position.y * cos_theta;
            landmark_marker.pose.position.z = 0.25;  // Slightly higher for visibility
            
            // Transform orientation to global frame
            double global_orientation = visible_landmarks[i].orientation + robot_pose_.theta;
            double half_theta = global_orientation / 2.0;
            landmark_marker.pose.orientation.x = 0.0;
            landmark_marker.pose.orientation.y = 0.0;
            landmark_marker.pose.orientation.z = std::sin(half_theta);
            landmark_marker.pose.orientation.w = std::cos(half_theta);
            
            // Set size and color based on landmark type (bright for global frame)
            set_landmark_marker_properties(landmark_marker, visible_landmarks[i].type, true);  // Use bright colors
            
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
        marker_array.markers.push_back(visible_points_robot_marker);      // Robot frame white line points (blue)
        marker_array.markers.push_back(visible_points_global_marker);     // Global frame white line points (white)  
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
        
        // Keep robot within extended boundaries to allow operation outside field
        const double FIELD_X_MAX = 9.0;  // Extended margin beyond field boundaries
        const double FIELD_Y_MAX = 6.5;  // Extended margin beyond field boundaries
        
        if (robot_pose_.x > FIELD_X_MAX || robot_pose_.x < -FIELD_X_MAX ||
            robot_pose_.y > FIELD_Y_MAX || robot_pose_.y < -FIELD_Y_MAX) {
            // If robot goes out of bounds, turn towards center
            double angle_to_center = std::atan2(-robot_pose_.y, -robot_pose_.x);
            robot_pose_.theta = angle_to_center;
        }
    }
    
    void updateKeyboardMovement()
    {
        // ロボットの速度に関連する定数（実際のロボット特性に基づく）
        const double ROBOT_MAX_SPEED_X_PLUS = 4.0 / 4.138;   // 4.138秒で4m進むのが最大速度 (~0.967 m/s)
        const double ROBOT_MAX_SPEED_X_MINUS = -4.0 / 4.322; // 4.322秒で4m下がるのが最大速度 (~-0.925 m/s)
        // const double ROBOT_MAX_SPEED_Y = 4.0 / 10.55;     // 10.55秒で4m進むのが最大速度 (~0.379 m/s) - 現在は使用されていない
        const double ROBOT_MAX_SPEED_PI = 2 * M_PI / 4.492;  // 4.492秒で360度回転が最大速度 (~1.398 rad/s)
        
        double dt = 0.1;  // Time step for movement calculation
        
        // Apply robot speed limits to commanded velocities
        double limited_linear_x = cmd_vel_linear_;
        double limited_angular_z = cmd_vel_angular_;
        
        // Limit linear velocity based on direction
        if (limited_linear_x > 0) {
            limited_linear_x = std::min(limited_linear_x, ROBOT_MAX_SPEED_X_PLUS);
        } else {
            limited_linear_x = std::max(limited_linear_x, ROBOT_MAX_SPEED_X_MINUS);
        }
        
        // Limit angular velocity
        limited_angular_z = std::max(-ROBOT_MAX_SPEED_PI, std::min(limited_angular_z, ROBOT_MAX_SPEED_PI));
        
        // Update robot orientation first
        double dtheta_true = limited_angular_z * dt;
        robot_pose_.theta += dtheta_true;
        
        // Normalize angle to [-pi, pi]
        while (robot_pose_.theta > M_PI) robot_pose_.theta -= 2 * M_PI;
        while (robot_pose_.theta < -M_PI) robot_pose_.theta += 2 * M_PI;
        
        // Update robot position based on limited linear velocity in robot's local frame
        double dx_true = limited_linear_x * dt * std::cos(robot_pose_.theta);
        double dy_true = limited_linear_x * dt * std::sin(robot_pose_.theta);
        
        robot_pose_.x += dx_true;
        robot_pose_.y += dy_true;
        
        // Update odometry with some drift/error
        const double ODOM_DRIFT_RATE = 0.002;  // 0.2% drift per update
        const double ODOM_NOISE = 0.0005;      // Small random noise
        
        // Simulate odometry drift for angular velocity
        double dtheta_odom = dtheta_true * (1.0 + ODOM_DRIFT_RATE * ((rand() % 100) / 100.0 - 0.5));
        odom_pose_.theta += dtheta_odom;
        
        // Normalize odometry angle
        while (odom_pose_.theta > M_PI) odom_pose_.theta -= 2 * M_PI;
        while (odom_pose_.theta < -M_PI) odom_pose_.theta += 2 * M_PI;
        
        // Calculate odometry position update using odometry's angle (not true angle)
        // This simulates the fact that odometry uses its own angle estimate
        double dx_odom = cmd_vel_linear_ * dt * std::cos(odom_pose_.theta);
        double dy_odom = cmd_vel_linear_ * dt * std::sin(odom_pose_.theta);
        
        // Add drift and noise to position
        dx_odom *= (1.0 + ODOM_DRIFT_RATE * ((rand() % 100) / 100.0 - 0.5));
        dy_odom *= (1.0 + ODOM_DRIFT_RATE * ((rand() % 100) / 100.0 - 0.5));
        dx_odom += ODOM_NOISE * ((rand() % 100) / 100.0 - 0.5);
        dy_odom += ODOM_NOISE * ((rand() % 100) / 100.0 - 0.5);
        
        odom_pose_.x += dx_odom;
        odom_pose_.y += dy_odom;
        
        // Keep robot within extended boundaries to allow operation outside field
        const double FIELD_X_MAX = 9.0;  // Extended margin beyond field boundaries
        const double FIELD_Y_MAX = 6.5;  // Extended margin beyond field boundaries
        
        // Clamp position to extended boundaries
        if (robot_pose_.x > FIELD_X_MAX) robot_pose_.x = FIELD_X_MAX;
        if (robot_pose_.x < -FIELD_X_MAX) robot_pose_.x = -FIELD_X_MAX;
        if (robot_pose_.y > FIELD_Y_MAX) robot_pose_.y = FIELD_Y_MAX;
        if (robot_pose_.y < -FIELD_Y_MAX) robot_pose_.y = -FIELD_Y_MAX;
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
        
        // Keep robot within extended boundaries to allow operation outside field
        const double FIELD_X_MAX = 9.0;  // Extended margin beyond field boundaries
        const double FIELD_Y_MAX = 6.5;  // Extended margin beyond field boundaries
        
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
    rclcpp::Publisher<booster_interface::msg::Odometer>::SharedPtr odometer_state_publisher_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_publisher_;
    
    // ROS2 subscriber
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    
    // TF broadcaster
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Simulation components
    std::unique_ptr<Preprocessor> preprocessor_;
    std::vector<Point2f> all_whiteline_points_;
    std::vector<Landmark> all_landmarks_;
    // 状態管理
    Pose robot_pose_;         // True robot pose (ground truth)
    Pose odom_pose_;          // Odometry estimate with accumulated error
    
    // パラメータ
    double fov_;
    double max_distance_;
    double robot_speed_;
    double time_step_;
    int movement_state_;      // Current movement state (0-7)
    double state_duration_;   // How long in current state
    Pose prev_robot_pose_;    // Previous pose for odometry calculation
    
    // カメラシミュレーションパラメータ
    CameraSimulationParams camera_params_;
    bool use_realistic_simulation_;
    
    bool keyboard_control_;   // Whether keyboard control is enabled
    double cmd_vel_linear_;   // Commanded linear velocity from keyboard
    double cmd_vel_angular_;  // Commanded angular velocity from keyboard
    
    // Real robot compatibility parameters
    double robot_odom_factor_;   // Robot odometry correction factor
    bool publish_odometer_state_; // Enable /odometer_state publishing
    
    // タイムスタンプ
    rclcpp::Time current_timestamp_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhitelineSimulatorNode>());
    rclcpp::shutdown();
    return 0;
} 