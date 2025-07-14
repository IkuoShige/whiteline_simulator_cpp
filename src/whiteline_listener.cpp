#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "whiteline_simulator_cpp/preprocessor.hpp"

class WhitelineListenerNode : public rclcpp::Node
{
public:
    WhitelineListenerNode()
    : Node("whiteline_listener_node"), preprocessor_(std::make_unique<Preprocessor>())
    {
        // Subscribers
        whiteline_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "whiteline_points", 10, 
            std::bind(&WhitelineListenerNode::whiteline_callback, this, std::placeholders::_1));
        
        landmarks_subscription_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "landmarks", 10,
            std::bind(&WhitelineListenerNode::landmarks_callback, this, std::placeholders::_1));
        
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose2D>(
            "robot_pose", 10,
            std::bind(&WhitelineListenerNode::pose_callback, this, std::placeholders::_1));
            
        RCLCPP_INFO(this->get_logger(), "Whiteline Listener Node started");
        RCLCPP_INFO(this->get_logger(), "Subscribed to: /whiteline_points, /landmarks, /robot_pose");
    }

private:
    void whiteline_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        size_t num_points = msg->data.size() / 2;
        RCLCPP_INFO(this->get_logger(), "Received %zu white line points:", num_points);
        
        // Print first few points for demo
        size_t max_display = std::min(num_points, static_cast<size_t>(3));
        for (size_t i = 0; i < max_display; ++i) {
            float x = msg->data[i * 2];
            float y = msg->data[i * 2 + 1];
            RCLCPP_INFO(this->get_logger(), "  Point %zu: (%.2f, %.2f)", i + 1, x, y);
        }
        
        if (num_points > max_display) {
            RCLCPP_INFO(this->get_logger(), "  ... and %zu more points", num_points - max_display);
        }
    }
    
    void landmarks_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
    {
        size_t num_landmarks = msg->data.size() / 4;
        RCLCPP_INFO(this->get_logger(), "Received %zu landmarks:", num_landmarks);
        
        // Print all landmarks with their types
        for (size_t i = 0; i < num_landmarks; ++i) {
            float x = msg->data[i * 4];
            float y = msg->data[i * 4 + 1];
            int type = static_cast<int>(msg->data[i * 4 + 2]);
            float orientation = msg->data[i * 4 + 3];
            
            // Convert type to string
            std::string type_name = preprocessor_->get_landmark_type_name(static_cast<LandmarkType>(type));
            
            RCLCPP_INFO(this->get_logger(), 
                "  Landmark %zu: (%.2f, %.2f) Type: %s, Orientation: %.2f rad (%.1f deg)", 
                i + 1, x, y, type_name.c_str(), orientation, orientation * 180.0 / M_PI);
        }
    }
    
    void pose_callback(const geometry_msgs::msg::Pose2D::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), 
            "Robot pose: x=%.2f, y=%.2f, theta=%.2f rad (%.1f deg)", 
            msg->x, msg->y, msg->theta, msg->theta * 180.0 / M_PI);
    }
    
    std::unique_ptr<Preprocessor> preprocessor_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr whiteline_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr landmarks_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose2D>::SharedPtr pose_subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WhitelineListenerNode>());
    rclcpp::shutdown();
    return 0;
} 