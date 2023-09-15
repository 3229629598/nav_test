#ifndef data_process_hpp
#define data_process_hpp

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <std_msgs/msg/u_int8.hpp>
// #include <std_msgs/msg/float32_multi_array.hpp>
// #include <tf2_ros/transform_broadcaster.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <data_process/crc.hpp>
#include <data_process/packet.hpp>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <chrono>

namespace data_process_ns
{
    class Data_Process : public rclcpp::Node
    {
        public:
        Data_Process(const rclcpp::NodeOptions & options);
        ~Data_Process();
        private:
        rclcpp::CallbackGroup::SharedPtr callback_group_sub;
        rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr rx_sub;
        void process_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr rx_buf);
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr nav_request_pub;
        geometry_msgs::msg::PoseStamped nav_request;
        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr plan_sub;
        void plan_callback(const geometry_msgs::msg::Twist::SharedPtr plan_vel);
        rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr tx_pub;
        std_msgs::msg::UInt8MultiArray tx_buf;
        void set_pose_vector(void);
        std::vector<geometry_msgs::msg::Pose> nav_pose;
        rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr u8_sub;
        void u8_callback(const std_msgs::msg::UInt8::SharedPtr u8_nav);
    };
}

#endif
