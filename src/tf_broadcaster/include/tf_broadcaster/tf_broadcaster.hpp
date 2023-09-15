#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/utils.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace tf_broadcaster_ns
{
    class TF_broadcaster : public rclcpp::Node
    {
        public:
        TF_broadcaster();
        ~TF_broadcaster();
        private:
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener;
        std::unique_ptr<tf2_ros::Buffer> tf_buf;
        geometry_msgs::msg::TransformStamped tf_f_to_b;
        geometry_msgs::msg::TransformStamped tf_o_to_f;
        rclcpp::TimerBase::SharedPtr tim;
        void tf_tim_callback(void);
    };
}