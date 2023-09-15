#include <data_process/data_process.hpp>

namespace data_process_ns
{
    Data_Process::Data_Process(const rclcpp::NodeOptions & options):Node("data_process_node",options)
    {
        RCLCPP_INFO(get_logger(),"Data_process_node is running.");
        callback_group_sub=this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        rclcpp::SubscriptionOptions sub_opt;
        sub_opt.callback_group = callback_group_sub;
        rx_sub=this->create_subscription<std_msgs::msg::UInt8MultiArray>("/rx_data", 10, std::bind(&Data_Process::process_callback, this, std::placeholders::_1),sub_opt);
        nav_request_pub=this->create_publisher<geometry_msgs::msg::PoseStamped>("/nav_request",10);
        // tf_broadcaster= std::make_unique<tf2_ros::TransformBroadcaster>(this);
        plan_sub=this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel_nav",10,std::bind(&Data_Process::plan_callback, this, std::placeholders::_1),sub_opt);
        tx_pub=this->create_publisher<std_msgs::msg::UInt8MultiArray>("/tx_data",10);
        u8_sub=this->create_subscription<std_msgs::msg::UInt8>("/u8_nav", 10, std::bind(&Data_Process::u8_callback, this, std::placeholders::_1),sub_opt);
        this->set_pose_vector();
    }
    Data_Process::~Data_Process()
    {}
    void Data_Process::process_callback(const std_msgs::msg::UInt8MultiArray::SharedPtr rx_buf)
    {
        rx_bag rx_data;
        std::copy(rx_buf->data.begin(), rx_buf->data.end(), reinterpret_cast<uint8_t *>(&rx_data));
        if(crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&rx_data),rx_len))
        {
            if(rx_data.nav_request<nav_pose.size())
            {
                nav_request.header.frame_id="map";
                nav_request.header.stamp=this->now();
                nav_request.pose=nav_pose[rx_data.nav_request];
                nav_request_pub->publish(nav_request);
            }
        }
        else
        {
            RCLCPP_ERROR(get_logger(),"crc false.");
        }
    }
    void Data_Process::u8_callback(const std_msgs::msg::UInt8::SharedPtr u8_nav)
    {
        if(u8_nav->data<nav_pose.size())
        {
            nav_request.header.frame_id="map";
            nav_request.header.stamp=this->now();
            nav_request.pose=nav_pose[u8_nav->data];
            nav_request_pub->publish(nav_request);
        }
    }
    void Data_Process::plan_callback(const geometry_msgs::msg::Twist::SharedPtr plan_vel)
    {
        tx_bag tx_data;
        tx_data.vx=-plan_vel->linear.x;
        tx_data.vy=-plan_vel->linear.y;
        tx_data.wz=-plan_vel->angular.z;
        crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&tx_data),tx_len);
        tx_buf.data=toVector(tx_data);
        tx_pub->publish(tx_buf);
    }
    void Data_Process::set_pose_vector()
    {
        geometry_msgs::msg::Pose pose0;
        pose0.position.x=-5.0;
        pose0.position.y=1.0;
        pose0.orientation.w=1.0;
        geometry_msgs::msg::Pose pose1;
        pose1.position.x=-5.0;
        pose1.position.y=0.0;
        pose1.orientation.w=1.0;
        geometry_msgs::msg::Pose pose2;
        pose2.position.x=0.0;
        pose2.position.y=0.0;
        pose2.orientation.w=1.0;
        geometry_msgs::msg::Pose pose3;
        pose3.position.x=0.0;
        pose3.position.y=1.0;
        pose3.orientation.w=1.0;
        /* geometry_msgs::msg::Pose pose0;
        pose0.position.x=1.0;
        pose0.position.y=1.0;
        pose0.orientation.w=1.0;
        geometry_msgs::msg::Pose pose1;
        pose1.position.x=-1.0;
        pose1.position.y=1.0;
        pose1.orientation.w=1.0;
        geometry_msgs::msg::Pose pose2;
        pose2.position.x=-1.0;
        pose2.position.y=-1.0;
        pose2.orientation.w=1.0;
        geometry_msgs::msg::Pose pose3;
        pose3.position.x=1.0;
        pose3.position.y=-1.0;
        pose3.orientation.w=1.0; */
        nav_pose.resize(4);
        nav_pose[0]=pose0;
        nav_pose[1]=pose1;
        nav_pose[2]=pose2;
        nav_pose[3]=pose3;
    }
}

int main(int argc, char **argv) 
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<data_process_ns::Data_Process>(rclcpp::NodeOptions());
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return EXIT_SUCCESS;
}