#include <tf_broadcaster/tf_broadcaster.hpp>

namespace tf_broadcaster_ns
{
    using namespace std::chrono_literals;
    TF_broadcaster::TF_broadcaster():Node("tf_broadcaster_node")
    {
        tf_broadcaster=std::make_unique<tf2_ros::TransformBroadcaster>(this);
        tf_listener=std::make_shared<tf2_ros::TransformListener>(*tf_buf);
        tim=this->create_wall_timer(500ms,std::bind(&TF_broadcaster::tf_tim_callback,this));
    }
    TF_broadcaster::~TF_broadcaster()
    {}
    void TF_broadcaster::tf_tim_callback()
    {
        /* tf_f_to_b.header.stamp=this->now();
        tf_f_to_b.header.frame_id="base_footprint";
        tf_f_to_b.child_frame_id="base_link";
        tf_f_to_b.transform.translation.x=0;
        tf_f_to_b.transform.translation.y=0;
        tf_f_to_b.transform.translation.z=0;
        // auto quat=tf2::Quaternion();
        // quat.setRPY(0,0,0);
        tf_f_to_b.transform.rotation.x=0;
        tf_f_to_b.transform.rotation.y=0;
        tf_f_to_b.transform.rotation.z=0;
        tf_f_to_b.transform.rotation.w=1;
        tf_broadcaster->sendTransform(tf_f_to_b); */
        try
        {
            tf_o_to_f=tf_buf->lookupTransform("odom","base_link",tf2::TimePointZero);
        }
        catch(const tf2::TransformException & ex)
        {
            return;
        }
        tf_o_to_f.header.frame_id="map";
        tf_o_to_f.child_frame_id="base_footprint";
        tf_broadcaster->sendTransform(tf_o_to_f);
    }
}

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::executors::MultiThreadedExecutor executor;
    auto node = std::make_shared<tf_broadcaster_ns::TF_broadcaster>();
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}