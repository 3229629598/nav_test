import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from geometry_msgs.msg import PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile
from rclpy.duration import Duration

class nav_ctrl_node(Node):
    def __init__(self,name):
        super().__init__(name)
        self.get_logger().info("Nav_ctrl_node is running.")
        self._callback_group = ReentrantCallbackGroup()
        self.nav=BasicNavigator()
        #初始化位置，代替rviz2的2D Pose Estimate
        self.initial_pose = PoseStamped()
        self.initial_pose.header.frame_id='map'
        self.initial_pose.header.stamp=self.get_clock().now().to_msg()
        self.initial_pose.pose.position.x=0.0
        self.initial_pose.pose.position.y=0.0
        self.initial_pose.pose.orientation.z=0.0
        self.initial_pose.pose.orientation.w=1.0
        self.nav.setInitialPose(self.initial_pose)
        self.nav.waitUntilNav2Active()
        #如果需要，您也可以更改或加载地图
        #navigator.changeMap('/path/to/map.yaml')
        #您可以使用导航器清除或获取成本图
        #navigator.clearAllCostmaps() # 还有clearLocalCostmap() 和clearGlobalCostmap()
        #global_costmap = navigator.getGlobalCostmap()
        #local_costmap = navigator.getLocalCostmap()
        self.create_subscription(
            msg_type=PoseStamped,
            topic="/nav_request",
            callback=self.nav_request_callback,
            qos_profile=QoSProfile(depth=1),
            callback_group=self._callback_group,
        )

    def nav_request_callback(self,nav_request:PoseStamped):
        self.get_logger().info("There is a new requestion.")
        self.nav.goToPose(nav_request)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()
            #一些导航超时导致演示取消
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
                self.nav.cancelTask()

def main(args=None):
    rclpy.init(args=args)
    node = nav_ctrl_node("nav_ctrl_node")
    rclpy.spin(node)
    node.get_logger().error("nav_ctrl_node has died.")
    node.nav.lifecycleShutdown()
    rclpy.shutdown()
    exit(0)
