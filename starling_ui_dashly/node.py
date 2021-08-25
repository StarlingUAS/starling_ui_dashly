import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String

class Dashboard_Node(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        # ROS Setup
        self.mission_start_publisher_ = self.create_publisher(String, 'mission_start', 10)
        self.emergency_stop_publisher_ = self.create_publisher(String, 'emergency_stop', 10)
        self.emergency_cutoff_publisher_ = self.create_publisher(String, 'emergency_cutoff', 10)
        timer_period = 0.5  # seconds

    def call_mission_start(self):
        msg = String()
        msg.data = 'mission_start'
        self.mission_start_publisher_.publish(msg)
        self.get_logger().info('mission_start published')

    def call_emergency_stop(self):
        msg = String()
        msg.data = 'emergency_stop'
        self.emergency_stop_publisher_.publish(msg)
        self.get_logger().info('emergency_stop published')

    def call_emergency_cutoff(self):
        msg = String()
        msg.data = 'emergency_cutoff'
        self.emergency_cutoff_publisher_.publish(msg)
        self.get_logger().info('emergency_cutoff published')