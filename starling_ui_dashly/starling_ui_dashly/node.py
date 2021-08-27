import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

from std_msgs.msg import String

class Dashboard_Node(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        # ROS Setup
        self.mission_start_publisher_ = self.create_publisher(String, 'mission_start', 10)
        self.mission_abort_publisher_ = self.create_publisher(String, 'mission_abort', 10)
        self.emergency_stop_publisher_ = self.create_publisher(String, 'emergency_stop', 10)
        self.timer_period = 0.01  # seconds
        self.emergency_stop_timer = None

    def call_mission_start(self):
        msg = String()
        msg.data = 'mission_start'
        self.mission_start_publisher_.publish(msg)
        self.get_logger().info('mission_start published')

    def toggle_emergency_stop(self, send):
        if self.emergency_stop_timer:
            self.emergency_stop_timer.cancel()
            self.emergency_stop_timer = None

        if send:
            self.emergency_stop_timer = self.create_timer(self.timer_period, self.send_emergency_stop)
        self.get_logger().info(f"ESTOP {'ENABLED' if send else 'DISABLED'}")

    def send_emergency_stop(self):
        msg = String()
        msg.data = 'emergency_stop'
        self.emergency_stop_publisher_.publish(msg)
        self.get_logger().info('emergency_stop published')

    def call_mission_abort(self):
        msg = String()
        msg.data = 'mission_abort'
        self.mission_abort_publisher_.publish(msg)
        self.get_logger().info('mission_abort published')