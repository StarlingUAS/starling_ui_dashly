import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import numpy as np

from std_msgs.msg import Empty
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from starling_allocator_msgs.msg import Allocations
from starling_allocator_msgs.srv import AllocateTrajectories
from builtin_interfaces.msg import Duration

TRAJ_TYPE_MAP = {
    'position': ['x', 'y', 'z'],
    'velocity': ['vx', 'vy', 'vz'],
    'attitude': ['pitch', 'roll', 'yaw'],
    'rates': ['pitch_rate', 'roll_rate', 'yaw_rate']
}

class Dashboard_Node(Node):
    def __init__(self):
        super().__init__('dashboard_node')

        # ROS Setup
        self.mission_start_publisher_ = self.create_publisher(Empty, 'mission_start', 10)
        self.mission_abort_publisher_ = self.create_publisher(Empty, 'mission_abort', 10)
        self.emergency_stop_publisher_ = self.create_publisher(Empty, 'emergency_stop', 10)

        self.trajectory_allocation_client_ = self.create_client(AllocateTrajectories, 'submit_trajectories')
        self.timer_period = 0.01  # seconds
        self.emergency_stop_timer = None

        self.current_allocated_trajectory = self.create_subscription(Allocations, 'current_allocated_trajectory', self.current_allocation_cb, 10)
        self.current_allocation = [] # List sorted by trajectory id

        self.current_vehicle_namespaces = []

        self.valid_methods = [
            'nearest',
            'random',
            'manual'
        ]
    
    def current_allocation_cb(self, msg):
        a_tuple = {a.trajectory_index: a.vehicle for a in msg.allocation}
        self.current_allocation = list(sorted(a_tuple, key=a_tuple.get))
    
    def get_current_allocation(self):
        return self.current_allocation

    def call_mission_start(self):
        msg = Empty()
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
        msg = Empty()
        self.emergency_stop_publisher_.publish(msg)
        self.get_logger().info('emergency_stop published')

    def call_mission_abort(self):
        msg = Empty()
        self.mission_abort_publisher_.publish(msg)
        self.get_logger().info('mission_abort published')

    def send_emergency_stop_drone(self, drone_namespace):
        msg = Empty()
        emergency_stop_publisher_ = self.create_publisher(Empty, f'/{drone_namespace}/emergency_stop', 10)
        emergency_stop_publisher_.publish(msg)
        self.get_logger().info(f'emergency_stop published to {drone_namespace}')
        emergency_stop_publisher_.destroy()

    def call_mission_abort_drone(self, drone_namespace):
        msg = Empty()
        mission_abort_publisher_ = self.create_publisher(Empty, f'/{drone_namespace}/mission_abort', 10)
        mission_abort_publisher_.publish(msg)
        self.get_logger().info(f'mission_abort published to {drone_namespace}')
        mission_abort_publisher_.destroy()

    def get_current_vehicle_namespaces(self):
        topic_list = self.get_topic_names_and_types()
        namespaces = set()
        # self.get_logger().info('Found the following topics:')
        for topic_name, _ in topic_list:
            # self.get_logger().info(topic_name)
            if 'mavros/state' in topic_name:
                name = topic_name.split('/')[1]
                if name == 'mavros':
                    name = ''
                namespaces.add(name)
        self.get_logger().info(f'Found {len(namespaces)} namespaces: {",".join(namespaces)}')
        self.current_vehicle_namespaces = list(namespaces)
        return self.current_vehicle_namespaces

    def is_valid_trajectory_dict(self, trajectory_dict):
        self.get_logger().info("testing validity of trajectory dictionary")
        types = []
        for traj in trajectory_dict:
            try:
                keys = traj['data'][0].keys()
            except Exception as e:
                self.get_logger().info('No data read in, empty trajectory')
                types.append('ERROR')
                continue
            found = False
            for typ, v in TRAJ_TYPE_MAP.items():
                if set(v).issubset(set(keys)):
                    types.append(typ)
                    found = True
                    break
            if not found:
                self.get_logger().warn('Trajectory keys do not match any trajectory type')
                types.append('ERROR')
        return types


    def send_trajectory_allocation(self, trajectory_dict, allocation_method="nearest", manual_alloc=[]):
        self.get_logger().info('sending allocate trajectory service request')

        while not self.trajectory_allocation_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        request = AllocateTrajectories.Request()

        validities = self.is_valid_trajectory_dict(trajectory_dict)
        if 'ERROR' in validities:
            self.get_logger().error('Trajectory with error exists, sending aborted')
            return

        rtraj = []
        for traj, valid in zip(trajectory_dict, validities):
            jt = JointTrajectory()
            jt.points = []
            for p in traj['data']:
                jtp = JointTrajectoryPoint()
                d = Duration()
                time = p['time']
                sec = int(np.floor(time))
                nanosec = int((time - sec) * 1e9)
                d.sec = sec
                d.nanosec = nanosec
                jtp.time_from_start = d

                point = []
                for k in TRAJ_TYPE_MAP[valid]:
                    point.append(p[k])

                if valid in ['position', 'velocity']:
                    if 'yaw' in p:
                        point.append(p['yaw'])
                    if 'yaw_rate' in p:
                        point.append(p['yaw_rate'])

                elif valid in ['attitude', 'rates']:
                    if 'thrust' in p:
                        point.append(p['thrust'])

                point = [float(p) for p in point]

                if valid in ['position', 'attitude']:
                    jtp.positions = point
                elif valid in ['velocity', 'rates']:
                    jtp.velocities = point
                jt.points.append(jtp)
            rtraj.append(jt)

        request.trajectories = rtraj
        request.trajectory_types = validities
        request.method = allocation_method
        if allocation_method=="manual":
            request.manual_allocation_targets = manual_alloc
        self.get_logger().info('The request is:\n' +str(request))
        future = self.trajectory_allocation_client_.call_async(request)
