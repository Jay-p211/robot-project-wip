# occupancy_grid/occupancy_grid/occupancy_grid_node.py
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
from rclpy.parameter import Parameter
from .occupancy_grid_map import OccupancyGridMap

class OccupancyGridNode(Node):
    def __init__(self):
        super().__init__('occupancy_grid')

        # Publisher with transient local durability
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pub = self.create_publisher(OccupancyGrid, '/map', qos)

        # Declare parameters
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('resolution', 0.05)
        self.declare_parameter('boundary', [0.0, 0.0, 10.0, 10.0])
        self.declare_parameter('blocks', [])

        frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        resolution = self.get_parameter('resolution').get_parameter_value().double_value
        boundary = self.get_parameter('boundary').get_parameter_value().double_array_value
        blocks = self.get_parameter('blocks').get_parameter_value().double_array_value

        # Build map
        ogm = OccupancyGridMap.empty_from_boundary(tuple(boundary), resolution)

        # Blocks array is 1x(4N): xmin, ymin, xmax, ymax for each block
        for k in range(0, len(blocks), 4):
            xmin, ymin, xmax, ymax = blocks[k:k+4]
            ogm.add_block(xmin, ymin, xmax, ymax)

        msg = ogm.to_msg(frame_id)
        self.pub.publish(msg)
        self.get_logger().info('Published occupancy grid')

def main(args=None):
    rclpy.init(args=args)
    node = OccupancyGridNode()
    try:
        rclpy.spin(node)  # keep node alive
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
