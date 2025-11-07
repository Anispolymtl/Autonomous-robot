#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapToCmap(Node):
    def __init__(self):
        super().__init__('map_to_cmap_node')

        # Thresholds
        self.M = 75   # obstacle threshold
        self.N = 35   # free threshold

        # Publisher & Subscriber
        self.publisher_ = self.create_publisher(OccupancyGrid, 'cmap', 10)
        self.subscription = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )

        self.get_logger().info('✅ Node started: listening on /map and publishing on /cmap')

    def map_callback(self, msg: OccupancyGrid):
        # Copy the incoming map
        cmap = OccupancyGrid()
        cmap.header = msg.header
        cmap.info = msg.info
        cmap.data = list(msg.data)  # convert from tuple to list for mutability

        height = cmap.info.height
        width = cmap.info.width

        # Apply conversion rules
        for y in range(height):
            for x in range(width):
                i = x + (height - 1 - y) * width
                val = cmap.data[i]

                if val >= 0:  # known cell
                    if val >= self.M:
                        cmap.data[i] = 100  # obstacle
                    elif val < self.N:
                        cmap.data[i] = 0    # free
                    else:
                        cmap.data[i] = -1
                else:
                    cmap.data[i] = -1      # unknown

        # Publish converted map
        cmap.data = tuple(cmap.data)
        self.publisher_.publish(cmap)
        self.get_logger().debug('Published /cmap')


def main(args=None):
    rclpy.init(args=args)
    node = MapToCmap()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
