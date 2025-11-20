import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np


def merge_maps_with_tf(map1, map2, tf1, tf2):
    merged_map = OccupancyGrid()
    merged_map.header.frame_id = "map"
    merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution)

    def extract_xy(tf: TransformStamped):
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        return x, y

    map1_offset = extract_xy(tf1)
    map2_offset = extract_xy(tf2)

    # Détermination des limites globales
    min_x = min(map1_offset[0] + map1.info.origin.position.x,
                map2_offset[0] + map2.info.origin.position.x)

    min_y = min(map1_offset[1] + map1.info.origin.position.y,
                map2_offset[1] + map2.info.origin.position.y)

    max_x = max(map1_offset[0] + map1.info.origin.position.x + map1.info.width * map1.info.resolution,
                map2_offset[0] + map2.info.origin.position.x + map2.info.width * map2.info.resolution)

    max_y = max(map1_offset[1] + map1.info.origin.position.y + map1.info.height * map1.info.resolution,
                map2_offset[1] + map2.info.origin.position.y + map2.info.height * map2.info.resolution)

    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.width = int((max_x - min_x) / merged_map.info.resolution)
    merged_map.info.height = int((max_y - min_y) / merged_map.info.resolution)

    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

    def insert_map(source_map, offset):
        for y in range(source_map.info.height):
            for x in range(source_map.info.width):

                i = x + y * source_map.info.width
                cell = source_map.data[i]

                if cell == -1:
                    continue

                world_x = offset[0] + source_map.info.origin.position.x + x * source_map.info.resolution
                world_y = offset[1] + source_map.info.origin.position.y + y * source_map.info.resolution

                merged_x = int((world_x - min_x) / merged_map.info.resolution)
                merged_y = int((world_y - min_y) / merged_map.info.resolution)

                if 0 <= merged_x < merged_map.info.width and 0 <= merged_y < merged_map.info.height:
                    merged_i = merged_x + merged_y * merged_map.info.width

                    if merged_map.data[merged_i] == -1:
                        merged_map.data[merged_i] = cell
                    else:
                        merged_map.data[merged_i] = max(merged_map.data[merged_i], cell)

    insert_map(map1, map1_offset)
    insert_map(map2, map2_offset)

    return merged_map


class MergeMapNode(Node):
    def __init__(self):
        super().__init__('merge_map_node')

        self.publisher = self.create_publisher(
            OccupancyGrid,
            "/merged_map",
            10
        )

        self.sub_map1 = self.create_subscription(
            OccupancyGrid,
            "/limo1/map",
            self.map1_callback,
            10
        )

        self.sub_map2 = self.create_subscription(
            OccupancyGrid,
            "/limo2/map",
            self.map2_callback,
            10
        )

        self.map1 = None
        self.map2 = None

        # TF handler
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("✅ MergeMapNode avec TF dynamique prêt.")

    def map1_callback(self, msg):
        self.map1 = msg
        self.try_merge()

    def map2_callback(self, msg):
        self.map2 = msg
        self.try_merge()

    def get_transform(self, target_frame, source_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            return transform

        except Exception as e:
            self.get_logger().warn(
                f"TF non disponible: {target_frame} → {source_frame}"
            )
            return None

    def try_merge(self):
        if self.map1 is None or self.map2 is None:
            return

        tf1 = self.get_transform("map", self.map1.header.frame_id)
        tf2 = self.get_transform("map", self.map2.header.frame_id)

        if tf1 is None or tf2 is None:
            return

        merged = merge_maps_with_tf(self.map1, self.map2, tf1, tf2)
        self.publisher.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    node = MergeMapNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()