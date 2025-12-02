# Authors: Abdulkadir Ture
# Github : abdulkadrtr

import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def quaternion_to_yaw(q):
    """Retourne le yaw (rotation autour de Z) à partir d'un quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def rotate_point(x, y, yaw):
    """Rotation 2D d'un point autour de l'origine."""
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    return x * cos_yaw - y * sin_yaw, x * sin_yaw + y * cos_yaw


def compute_bounds_in_target(map_msg, transform):
    """Calcule le bounding box de la carte dans le repère cible (merge_map)."""
    res = map_msg.info.resolution
    width = map_msg.info.width
    height = map_msg.info.height
    ox = map_msg.info.origin.position.x
    oy = map_msg.info.origin.position.y

    yaw_origin = quaternion_to_yaw(map_msg.info.origin.orientation)
    yaw_tf = quaternion_to_yaw(transform.transform.rotation)
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y

    corners = [
        (0.0, 0.0),
        (width * res, 0.0),
        (0.0, height * res),
        (width * res, height * res),
    ]

    min_x = float("inf")
    min_y = float("inf")
    max_x = float("-inf")
    max_y = float("-inf")

    for cx, cy in corners:
        rx, ry = rotate_point(cx, cy, yaw_origin)
        px = ox + rx
        py = oy + ry

        px, py = rotate_point(px, py, yaw_tf)
        px += tx
        py += ty

        min_x = min(min_x, px)
        min_y = min(min_y, py)
        max_x = max(max_x, px)
        max_y = max(max_y, py)

    return min_x, min_y, max_x, max_y


def copy_map_into_merge(src_map, transform, merged_map, min_x, min_y, resolution, overwrite_unknown_only=False):
    """Projette chaque cellule de src_map dans merged_map en utilisant le TF fourni."""
    yaw_origin = quaternion_to_yaw(src_map.info.origin.orientation)
    yaw_tf = quaternion_to_yaw(transform.transform.rotation)
    tx = transform.transform.translation.x
    ty = transform.transform.translation.y

    ox = src_map.info.origin.position.x
    oy = src_map.info.origin.position.y
    src_res = src_map.info.resolution

    for y in range(src_map.info.height):
        for x in range(src_map.info.width):
            i = x + y * src_map.info.width
            cell_value = src_map.data[i]

            # Coordonnées de la cellule dans le repère map (en tenant compte de l'orientation de l'origine)
            rx, ry = rotate_point(x * src_res, y * src_res, yaw_origin)
            px = ox + rx
            py = oy + ry

            # Application du TF map -> merge_map
            px, py = rotate_point(px, py, yaw_tf)
            px += tx
            py += ty

            merged_x = int(math.floor((px - min_x) / resolution))
            merged_y = int(math.floor((py - min_y) / resolution))

            if (
                0 <= merged_x < merged_map.info.width
                and 0 <= merged_y < merged_map.info.height
            ):
                merged_i = merged_x + merged_y * merged_map.info.width
                if overwrite_unknown_only:
                    if merged_map.data[merged_i] == -1 and cell_value != -1:
                        merged_map.data[merged_i] = cell_value
                else:
                    merged_map.data[merged_i] = cell_value


def merge_maps(map1, tf1, map2, tf2, target_frame, stamp):
    """Fusionne les deux cartes en appliquant les TF vers target_frame."""
    bounds1 = compute_bounds_in_target(map1, tf1)
    bounds2 = compute_bounds_in_target(map2, tf2)

    min_x = min(bounds1[0], bounds2[0])
    min_y = min(bounds1[1], bounds2[1])
    max_x = max(bounds1[2], bounds2[2])
    max_y = max(bounds1[3], bounds2[3])

    merged_map = OccupancyGrid()
    merged_map.header.stamp = stamp.to_msg()
    merged_map.header.frame_id = target_frame

    merged_map.info.origin.position.x = min_x
    merged_map.info.origin.position.y = min_y
    merged_map.info.origin.orientation.w = 1.0

    merged_map.info.resolution = min(map1.info.resolution, map2.info.resolution)
    merged_map.info.width = max(
        1, int(math.ceil((max_x - min_x) / merged_map.info.resolution))
    )
    merged_map.info.height = max(
        1, int(math.ceil((max_y - min_y) / merged_map.info.resolution))
    )
    merged_map.data = [-1] * (merged_map.info.width * merged_map.info.height)

    copy_map_into_merge(
        map1, tf1, merged_map, min_x, min_y, merged_map.info.resolution
    )
    copy_map_into_merge(
        map2,
        tf2,
        merged_map,
        min_x,
        min_y,
        merged_map.info.resolution,
        overwrite_unknown_only=True,
    )

    return merged_map


class MergeMapNode(Node):
    def __init__(self):
        super().__init__("merge_map_node")

        # Permettre l'usage de /clock si demandé dans le launch
        self.declare_parameter("use_sim_time", False)

        self.target_frame = "merge_map"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(OccupancyGrid, "/merge_map", 10)
        self.map1_sub = self.create_subscription(
            OccupancyGrid, "/map1", self.map1_callback, 10
        )
        self.map2_sub = self.create_subscription(
            OccupancyGrid, "/map2", self.map2_callback, 10
        )
        self.map1 = None
        self.map2 = None

    def map1_callback(self, msg):
        self.map1 = msg
        self.try_merge()

    def map2_callback(self, msg):
        self.map2 = msg
        self.try_merge()

    def try_merge(self):
        if self.map1 is None or self.map2 is None:
            return

        try:
            tf1 = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.map1.header.frame_id,
                rclpy.time.Time(),
            )
            tf2 = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.map2.header.frame_id,
                rclpy.time.Time(),
            )
        except TransformException as ex:
            self.get_logger().warn(f"TF manquant pour la fusion des cartes: {ex}")
            return

        merged = merge_maps(
            self.map1, tf1, self.map2, tf2, self.target_frame, self.get_clock().now()
        )
        self.publisher.publish(merged)


def main(args=None):
    rclpy.init(args=args)
    merge_map_node = MergeMapNode()
    rclpy.spin(merge_map_node)
    merge_map_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()