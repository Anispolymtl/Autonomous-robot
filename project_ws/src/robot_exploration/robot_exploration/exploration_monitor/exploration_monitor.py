#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

import numpy as np
import json
from std_msgs.msg import Bool



class AlgoState:
    FRONTIER_DETECTION = "Frontier Detection"
    FRONTIER_CLUSTERING = "Frontier Clustering"
    CANDIDATE_EVALUATION = "Candidate Evaluation"
    IDLE = "Idle"


class ExplorationMonitor(Node):

    def __init__(self):
        super().__init__("exploration_monitor")

        # --- PARAM ---
        self.declare_parameter("robot_namespace", "limo1")
        self.robot_ns = self.get_parameter("robot_namespace").value

        self.map_topic = f"/{self.robot_ns}/map"
        self.map_frame = f"{self.robot_ns}/map"

        self.get_logger().info(f"🧠 Exploration Monitor démarré pour : {self.robot_ns}")
        self.get_logger().info(f"📡 Listening on topic: {self.map_topic}")
        self.get_logger().info(f"🖼️ Frame utilisé: {self.map_frame}")

        # --- Subscriptions ---
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )
        
        self.exploring = False
        self.resume_sub = self.create_subscription(
            Bool,
            'explore/resume',
            self.resume_callback,
            10
        )

        # --- Publishers ---
        self.step_pub = self.create_publisher(String, "exploration_step", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "frontiers_markers", 10)
        self.debug_pub = self.create_publisher(String, "exploration_debug", 10)
        self.candidate_pub = self.create_publisher(Point, "candidate_frontier", 10)

        self.timer = self.create_timer(1.0, self.loop)

        self.current_map = None
        self.current_state = AlgoState.IDLE

        self.frontiers = []
        self.clusters = []
        self.selected_frontier = None

    # -------------------------------------------------
    def map_callback(self, msg):
        self.current_map = msg

    # -------------------------------------------------
    def resume_callback(self, msg: Bool):
        self.exploring = msg.data

        if self.exploring:
            self.get_logger().info("🟢 Exploration démarrée → activation du monitor")
        else:
            self.get_logger().info("🔴 Exploration arrêtée → monitor en pause")

    # -------------------------------------------------
    def loop(self):
        if self.current_map is None:
            return

        if not self.exploring:
            self.publish_idle()
            return

        self.detect_frontiers()
        self.cluster_frontiers()
        self.select_frontier()
        self.publish_debug()
        self.publish_markers()

    # -------------------------------------------------

    def publish_idle(self):
        if self.current_state != AlgoState.IDLE:
            self.current_state = AlgoState.IDLE
            self.publish_state()
            self.get_logger().info("⏸️ Monitor en mode IDLE (exploration stoppée)")

    # -------------------------------------------------

    def detect_frontiers(self):
        self.current_state = AlgoState.FRONTIER_DETECTION
        self.publish_state()

        self.get_logger().info(f"🔍 [{self.robot_ns}] Détection des frontiers en cours...")

        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )

        frontiers = []

        for y in range(1, map_data.shape[0] - 1):
            for x in range(1, map_data.shape[1] - 1):

                if map_data[y, x] == 0:
                    neighbors = [
                        map_data[y+1,x],
                        map_data[y-1,x],
                        map_data[y,x+1],
                        map_data[y,x-1]
                    ]

                    if -1 in neighbors:
                        wx = self.current_map.info.origin.position.x + x * self.current_map.info.resolution
                        wy = self.current_map.info.origin.position.y + y * self.current_map.info.resolution
                        frontiers.append([wx, wy])

        self.frontiers = frontiers

        self.get_logger().info(
            f"✅ [{self.robot_ns}] Frontiers détectés : {len(self.frontiers)} points"
        )

    # -------------------------------------------------
    def cluster_frontiers(self):
        self.current_state = AlgoState.FRONTIER_CLUSTERING
        self.publish_state()

        if len(self.frontiers) < 5:
            self.clusters = []
            self.get_logger().info("🔵 Pas assez de frontiers pour clusteriser")
            return

        # On regroupe tout en un cluster pour l'instant
        self.clusters = [self.frontiers]
        self.get_logger().info(f"🟣 1 cluster simple créé ({len(self.frontiers)} points)")

    # -------------------------------------------------
    def select_frontier(self):
        self.current_state = AlgoState.CANDIDATE_EVALUATION
        self.publish_state()

        self.get_logger().info(f"🎯 [{self.robot_ns}] Sélection du frontier à explorer...")

        if not self.clusters:
            self.selected_frontier = None
            self.get_logger().warn(
                f"❌ [{self.robot_ns}] Aucun cluster valide → impossible de choisir un frontier"
            )
            return

        biggest_cluster = max(self.clusters, key=len)

        avg_x = np.mean([p[0] for p in biggest_cluster])
        avg_y = np.mean([p[1] for p in biggest_cluster])

        self.selected_frontier = [avg_x, avg_y]

        self.get_logger().info(
            f"✅ [{self.robot_ns}] Frontier sélectionné -> x={avg_x:.2f}, y={avg_y:.2f}"
        )

        msg = Point()
        msg.x = avg_x
        msg.y = avg_y
        msg.z = 0.0

        self.candidate_pub.publish(msg)

    # -------------------------------------------------
    def publish_debug(self):
        data = {
            "robot": self.robot_ns,
            "state": self.current_state,
            "frontiers_detected": len(self.frontiers),
            "clusters": len(self.clusters),
            "selected_frontier": self.selected_frontier
        }

        msg = String()
        msg.data = json.dumps(data)
        self.debug_pub.publish(msg)

    # -------------------------------------------------
    def publish_markers(self):
        markers = MarkerArray()

        marker = Marker()
        marker.header.frame_id = self.map_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "frontiers"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        for p in self.frontiers:
            point = Point()
            point.x = p[0]
            point.y = p[1]
            point.z = 0.0
            marker.points.append(point)

        markers.markers.append(marker)

        if self.selected_frontier:
            sel = Marker()
            sel.header.frame_id = self.map_frame
            sel.ns = "selected"
            sel.id = 1
            sel.type = Marker.SPHERE
            sel.scale.x = 0.3
            sel.scale.y = 0.3
            sel.scale.z = 0.3
            sel.color.r = 1.0
            sel.color.g = 0.0
            sel.color.b = 0.0
            sel.color.a = 1.0

            sel.pose.position.x = self.selected_frontier[0]
            sel.pose.position.y = self.selected_frontier[1]
            sel.pose.position.z = 0.0

            markers.markers.append(sel)

        self.marker_pub.publish(markers)

    # -------------------------------------------------
    def publish_state(self):
        self.get_logger().info(f"📍 État actuel : {self.current_state}")

        msg = String()
        msg.data = self.current_state
        self.step_pub.publish(msg)


def main():
    rclpy.init()
    node = ExplorationMonitor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()