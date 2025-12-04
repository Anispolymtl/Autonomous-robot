#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray
import numpy as np
import json
import time

class AlgoState:
    FRONTIER_DETECTION = "Frontier Detection"
    FRONTIER_CLUSTERING = "Frontier Clustering"
    CANDIDATE_EVALUATION = "Candidate Evaluation"
    IDLE = "Idle"


class ExplorationMonitor(Node):
    def __init__(self):
        super().__init__("exploration_monitor")

        # PARAMS
        self.declare_parameter("robot_namespace", "limo1")
        self.robot_ns = self.get_parameter("robot_namespace").value

        self.map_topic = f"/{self.robot_ns}/map"
        self.map_frame = f"{self.robot_ns}/map"

        self.get_logger().info(f"🧠 Exploration Monitor pour : {self.robot_ns}")

        # SUBS
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )
        self.resume_sub = self.create_subscription(
            Bool,
            "explore/resume",
            self.resume_callback,
            10
        )

        # PUBS
        self.step_pub = self.create_publisher(String, "exploration_step", 10)
        self.debug_pub = self.create_publisher(String, "exploration_debug", 10)

        self.timer = self.create_timer(1.0, self.loop)

        self.exploring = False
        self.current_map = None

        self.current_state = AlgoState.IDLE
        self.last_state_change_time = time.time()
        self.step_delay = 3.0  # secondes pour visualisation claire

        self.frontiers = []
        self.clusters = []
        self.selected_frontier = None

    def map_callback(self, msg):
        self.current_map = msg

    def resume_callback(self, msg: Bool):
        self.exploring = msg.data
        self.last_state_change_time = time.time()  # reset timer

        if not self.exploring:
            self.publish_state(AlgoState.IDLE)

    def loop(self):
        if self.current_map is None:
            return

        if not self.exploring:
            return

        now = time.time()
        if now - self.last_state_change_time < self.step_delay:
            return  # On attend pour la visualisation

        # SWITCH STATE → Transition contrôlée
        if self.current_state == AlgoState.FRONTIER_DETECTION:
            self.cluster_and_publish()
        elif self.current_state == AlgoState.FRONTIER_CLUSTERING:
            self.evaluate_and_publish()
        else:
            self.detect_and_publish()

        self.last_state_change_time = time.time()  # Reset timer

        self.publish_debug()

    # ------------------ LOGIC ------------------ #

    def detect_and_publish(self):
        self.current_state = AlgoState.FRONTIER_DETECTION
        self.publish_state()

        map_data = np.array(self.current_map.data).reshape(
            self.current_map.info.height,
            self.current_map.info.width
        )

        self.frontiers.clear()
        for y in range(1, map_data.shape[0]-1):
            for x in range(1, map_data.shape[1]-1):
                if map_data[y,x] == 0:
                    if -1 in [
                        map_data[y+1,x],
                        map_data[y-1,x],
                        map_data[y,x+1],
                        map_data[y,x-1]
                    ]:
                        wx = self.current_map.info.origin.position.x + x * self.current_map.info.resolution
                        wy = self.current_map.info.origin.position.y + y * self.current_map.info.resolution
                        self.frontiers.append((wx, wy))

        self.get_logger().info(f"🔍 {len(self.frontiers)} frontiers détectés")

    def cluster_and_publish(self):
        self.current_state = AlgoState.FRONTIER_CLUSTERING
        self.publish_state()

        if len(self.frontiers) < 5:
            self.clusters = []
        else:
            self.clusters = [self.frontiers]

        self.get_logger().info("🟣 Clustering complété")

    def evaluate_and_publish(self):
        self.current_state = AlgoState.CANDIDATE_EVALUATION
        self.publish_state()

        if not self.clusters:
            self.selected_frontier = None
            self.get_logger().warn("❌ Aucun cluster valide")
            return

        biggest = max(self.clusters, key=len)
        xs = [p[0] for p in biggest]
        ys = [p[1] for p in biggest]
        self.selected_frontier = (np.mean(xs), np.mean(ys))

        self.get_logger().info("🎯 Frontier choisi")

    # ------------------ PUB HELPERS ------------------ #

    def publish_state(self, state=None):
        if state:
            self.current_state = state

        msg = String()
        msg.data = self.current_state
        self.step_pub.publish(msg)

        self.get_logger().info(f"📍 État : {self.current_state}")

    def publish_debug(self):
        msg = String()
        data = {
            "robot": self.robot_ns,
            "state": self.current_state,
            "frontiers": len(self.frontiers),
            "clusters": len(self.clusters),
            "candidate": self.selected_frontier
        }
        msg.data = json.dumps(data)
        self.debug_pub.publish(msg)


def main():
    rclpy.init()
    rclpy.spin(ExplorationMonitor())
    rclpy.shutdown()


if __name__ == "__main__":
    main()