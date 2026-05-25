#!/usr/bin/env python3
"""
─────────────────────────────
This script:
  1. Publishes a fixed initial pose to /initialpose   (AMCL localisation)
  2. Waits for AMCL to converge
  3. Sends a single NavigateToPose goal (the fixed end point)
  4. Tracks and logs all performance metrics to a JSON results file
     for later graph generation with plot_results.py

Usage:
  ros2 run path_planning_pkg fixed_start_goal_navigator
  or directly 
  python3 fixed_start_goal_navigator.py
"""

import json
import math
import os
import time
from datetime import datetime

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

START_X:   float =  -4.0
START_Y:   float = -3.60
START_YAW: float =  90.0   

GOAL_X:   float =  3.6
GOAL_Y:   float =  3.75
GOAL_YAW: float =  90.0



ALGORITHM: str = "RRT"  
RESULTS_FILE: str = os.path.expanduser("~/nav_results.json")


AMCL_SETTLE_SEC: float = 4.0
def _yaw_to_quat(yaw_deg: float) -> tuple[float, float, float, float]:
    """Yaw (degrees) → quaternion (x, y, z, w)."""
    yaw = math.radians(yaw_deg)
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FixedStartGoalNavigator(Node):
    """
    Publishes a fixed initial pose, waits for AMCL to settle,
    then sends a single fixed goal and records all metrics.
    """

    def __init__(self):
        super().__init__("fixed_start_goal_navigator")
        global ALGORITHM
        self.declare_parameter("algorithm", ALGORITHM)
        ALGORITHM = self.get_parameter("algorithm").value
        _alias = {"rrt": "RRT",}
        ALGORITHM = _alias.get(ALGORITHM.lower(), ALGORITHM)

        
        self._nav_active: bool = False
        self._nav_start_time: float | None = None

        self._last_odom: tuple[float, float] | None = None
        self._total_distance: float = 0.0
        self._recovery_count: int = 0
        self._planned_path_length: float = 0.0

        
        self._amcl_pose: tuple[float, float] | None = None
        self._goal_pose: tuple[float, float] = (GOAL_X, GOAL_Y)

        
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=10,
        )

        initialpose_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )

        cb_group = ReentrantCallbackGroup()
        
        self._initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", initialpose_qos
        )
        self._nav_client = ActionClient(
            self, NavigateToPose, "navigate_to_pose", callback_group=cb_group
        )
        self.create_subscription(Odometry, "/odom", self._odom_cb, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_cb, reliable_qos
        )
        self.create_subscription(Path, "/plan", self._plan_cb, 10)
        self.create_subscription(String, "/recovery_execution", self._recovery_cb, 10)
        self.get_logger().info(
            f"\n{'='*55}\n"
            f"  Fixed Start-Goal Navigator\n"
            f"  Algorithm : {ALGORITHM}\n"
            f"  Start     : ({START_X}, {START_Y}, {START_YAW}°)\n"
            f"  Goal      : ({GOAL_X}, {GOAL_Y}, {GOAL_YAW}°)\n"
            f"{'='*55}"
        )
        self._publish_initial_pose()
        self._startup_timer = self.create_timer(
            AMCL_SETTLE_SEC,
            self._send_goal_once,
        )
    def _publish_initial_pose(self):
        qx, qy, qz, qw = _yaw_to_quat(START_YAW)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = "map"
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.pose.pose.position.x = START_X
        msg.pose.pose.position.y = START_Y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.pose.covariance[0]  = 0.05   # xx
        msg.pose.covariance[7]  = 0.05   # yy
        msg.pose.covariance[35] = 0.068  # yaw·yaw

        self._initial_pose_pub.publish(msg)
        self.get_logger().info(
            f"[1/3] Initial pose published → ({START_X}, {START_Y}, {START_YAW}°). "
            f"Waiting {AMCL_SETTLE_SEC}s for AMCL…"
        )
    def _send_goal_once(self):
        self._startup_timer.cancel()

        self.get_logger().info("[2/3] Connecting to NavigateToPose action server…")
        if not self._nav_client.wait_for_server(timeout_sec=15.0):
            self.get_logger().error("NavigateToPose server not available — aborting.")
            return

        qx, qy, qz, qw = _yaw_to_quat(GOAL_YAW)
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = GOAL_X
        pose.pose.position.y = GOAL_Y
        pose.pose.position.z = 0.0
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        self.get_logger().info(
            f"[3/3] Sending goal → ({GOAL_X}, {GOAL_Y}, {GOAL_YAW}°)"
        )
        future = self._nav_client.send_goal_async(
            goal_msg, feedback_callback=self._feedback_cb
        )
        future.add_done_callback(self._goal_response_cb)
    def _odom_cb(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._nav_active and self._last_odom is not None:
            dx = x - self._last_odom[0]
            dy = y - self._last_odom[1]
            self._total_distance += math.hypot(dx, dy)
        self._last_odom = (x, y)

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
      
        if self._amcl_pose is None:
            self._amcl_pose = (
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
            )
            self.get_logger().info(
                f"Start pose locked from AMCL: "
                f"({self._amcl_pose[0]:.3f}, {self._amcl_pose[1]:.3f})"
            )

    def _plan_cb(self, msg: Path):
        if len(msg.poses) < 2:
            return
        length = 0.0
        for i in range(1, len(msg.poses)):
            p1 = msg.poses[i - 1].pose.position
            p2 = msg.poses[i].pose.position
            length += math.hypot(p2.x - p1.x, p2.y - p1.y)

        
        if self._planned_path_length == 0.0:
            self._planned_path_length = length
            self.get_logger().info(
                f"Initial plan received — planned length: {length:.3f} m"
            )
        else:
            self.get_logger().debug(
                f"Replan received — length: {length:.3f} m (ignored for metrics)"
            )

    def _recovery_cb(self, _msg: String):
        self._recovery_count += 1
        self.get_logger().warn(f"Recovery triggered (total: {self._recovery_count})")

    def _feedback_cb(self, feedback_msg):
        dist = feedback_msg.feedback.distance_remaining
        self.get_logger().debug(f"Distance remaining: {dist:.2f} m")

    def _goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error("Goal rejected by Nav2!")
            return
        self.get_logger().info("Goal accepted — navigation started.")
        self._nav_active = True
        self._nav_start_time = time.time()
        handle.get_result_async().add_done_callback(self._result_cb)

    def _result_cb(self, future):
        self._nav_active = False
        elapsed = time.time() - self._nav_start_time if self._nav_start_time else 0.0
        status = future.result().status
        start_xy = self._amcl_pose or (START_X, START_Y)
        straight_line = math.hypot(
            GOAL_X - start_xy[0],
            GOAL_Y - start_xy[1],
        )
        efficiency = (
            (straight_line / self._total_distance * 100.0)
            if self._total_distance > 0
            else 0.0
        )

        status_str = {
            GoalStatus.STATUS_SUCCEEDED: "SUCCEEDED",
            GoalStatus.STATUS_ABORTED:   "ABORTED",
            GoalStatus.STATUS_CANCELED:  "CANCELED",
        }.get(status, "UNKNOWN")

        sep = "=" * 60
        self.get_logger().info(
            f"\n{sep}\n"
            f"  NAVIGATION COMPLETE — {status_str}\n"
            f"{sep}\n"
            f"  Algorithm                    : {ALGORITHM}\n"
            f"  Start point                  : ({START_X}, {START_Y}, {START_YAW}°)\n"
            f"  Goal  point                  : ({GOAL_X},  {GOAL_Y},  {GOAL_YAW}°)\n"
            f"  ─────────────────────────────────────────────────────\n"
            f"  Total path length (odometry) : {self._total_distance:.3f} m\n"
            f"  Planned path length          : {self._planned_path_length:.3f} m\n"
            f"  Straight-line distance       : {straight_line:.3f} m\n"
            f"  Path efficiency              : {efficiency:.1f}  %\n"
            f"  Total navigation time        : {elapsed:.2f}  s\n"
            f"  Recovery behaviors           : {self._recovery_count}\n"
            f"{sep}"
        )

        result = {
            "timestamp":             datetime.now().isoformat(),
            "algorithm":             ALGORITHM,
            "start":                 {"x": START_X, "y": START_Y, "yaw": START_YAW},
            "goal":                  {"x": GOAL_X,  "y": GOAL_Y,  "yaw": GOAL_YAW},
            "status":                status_str,
            "actual_path_length_m":  round(self._total_distance, 4),
            "planned_path_length_m": round(self._planned_path_length, 4),
            "straight_line_m":       round(straight_line, 4),
            "path_efficiency_pct":   round(efficiency, 2),
            "navigation_time_s":     round(elapsed, 3),
            "recovery_count":        self._recovery_count,
        }

        results = []
        if os.path.exists(RESULTS_FILE):
            with open(RESULTS_FILE) as f:
                try:
                    results = json.load(f)
                except json.JSONDecodeError:
                    results = []

        results.append(result)
        with open(RESULTS_FILE, "w") as f:
            json.dump(results, f, indent=2)

        self.get_logger().info(f"Results saved → {RESULTS_FILE}")


def main(args=None):
    rclpy.init(args=args)
    node = FixedStartGoalNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()