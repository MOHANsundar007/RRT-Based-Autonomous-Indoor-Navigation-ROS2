#!/usr/bin/env python3
import math
import random
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Point
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker, MarkerArray



def _rgba(r, g, b, a=1.0) -> ColorRGBA:
    c = ColorRGBA()
    c.r, c.g, c.b, c.a = float(r), float(g), float(b), float(a)
    return c

_GRADIENT = [
    (0.00, (0.00, 0.95, 1.00)),  
    (0.30, (0.10, 1.00, 0.40)), 
    (0.60, (1.00, 0.90, 0.00)),  
    (1.00, (1.00, 0.40, 0.00)),   
]


def _depth_colour(t: float) -> ColorRGBA:
    t = max(0.0, min(1.0, t))
    for i in range(len(_GRADIENT) - 1):
        t0, c0 = _GRADIENT[i]
        t1, c1 = _GRADIENT[i + 1]
        if t <= t1:
            s = (t - t0) / (t1 - t0)
            r = c0[0] + s * (c1[0] - c0[0])
            g = c0[1] + s * (c1[1] - c0[1])
            b = c0[2] + s * (c1[2] - c0[2])
            return _rgba(r, g, b, 0.82)
    return _rgba(*_GRADIENT[-1][1], 0.82)

class RRTNode:
    __slots__ = ('x', 'y', 'parent', 'depth')

    def __init__(self, x: float, y: float):
        self.x      = x
        self.y      = y
        self.parent: 'RRTNode | None' = None
        self.depth  = 0            


class RRT:
    def __init__(self, max_iter=5000, step_size=0.2,
                 goal_bias=0.1, goal_tolerance=0.3):
        self.max_iter       = max_iter
        self.step_size      = step_size
        self.goal_bias      = goal_bias
        self.goal_tolerance = goal_tolerance
        self.viz_edges:    list = []
        self.viz_accepted: list = []
        self.viz_rejected: list = []
        self.max_depth:    int  = 1

    def plan(self, sx, sy, gx, gy, ox, oy, resolution, data, width, height):
        self.viz_edges    = []
        self.viz_accepted = []
        self.viz_rejected = []
        self.max_depth    = 1

        root       = RRTNode(sx, sy)
        root.depth = 0
        tree       = [root]

        x_min = ox;              x_max = ox + width  * resolution
        y_min = oy;              y_max = oy + height * resolution

        for _ in range(self.max_iter):
            if random.random() < self.goal_bias:
                rx, ry = gx, gy
            else:
                rx = random.uniform(x_min, x_max)
                ry = random.uniform(y_min, y_max)

            nearest  = self._nearest(tree, rx, ry)
            new_node = self._steer(nearest, rx, ry)

            if self._collision_free(nearest, new_node,
                                    ox, oy, resolution, data, width, height):
                new_node.parent = nearest
                new_node.depth  = nearest.depth + 1
                if new_node.depth > self.max_depth:
                    self.max_depth = new_node.depth
                tree.append(new_node)
                self.viz_edges.append(
                    (nearest.x, nearest.y, new_node.x, new_node.y,
                     new_node.depth))
                self.viz_accepted.append((rx, ry))

                if math.hypot(new_node.x - gx, new_node.y - gy) \
                        < self.goal_tolerance:
                    goal_node        = RRTNode(gx, gy)
                    goal_node.parent = new_node
                    goal_node.depth  = new_node.depth + 1
                    self.viz_edges.append(
                        (new_node.x, new_node.y, gx, gy, goal_node.depth))
                    return self._trace(goal_node)
            else:
                self.viz_rejected.append((rx, ry))

        return None

    def _nearest(self, tree, x, y):
        return min(tree, key=lambda n: math.hypot(n.x - x, n.y - y))

    def _steer(self, from_node, tx, ty):
        dx = tx - from_node.x;  dy = ty - from_node.y
        d  = math.hypot(dx, dy)
        if d < self.step_size:
            return RRTNode(tx, ty)
        r = self.step_size / d
        return RRTNode(from_node.x + dx * r, from_node.y + dy * r)

    def _collision_free(self, n1, n2, ox, oy, res, data, w, h):
        dist  = math.hypot(n2.x - n1.x, n2.y - n1.y)
        steps = max(int(dist / (res * 0.5)), 1)
        for i in range(steps + 1):
            t  = i / steps
            wx = n1.x + t * (n2.x - n1.x)
            wy = n1.y + t * (n2.y - n1.y)
            cx = int((wx - ox) / res);  cy = int((wy - oy) / res)
            if not (0 <= cx < w and 0 <= cy < h):
                return False
            if data[cy, cx] >= 90:
                return False
        return True

    def _trace(self, node):
        path = []
        while node:
            path.append((node.x, node.y))
            node = node.parent
        return list(reversed(path))
    
def _smooth_path(waypoints, n_out=200):
    if len(waypoints) < 4:
        return waypoints
    pts  = np.array(waypoints)
    d    = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    t_in = np.concatenate([[0.0], np.cumsum(d)])
    t_in = t_in / t_in[-1]
    t_out = np.linspace(0.0, 1.0, n_out)
    xs   = np.interp(t_out, t_in, pts[:, 0])
    ys   = np.interp(t_out, t_in, pts[:, 1])
    k    = np.array([1,2,4,8,16,8,4,2,1], dtype=float);  k /= k.sum()
    xs   = np.convolve(xs, k, mode='same')
    ys   = np.convolve(ys, k, mode='same')
    xs[0],  ys[0]  = waypoints[0]
    xs[-1], ys[-1] = waypoints[-1]
    return list(zip(xs.tolist(), ys.tolist()))

def _hdr(frame, stamp):
    h = Header();  h.frame_id = frame;  h.stamp = stamp;  return h


def _make_tree_marker_gradient(edges, max_depth, stamp):
    """
    LINE_LIST with per-vertex colours → depth gradient on every edge.
    Each edge gets two identical colours (one per endpoint) so RViz
    interpolates smoothly.
    """
    m = Marker()
    m.header  = _hdr('map', stamp)
    m.ns      = 'rrt_tree'
    m.id      = 0
    m.type    = Marker.LINE_LIST
    m.action  = Marker.ADD
    m.scale.x = 0.025          
    m.pose.orientation.w = 1.0
    denom = max(max_depth, 1)
    for (x1, y1, x2, y2, depth) in edges:
        t  = depth / denom
        c  = _depth_colour(t)
        p1 = Point();  p1.x = x1;  p1.y = y1;  p1.z = 0.03
        p2 = Point();  p2.x = x2;  p2.y = y2;  p2.z = 0.03
        m.points.append(p1);  m.colors.append(c)
        m.points.append(p2);  m.colors.append(c)
    return m


def _make_cylinder(x, y, r, g, b, a, radius, height, ns, mid, stamp, z=0.0):
    m = Marker()
    m.header  = _hdr('map', stamp)
    m.ns      = ns;  m.id = mid
    m.type    = Marker.CYLINDER
    m.action  = Marker.ADD
    m.pose.position.x    = float(x)
    m.pose.position.y    = float(y)
    m.pose.position.z    = float(z + height / 2.0)
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = float(radius * 2)
    m.scale.z = float(height)
    m.color   = _rgba(r, g, b, a)
    return m


def _make_pole(x, y, r, g, b, ns, mid, stamp):
    m = Marker()
    m.header  = _hdr('map', stamp)
    m.ns      = ns;  m.id = mid
    m.type    = Marker.CYLINDER
    m.action  = Marker.ADD
    m.pose.position.x    = float(x)
    m.pose.position.y    = float(y)
    m.pose.position.z    = 0.35
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = 0.04
    m.scale.z = 0.70
    m.color   = _rgba(r, g, b, 0.9)
    return m


def _make_halo(x, y, r, g, b, ns, mid, stamp):
    m = Marker()
    m.header  = _hdr('map', stamp)
    m.ns      = ns;  m.id = mid
    m.type    = Marker.CYLINDER
    m.action  = Marker.ADD
    m.pose.position.x    = float(x)
    m.pose.position.y    = float(y)
    m.pose.position.z    = 0.01
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = 0.80 
    m.scale.z = 0.02
    m.color   = _rgba(r, g, b, 0.45)
    return m


def _make_frontier_sphere(edges, stamp):
    if not edges:
        return None
    x2, y2 = edges[-1][2], edges[-1][3]
    m = Marker()
    m.header  = _hdr('map', stamp)
    m.ns      = 'rrt_frontier'
    m.id      = 0
    m.type    = Marker.SPHERE
    m.action  = Marker.ADD
    m.pose.position.x    = float(x2)
    m.pose.position.y    = float(y2)
    m.pose.position.z    = 0.06
    m.pose.orientation.w = 1.0
    m.scale.x = m.scale.y = m.scale.z = 0.18
    m.color   = _rgba(0.0, 1.0, 1.0, 0.95)   
    return m


def _make_progress_text(n_edges, sx, sy, stamp):
    m = Marker()
    m.header  = _hdr('map', stamp)
    m.ns      = 'rrt_label'
    m.id      = 0
    m.type    = Marker.TEXT_VIEW_FACING
    m.action  = Marker.ADD
    m.pose.position.x    = float(sx)
    m.pose.position.y    = float(sy) + 0.6
    m.pose.position.z    = 0.80
    m.pose.orientation.w = 1.0
    m.scale.z = 0.28         
    m.color   = _rgba(1.0, 1.0, 1.0, 0.95)
    m.text    = f'RRT  {n_edges} edges'
    return m


def _make_sample_array(accepted, rejected, stamp):
    arr     = MarkerArray()
    del_all = Marker()
    del_all.header = _hdr('map', stamp)
    del_all.ns     = 'rrt_samples'
    del_all.id     = 0
    del_all.action = Marker.DELETEALL
    arr.markers.append(del_all)

    mid = 1
   
    for (x, y) in accepted:
        m = Marker()
        m.header  = _hdr('map', stamp)
        m.ns      = 'rrt_samples';  m.id = mid
        m.type    = Marker.CUBE
        m.action  = Marker.ADD
        m.pose.position.x    = float(x)
        m.pose.position.y    = float(y)
        m.pose.position.z    = 0.005
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.06;  m.scale.z = 0.01
        m.color   = _rgba(0.0, 0.90, 1.0, 0.60)
        arr.markers.append(m);  mid += 1

   
    for (x, y) in rejected:
        m = Marker()
        m.header  = _hdr('map', stamp)
        m.ns      = 'rrt_samples';  m.id = mid
        m.type    = Marker.CUBE
        m.action  = Marker.ADD
        m.pose.position.x    = float(x)
        m.pose.position.y    = float(y)
        m.pose.position.z    = 0.005
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = 0.04;  m.scale.z = 0.01
        m.color   = _rgba(0.70, 0.05, 0.05, 0.45)
        arr.markers.append(m);  mid += 1

    return arr


def _nav_path(waypoints, stamp, z=0.0):
    path        = Path()
    path.header = _hdr('map', stamp)
    for (wx, wy) in waypoints:
        ps = PoseStamped()
        ps.header             = path.header
        ps.pose.position.x    = float(wx)
        ps.pose.position.y    = float(wy)
        ps.pose.position.z    = float(z)
        ps.pose.orientation.w = 1.0
        path.poses.append(ps)
    return path

class RRTPlannerNode(Node):

    def __init__(self):
        super().__init__('rrt_planner_node')

        self.declare_parameter('max_iterations', 5000)
        self.declare_parameter('step_size',      0.2)
        self.declare_parameter('goal_bias',      0.1)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('viz_batch_size', 60)
        self.declare_parameter('viz_tick_ms',    20)
        self.declare_parameter('smooth_path',    True)
        self.declare_parameter('smooth_points',  200)

        self._rrt = RRT(
            max_iter       = self.get_parameter('max_iterations').value,
            step_size      = self.get_parameter('step_size').value,
            goal_bias      = self.get_parameter('goal_bias').value,
            goal_tolerance = self.get_parameter('goal_tolerance').value,
        )

        self._map:        OccupancyGrid | None = None
        self._robot_pose: tuple | None         = None
        self._map_data:   np.ndarray | None    = None

        self._anim_lock      = threading.Lock()
        self._anim_edges     = []
        self._anim_idx       = 0
        self._anim_batch     = 60
        self._anim_timer     = None
        self._anim_sx        = 0.0
        self._anim_sy        = 0.0
        self._anim_gx        = 0.0
        self._anim_gy        = 0.0
        self._anim_stamp     = None
        self._anim_waypoints = []
        self._anim_max_depth = 1

        transient_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=1)
        viz_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE, depth=5)

        self.create_subscription(
            OccupancyGrid, '/map', self._map_cb, transient_qos)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 10)

        self._path_pub     = self.create_publisher(Path,        '/plan',                10)
        self._tree_pub     = self.create_publisher(MarkerArray, '/rrt_tree',            viz_qos)
        self._rrt_path_pub = self.create_publisher(Path,        '/rrt_path',            viz_qos)
        self._smooth_pub   = self.create_publisher(Path,        '/rrt_smoothed_path',   viz_qos)
        self._sample_pub   = self.create_publisher(MarkerArray, '/rrt_samples',         viz_qos)

        self.get_logger().info(
            'RRT Planner (Visualization v2) ready.\n'
            '  Topics: /rrt_tree  /rrt_path  /rrt_smoothed_path  /rrt_samples'
        )

    def _map_cb(self, msg: OccupancyGrid):
        self._map      = msg
        h, w           = msg.info.height, msg.info.width
        self._map_data = np.array(msg.data, dtype=np.int8).reshape(h, w)
        self.get_logger().info(
            f'Map received: {w}×{h}, res={msg.info.resolution:.3f} m')

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        self._robot_pose = (
            msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _goal_cb(self, msg: PoseStamped):
        if self._map is None:
            self.get_logger().warn('No map yet — cannot plan.'); return
        if self._robot_pose is None:
            self.get_logger().warn('No robot pose yet — cannot plan.'); return

        sx, sy = self._robot_pose
        gx     = msg.pose.position.x
        gy     = msg.pose.position.y
        info   = self._map.info
        ox, oy = info.origin.position.x, info.origin.position.y

        self.get_logger().info(
            f'RRT planning ({sx:.2f},{sy:.2f}) → ({gx:.2f},{gy:.2f})')

        t0        = time.time()
        waypoints = self._rrt.plan(
            sx, sy, gx, gy,
            ox, oy, info.resolution, self._map_data,
            info.width, info.height)
        elapsed   = time.time() - t0

        if waypoints is None:
            self.get_logger().warn(
                f'RRT failed after {self._rrt.max_iter} iterations.'); return

        self.get_logger().info(
            f'RRT found {len(waypoints)} waypoints in {elapsed:.2f}s '
            f'({len(self._rrt.viz_edges)} edges, depth={self._rrt.max_depth})')

        self._publish_path(waypoints)

        stamp = self.get_clock().now().to_msg()
        self._sample_pub.publish(
            _make_sample_array(self._rrt.viz_accepted,
                               self._rrt.viz_rejected, stamp))

        self._start_animation(sx, sy, gx, gy, waypoints, stamp)

    def _publish_path(self, waypoints):
        path                 = Path()
        path.header.frame_id = 'map'
        path.header.stamp    = self.get_clock().now().to_msg()
        for (wx, wy) in waypoints:
            ps                      = PoseStamped()
            ps.header               = path.header
            ps.pose.position.x      = wx
            ps.pose.position.y      = wy
            ps.pose.orientation.w   = 1.0
            path.poses.append(ps)
        self._path_pub.publish(path)

    def _start_animation(self, sx, sy, gx, gy, waypoints, stamp):
        batch = self.get_parameter('viz_batch_size').value
        tick  = self.get_parameter('viz_tick_ms').value

        with self._anim_lock:
            if self._anim_timer is not None:
                self._anim_timer.cancel()
                self._anim_timer = None

            self._anim_edges     = list(self._rrt.viz_edges)
            self._anim_max_depth = self._rrt.max_depth
            self._anim_idx       = 0
            self._anim_batch     = batch
            self._anim_sx        = sx
            self._anim_sy        = sy
            self._anim_gx        = gx
            self._anim_gy        = gy
            self._anim_stamp     = stamp
            self._anim_waypoints = waypoints
            self._anim_timer     = self.create_timer(
                tick / 1000.0, self._anim_tick)

    def _anim_tick(self):
        with self._anim_lock:
            end_idx = min(self._anim_idx + self._anim_batch,
                          len(self._anim_edges))
            visible = self._anim_edges[:end_idx]
            stamp   = self.get_clock().now().to_msg()
            arr     = MarkerArray()

            arr.markers.append(
                _make_tree_marker_gradient(
                    visible, self._anim_max_depth, stamp))

            frontier = _make_frontier_sphere(visible, stamp)
            if frontier:
                arr.markers.append(frontier)
            arr.markers.append(
                _make_progress_text(
                    len(visible),
                    self._anim_sx, self._anim_sy, stamp))
            arr.markers.append(
                _make_cylinder(self._anim_sx, self._anim_sy,
                               0.05, 0.90, 0.15, 0.95,
                               0.14, 0.08, 'rrt_startgoal', 1, stamp))
            arr.markers.append(
                _make_pole(self._anim_sx, self._anim_sy,
                           0.10, 0.95, 0.10, 'rrt_startgoal', 2, stamp))
            arr.markers.append(
                _make_cylinder(self._anim_gx, self._anim_gy,
                               0.95, 0.10, 0.10, 0.95,
                               0.14, 0.08, 'rrt_startgoal', 3, stamp))
            arr.markers.append(
                _make_pole(self._anim_gx, self._anim_gy,
                           0.95, 0.15, 0.10, 'rrt_startgoal', 4, stamp))
            arr.markers.append(
                _make_halo(self._anim_gx, self._anim_gy,
                           1.0, 0.30, 0.0, 'rrt_startgoal', 5, stamp))

            self._tree_pub.publish(arr)
            self._anim_idx = end_idx
            if self._anim_idx >= len(self._anim_edges):
                self._anim_timer.cancel()
                self._anim_timer = None
                self._rrt_path_pub.publish(
                    _nav_path(self._anim_waypoints, stamp, z=0.08))

                do_smooth = self.get_parameter('smooth_path').value
                if do_smooth and len(self._anim_waypoints) >= 4:
                    n_pts     = self.get_parameter('smooth_points').value
                    smooth_wp = _smooth_path(self._anim_waypoints, n_pts)
                    self._smooth_pub.publish(
                        _nav_path(smooth_wp, stamp, z=0.12))
                    self.get_logger().info(
                        f'Smoothed path: {len(smooth_wp)} pts → '
                        '/rrt_smoothed_path')
                else:
                    self._smooth_pub.publish(
                        _nav_path(self._anim_waypoints, stamp, z=0.12))

                self.get_logger().info('RRT visualization complete.')


def main(args=None):
    rclpy.init(args=args)
    node = RRTPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()