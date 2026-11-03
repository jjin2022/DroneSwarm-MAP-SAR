"""Patrol a square with lidar obstacle avoidance, mapping and object tracing, then
return home ALONG the square when the battery gets low.

Built on patrol_rtl (same states, PX4 handling and safety checks). Adds:

  Obstacle avoidance
      Each tick, pick the heading closest to the goal whose corridor (drone width
      + margin = avoid_radius_m on each side) is free for avoid_lookahead_m. Uses
      the live scan plus obstacles already in the map, because the lidar is blind
      in the rear 90 deg. The nose turns to face the direction of travel so the
      lidar looks where the drone is going. Backs off if something is closer than
      half of avoid_radius_m, holds if boxed in, and skips a waypoint it has made
      no progress toward for stuck_timeout_s.

  Mapping
      Log-odds occupancy grid. Published on /map (nav_msgs/OccupancyGrid) and saved
      to map_dir as lidar_map_<time>.pgm/.yaml (ROS map_server format) at the end.
      Beams that hit the ground because the drone is tilted are recognised using
      the vehicle attitude and only clear space, so they don't show up as walls.

  Object tracing
      Once a second, occupied map cells that touch (small gaps allowed) are
      grouped into objects. An object keeps its ID while its cells overlap the
      previous update's, and is confirmed after several updates in a row, so
      brief noise never becomes an object. Working from the map rather than raw
      scans means stray ground hits (drone tilted) can't chain objects together.
      Objects are logged, published on /objects (MarkerArray: the traced cells
      plus a label) and saved to lidar_map_<time>_objects.csv.

  Return along the square
      On low battery, fly the remaining corners (whichever way round is shorter)
      back to the first corner, then the takeoff leg home, staying at patrol
      altitude so the lidar keeps scanning. Never cuts across the middle.

Needs the lidar bridge running:
    ros2 launch las_offboard lidar_bridge.launch.py world:=lidar_test
View in RViz: Fixed Frame "map", Add > By topic > /map (Map) and /objects (MarkerArray).

Frames: PX4 local position is NED (x = north, y = east, z = down). The map,
markers and log messages use ENU relative to home (E = east, N = north, metres),
the same as the Gazebo world file.
"""
import array
import math
import os
import time

import numpy as np
import rclpy
from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                       HistoryPolicy, qos_profile_sensor_data)
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitude

from las_offboard.patrol_rtl import (PatrolRtl, RATE_HZ, NAN, FLYING_STATES,
                                     STREAMING_STATES)

# Where the lidar sits on the x500_lidar_2d, in the body FRD frame (x forward,
# y right, z down): 0.12 m forward and 0.26 + 0.055 m up (see model.sdf).
LIDAR_OFFSET_FRD = np.array([0.12, 0.0, -0.315])
# A beam with no return is treated as free space out to this distance.
FREE_RANGE_M = 8.0
# States where the drone follows the avoidance setpoint (self.cmd), not self.target.
AVOID_STATES = ('PATROL', 'RETURN')


def quat_to_rot(q):
    """Rotation matrix from a PX4 quaternion (w, x, y, z), FRD body -> NED."""
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


# ======================================================================
# Mapping
# ======================================================================
class OccupancyMap:
    """Square log-odds occupancy grid in ENU metres, centred on home.

    Each cell holds log(p / (1 - p)) for "occupied". A beam passing through a
    cell lowers it (free); the cell a beam ends in is raised (hit).
    """
    L_HIT = 0.85       # added where a beam ends on an obstacle
    L_MISS = -0.4      # added to cells a beam passes through
    L_CLAMP = 4.0      # limit, so a cell can still change its mind
    L_OCCUPIED = 1.0   # above this a cell counts as an obstacle (p ~ 0.73)

    def __init__(self, size_m, resolution_m):
        self.res = float(resolution_m)
        self.n = int(round(size_m / resolution_m))
        self.origin = -self.n * self.res / 2.0       # ENU of the grid's lower-left corner
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)   # [row = north, col = east]
        self.seen = np.zeros((self.n, self.n), dtype=bool)

    def _flat_index(self, e, n):
        """Unique flat indices of the cells containing points (e, n); outside points dropped."""
        col = np.floor((e - self.origin) / self.res).astype(np.int64)
        row = np.floor((n - self.origin) / self.res).astype(np.int64)
        inside = (col >= 0) & (col < self.n) & (row >= 0) & (row < self.n)
        return np.unique(row[inside] * self.n + col[inside])

    def update(self, oe, on, end_e, end_n, is_hit, stride=3):
        """Add one scan: beams from the lidar at (oe, on) to (end_e, end_n).

        Free space is traced on every `stride`-th beam to save time; every hit is used.
        """
        de, dn = end_e[::stride] - oe, end_n[::stride] - on
        length = np.hypot(de, dn)
        if length.size:
            steps = np.arange(0.0, float(length.max()), self.res / 2.0)
            frac = steps[None, :] / np.maximum(length[:, None], 1e-6)
            before_end = steps[None, :] < (length[:, None] - self.res)   # stop a cell short
            free = self._flat_index((oe + de[:, None] * frac)[before_end],
                                    (on + dn[:, None] * frac)[before_end])
        else:
            free = np.zeros(0, dtype=np.int64)
        hit = self._flat_index(end_e[is_hit], end_n[is_hit])

        cells = self.logodds.reshape(-1)    # views, so writes go into the grid
        seen = self.seen.reshape(-1)
        cells[free] += self.L_MISS
        cells[hit] += self.L_HIT
        np.clip(cells, -self.L_CLAMP, self.L_CLAMP, out=cells)
        seen[free] = True
        seen[hit] = True

    def occupied_near(self, e, n, radius):
        """ENU centres (M, 2) of occupied cells in a square window around (e, n)."""
        c0 = max(int((e - radius - self.origin) / self.res), 0)
        c1 = min(int((e + radius - self.origin) / self.res) + 1, self.n)
        r0 = max(int((n - radius - self.origin) / self.res), 0)
        r1 = min(int((n + radius - self.origin) / self.res) + 1, self.n)
        if c0 >= c1 or r0 >= r1:
            return np.zeros((0, 2))
        rows, cols = np.nonzero(self.logodds[r0:r1, c0:c1] > self.L_OCCUPIED)
        return np.stack([self.origin + (cols + c0 + 0.5) * self.res,
                         self.origin + (rows + r0 + 0.5) * self.res], axis=1)

    def to_msg(self, header):
        """nav_msgs/OccupancyGrid: -1 unknown, 0..100 occupancy probability."""
        msg = OccupancyGrid()
        msg.header = header
        msg.info.resolution = self.res
        msg.info.width = msg.info.height = self.n
        msg.info.origin.position.x = msg.info.origin.position.y = self.origin
        msg.info.origin.orientation.w = 1.0
        prob = 1.0 - 1.0 / (1.0 + np.exp(self.logodds))
        data = np.where(self.seen, np.round(prob * 100), -1).astype(np.int8)
        msg.data = array.array('b', data.tobytes())
        return msg

    def save(self, base):
        """Write base.pgm + base.yaml (map_server format: black occupied, white free)."""
        img = np.full(self.logodds.shape, 205, dtype=np.uint8)          # grey = unknown
        img[self.seen & (self.logodds <= 0.0)] = 254                      # white = free
        img[self.logodds > self.L_OCCUPIED] = 0                           # black = occupied
        img = np.flipud(img)                                              # image row 0 = north edge
        with open(base + '.pgm', 'wb') as f:
            f.write(f'P5\n{self.n} {self.n}\n255\n'.encode())
            f.write(img.tobytes())
        with open(base + '.yaml', 'w') as f:
            f.write(f'image: {os.path.basename(base)}.pgm\n'
                    f'resolution: {self.res}\n'
                    f'origin: [{self.origin}, {self.origin}, 0.0]\n'
                    'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\nmode: trinary\n')


# ======================================================================
# Object tracing
# ======================================================================
class TracedObject:
    """One object: a group of occupied map cells."""
    def __init__(self, oid):
        self.id = oid
        self.updates = 0          # map updates it has been present in
        self.missed = 0           # updates in a row it has been missing from the map
        self.confirmed = False
        self.cells = frozenset()  # flat map indices
        self.points = np.zeros((0, 2))
        self.box = [0.0, 0.0, 0.0, 0.0]   # [e_min, e_max, n_min, n_max], ENU metres

    def set_cells(self, cells, points, res):
        self.cells = cells
        self.points = points                # ENU cell centres, (K, 2)
        h = res / 2.0
        self.box = [points[:, 0].min() - h, points[:, 0].max() + h,
                    points[:, 1].min() - h, points[:, 1].max() + h]
        self.updates += 1
        self.missed = 0

    def center(self):
        return (self.box[0] + self.box[1]) / 2.0, (self.box[2] + self.box[3]) / 2.0

    def size(self):
        return self.box[1] - self.box[0], self.box[3] - self.box[2]

    def kind(self):
        longest = max(self.size())
        return 'wall' if longest > 1.5 else 'post' if longest < 0.7 else 'object'

    def describe(self):
        (e, n), (w, h) = self.center(), self.size()
        return f'#{self.id} {self.kind()} at E={e:.1f} N={n:.1f}, size {w:.1f} x {h:.1f} m'


class ObjectTracer:
    """Traces objects in the occupancy map.

    Each update groups occupied cells that are within link_cells of each other
    (so a wall with a missing cell stays one object). A group that shares cells
    with an object from the previous update keeps that object's ID (the oldest
    one if it touches several, i.e. objects merged; the biggest piece keeps the
    ID if one splits). An object is confirmed once it has been present in
    confirm_updates updates and has at least min_cells cells. One that drops out
    of the map is remembered (last known cells) for keep_missing updates, so a
    far object that flickers doesn't come back under a new ID.

    Defaults tuned on recorded flights in the lidar_test world.
    """
    def __init__(self, min_cells=5, link_cells=4, confirm_updates=5, keep_missing=5):
        self.min_cells = min_cells
        self.offsets = [(dr, dc) for dr in range(-link_cells, link_cells + 1)
                        for dc in range(-link_cells, link_cells + 1) if dr or dc]
        self.confirm_updates = confirm_updates
        self.keep_missing = keep_missing
        self.objects = {}
        self.next_id = 1

    def confirmed(self):
        return sorted((o for o in self.objects.values() if o.confirmed), key=lambda o: o.id)

    def _groups(self, rows, cols):
        """Lists of indices into rows/cols, one per group of linked cells."""
        index = {(r, c): i for i, (r, c) in enumerate(zip(rows, cols))}
        done = [False] * len(rows)
        groups = []
        for start in range(len(rows)):
            if done[start]:
                continue
            done[start] = True
            stack, members = [start], []
            while stack:
                i = stack.pop()
                members.append(i)
                for dr, dc in self.offsets:
                    j = index.get((rows[i] + dr, cols[i] + dc))
                    if j is not None and not done[j]:
                        done[j] = True
                        stack.append(j)
            groups.append(members)
        return groups

    def update(self, grid):
        """Re-trace objects from an OccupancyMap. Returns objects confirmed now."""
        rows, cols = np.nonzero(grid.logodds > grid.L_OCCUPIED)
        rows, cols = rows.tolist(), cols.tolist()
        current, newly = {}, []
        for members in sorted(self._groups(rows, cols), key=len, reverse=True):
            if len(members) < self.min_cells:
                continue
            r = np.array([rows[i] for i in members])
            c = np.array([cols[i] for i in members])
            cells = frozenset((r * grid.n + c).tolist())
            previous = sorted((o for o in self.objects.values()
                               if o.id not in current and o.cells & cells), key=lambda o: o.id)
            if previous:
                obj = previous[0]
            else:
                obj = TracedObject(self.next_id)
                self.next_id += 1
            points = np.stack([grid.origin + (c + 0.5) * grid.res,
                               grid.origin + (r + 0.5) * grid.res], axis=1)
            obj.set_cells(cells, points, grid.res)
            current[obj.id] = obj
            if not obj.confirmed and obj.updates >= self.confirm_updates:
                obj.confirmed = True
                newly.append(obj)
        # Keep recently-missing objects (not merged into another) for a few updates.
        for obj in self.objects.values():
            if obj.id not in current and obj.missed < self.keep_missing and not any(
                    obj.cells & o.cells for o in current.values()):
                obj.missed += 1
                current[obj.id] = obj
        self.objects = current
        return newly


# ======================================================================
# Node
# ======================================================================
class PatrolLidar(PatrolRtl):
    def __init__(self):
        super().__init__('patrol_lidar')

        p = self.declare_parameter
        scan_topic = p('scan_topic', '/scan').value
        att_topic = p('attitude_topic', 'fmu/out/vehicle_attitude').value
        self.speed = p('cruise_speed_m_s', 2.0).value          # approximate
        self.radius = p('avoid_radius_m', 0.8).value           # clearance each side
        self.lookahead = p('avoid_lookahead_m', 3.0).value     # how far ahead must be free
        self.stuck_timeout = p('stuck_timeout_s', 15.0).value
        self.min_obs_h = p('min_obstacle_height_m', 0.15).value   # lower hits = ground
        self.face_travel = p('face_travel_direction', True).value
        self.map_dir = p('map_dir', '.').value                 # '' = don't save
        self.map = OccupancyMap(p('map_size_m', 40.0).value, p('map_resolution_m', 0.1).value)
        self.tracker = ObjectTracer()

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        # Latched map so RViz gets the latest one when it subscribes late.
        map_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(LaserScan, scan_topic, self.on_scan, qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, att_topic, self.on_att, px4_qos)
        self.map_pub = self.create_publisher(OccupancyGrid, 'map', map_qos)
        self.obj_pub = self.create_publisher(MarkerArray, 'objects', 10)

        self.scan = None
        self.scan_time = None
        self.att = None
        self.obstacles = np.zeros((0, 2))   # live obstacle points near the drone, ENU
        self.cmd = None                     # avoidance setpoint (x, y, z) NED
        self.yaw_sp = None                  # yaw setpoint; None = hold home heading
        self.route = []                     # RETURN: corners then above-home, NED
        self.route_index = 0
        self.prog_goal = None               # progress tracking for stuck()
        self.prog_best = 0.0
        self.prog_ticks = 0
        self.avoid_side = 1                 # +1 = prefer passing on the left
        self.avoiding = False
        self.lidar_lost_ticks = 0
        self.lidar_ticks = 0
        self.finished = False

        self.get_logger().info(
            f'patrol_lidar: speed ~{self.speed} m/s, clearance {self.radius} m, '
            f'lookahead {self.lookahead} m, map {self.map.n * self.map.res:.0f} m '
            f'@ {self.map.res} m, listening on {scan_topic}')

    # ---------- subscriptions ----------
    def on_scan(self, msg):
        self.scan = msg
        self.scan_time = self.now_s()

    def on_att(self, msg):
        self.att = msg

    def scan_ok(self):
        return self.scan is not None and self.now_s() - self.scan_time < 1.0

    # ---------- frames ----------
    def enu(self, x, y):
        """PX4 local NED (x, y) -> ENU relative to home (e, n)."""
        return y - self.home[1], x - self.home[0]

    def ned(self, e, n):
        """ENU relative to home (e, n) -> PX4 local NED (x, y)."""
        return self.home[0] + n, self.home[1] + e

    # ---------- lidar processing: map, live obstacles, object tracing ----------
    def process_lidar(self):
        if not (self.scan_ok() and self.att is not None and self.home is not None
                and self.pos_ok()):
            return
        m = self.scan
        r = np.asarray(m.ranges, dtype=np.float64)
        a = m.angle_min + np.arange(r.size) * m.angle_increment
        valid = np.isfinite(r) & (r > m.range_min) & (r < m.range_max)

        # Beam directions: lidar frame is FLU (angle CCW from the nose); body is FRD.
        rot = quat_to_rot(self.att.q)
        d = np.stack([np.cos(a), -np.sin(a), np.zeros_like(a)], axis=1) @ rot.T
        origin = np.array([self.pos.x, self.pos.y, self.pos.z]) + rot @ LIDAR_OFFSET_FRD
        ends = origin + d * np.where(valid, r, FREE_RANGE_M)[:, None]

        # Hits below min_obstacle_height_m are the ground (drone tilted), not obstacles.
        height = self.home[2] - ends[:, 2]
        hit = valid & (height >= self.min_obs_h)

        e, n = self.enu(ends[:, 0], ends[:, 1])
        oe, on = self.enu(origin[0], origin[1])
        pe, pn = self.enu(self.pos.x, self.pos.y)
        dist = np.hypot(e - pe, n - pn)

        self.lidar_ticks += 1
        if self.lidar_ticks % 2 == 0:                    # map at 5 Hz
            self.map.update(oe, on, e, n, hit)

        near = hit & (dist < self.lookahead + self.radius + 1.0)
        self.obstacles = np.stack([e[near], n[near]], axis=1)

        if self.lidar_ticks % RATE_HZ == 0:              # map + object tracing at 1 Hz
            self.map_pub.publish(self.map.to_msg(self.map_header()))
            for obj in self.tracker.update(self.map):
                self.get_logger().info(f'Object found: {obj.describe()}')
        if self.lidar_ticks % 5 == 0:                    # markers at 2 Hz
            self.publish_objects()

    def map_header(self):
        msg = OccupancyGrid().header
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = 'map'
        return msg

    def publish_objects(self):
        header = self.map_header()
        arr = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)
        colors = {'wall': (0.9, 0.2, 0.2), 'post': (0.2, 0.4, 0.9), 'object': (0.2, 0.8, 0.3)}
        for obj in self.tracker.confirmed():
            ce, cn = obj.center()
            # The traced outline: one small cube per occupied cell of the object.
            box = Marker()
            box.header, box.ns, box.id = header, 'objects', obj.id
            box.type, box.action = Marker.CUBE_LIST, Marker.ADD
            box.pose.orientation.w = 1.0
            box.scale.x = box.scale.y = self.map.res
            box.scale.z = 0.3
            box.color.r, box.color.g, box.color.b = colors[obj.kind()]
            box.color.a = 0.9
            for pe_, pn_ in obj.points:
                pt = Point()
                pt.x, pt.y, pt.z = float(pe_), float(pn_), float(self.alt)
                box.points.append(pt)
            label = Marker()
            label.header, label.ns, label.id = header, 'labels', obj.id
            label.type, label.action = Marker.TEXT_VIEW_FACING, Marker.ADD
            label.pose.position.x, label.pose.position.y = ce, cn
            label.pose.position.z = float(self.alt) + 0.6
            label.pose.orientation.w = 1.0
            label.scale.z = 0.5
            label.color.r = label.color.g = label.color.b = label.color.a = 1.0
            label.text = f'#{obj.id} {obj.kind()}'
            arr.markers += [box, label]
        drone = Marker()
        drone.header, drone.ns, drone.id = header, 'drone', 0
        drone.type, drone.action = Marker.SPHERE, Marker.ADD
        drone.pose.position.x, drone.pose.position.y = self.enu(self.pos.x, self.pos.y)
        drone.pose.position.z = float(self.home[2] - self.pos.z)
        drone.pose.orientation.w = 1.0
        drone.scale.x = drone.scale.y = drone.scale.z = 0.5
        drone.color.g = drone.color.b = drone.color.a = 1.0
        arr.markers.append(drone)
        self.obj_pub.publish(arr)

    # ---------- obstacle avoidance ----------
    def obstacle_points(self, pe, pn):
        """Obstacles near the drone (live scan + map memory), relative to it, (M, 2)."""
        mem = self.map.occupied_near(pe, pn, self.lookahead + self.radius)
        pts = np.vstack([self.obstacles, mem]) if len(mem) else self.obstacles
        return pts - np.array([pe, pn])

    def free_heading(self, goal_heading, rel, check):
        """Heading (ENU rad) nearest the goal whose corridor is clear for `check` m.

        Candidates are tried 0, +5, -5, +10, -10 ... deg from the goal, starting on
        the side we last turned to, so the drone doesn't flip between sides.
        """
        offsets = [0.0]
        for k in range(1, 37):
            offsets += [self.avoid_side * 5.0 * k, -self.avoid_side * 5.0 * k]
        th = goal_heading + np.radians(offsets)
        u = np.stack([np.cos(th), np.sin(th)], axis=1)                 # (K, 2)
        along = rel @ u.T                                              # (M, K)
        cross = np.abs(rel[:, :1] * u[:, 1] - rel[:, 1:] * u[:, 0])    # (M, K)
        blocked = ((along > 0.0) & (along < check) & (cross < self.radius)).any(axis=0)
        free = np.flatnonzero(~blocked)
        if free.size == 0:
            return None, None
        off = offsets[free[0]]
        if off:
            self.avoid_side = 1 if off > 0 else -1
        return float(th[free[0]]), off

    def setpoint_toward(self, pe, pn, heading, step, z, face=True):
        e, n = pe + step * math.cos(heading), pn + step * math.sin(heading)
        if face and self.face_travel and step > 0.2:
            self.yaw_sp = math.atan2(math.cos(heading), math.sin(heading))   # NED yaw from ENU heading
        x, y = self.ned(e, n)
        return (x, y, z)

    def plan_to(self, goal):
        """Setpoint (x, y, z) NED that moves toward goal while steering around obstacles."""
        pe, pn = self.enu(self.pos.x, self.pos.y)
        ge, gn = self.enu(goal[0], goal[1])
        dist = math.hypot(ge - pe, gn - pn)
        z = goal[2]
        if dist < 0.05:
            return goal
        heading = math.atan2(gn - pn, ge - pe)
        step = min(dist, self.speed)     # PX4's position gain is ~1/s, so ~speed m/s
        rel = self.obstacle_points(pe, pn)
        if len(rel):
            ranges = np.hypot(rel[:, 0], rel[:, 1])
            k = int(np.argmin(ranges))
            if ranges[k] < 0.5 * self.radius:
                self.get_logger().warn(f'Obstacle {ranges[k]:.2f} m away, backing off',
                                       throttle_duration_sec=2.0)
                away = math.atan2(-rel[k, 1], -rel[k, 0])
                return self.setpoint_toward(pe, pn, away, 0.5, z, face=False)
            heading, off = self.free_heading(heading, rel, min(self.lookahead, dist))
            if heading is None:
                self.get_logger().warn('Boxed in by obstacles, holding position',
                                       throttle_duration_sec=2.0)
                return (self.pos.x, self.pos.y, z)
            if off and not self.avoiding:
                self.get_logger().info(
                    f'Obstacle ahead: steering {abs(off):.0f} deg {"left" if off > 0 else "right"}')
            elif not off and self.avoiding:
                self.get_logger().info('Path clear')
            self.avoiding = bool(off)
            if off:
                step = min(step, 0.5 * self.speed)      # slow down while going round
        return self.setpoint_toward(pe, pn, heading, step, z)

    def stuck(self, goal):
        """True once no progress (0.3 m closer) toward goal for stuck_timeout_s."""
        horiz, _ = self.dist_to(goal)
        if goal != self.prog_goal:
            self.prog_goal, self.prog_best, self.prog_ticks = goal, horiz, 0
            return False
        if horiz < self.prog_best - 0.3:
            self.prog_best, self.prog_ticks = horiz, 0
        else:
            self.prog_ticks += 1
        return self.prog_ticks >= self.stuck_timeout * RATE_HZ

    def label(self, wp):
        e, n = self.enu(wp[0], wp[1])
        return f'(E={e:.0f}, N={n:.0f})'

    # ---------- overrides of patrol_rtl ----------
    def set_state(self, new_state):
        super().set_state(new_state)
        self.cmd = None
        self.prog_goal = None

    def begin_land(self):
        # Hold where we are while PX4 switches to land (not the last waypoint).
        if self.pos is not None:
            self.target = (self.pos.x, self.pos.y, self.pos.z)
        super().begin_land()

    def publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.timestamp = self.now_us()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [NAN, NAN, NAN]
        msg.acceleration = [NAN, NAN, NAN]
        if self.yaw_sp is not None:
            msg.yaw = float(self.yaw_sp)
        else:
            msg.yaw = float(self.home[3]) if self.home else 0.0
        self.setpoint_pub.publish(msg)

    def on_timer(self):
        self.state_ticks += 1

        if self.state in FLYING_STATES:
            if not self.in_offboard():
                self.get_logger().warn(
                    'Left offboard mode (PX4 failsafe or pilot). Handing control back.')
                self.set_state('HANDOFF')
                return
            if not self.pos_ok():
                self.get_logger().error('Lost local position estimate. Landing in place.')
                self.begin_land()
                return

        self.process_lidar()

        # Without the lidar we can't avoid anything: hold, then land if it stays gone.
        if self.state in AVOID_STATES and not self.scan_ok():
            self.lidar_lost_ticks += 1
            if self.lidar_lost_ticks == 1:
                self.get_logger().error('Lost /scan. Holding position.')
                self.cmd = (self.pos.x, self.pos.y, self.target[2])
            if self.lidar_lost_ticks >= 5 * RATE_HZ:
                self.get_logger().error('No lidar for 5 s. Landing in place.')
                self.begin_land()
                return
        else:
            self.lidar_lost_ticks = 0
            getattr(self, 'step_' + self.state.lower())()

        streaming = (self.state in STREAMING_STATES or
                     (self.state == 'LAND' and not self.land_confirmed))
        if streaming and self.target is not None:
            sp = self.cmd if (self.state in AVOID_STATES and self.cmd is not None) else self.target
            self.publish_heartbeat()
            self.publish_setpoint(*sp)

    def step_wait_for_px4(self):
        if not self.scan_ok() or self.att is None:
            self.get_logger().info(
                'Waiting for /scan (is lidar_bridge running?) and vehicle attitude...',
                throttle_duration_sec=5.0)
            return
        super().step_wait_for_px4()

    def step_takeoff(self):
        if self.battery_low():
            self.start_return()
            return
        if self.reached(self.target):
            self.set_state('PATROL')

    def step_patrol(self):
        if self.battery_low():
            self.start_return()
            return
        goal = self.waypoints[self.wp_index]
        reached = self.reached(goal)
        if reached or self.stuck(goal):
            if not reached:
                self.get_logger().warn(
                    f'No progress toward corner {self.label(goal)} for '
                    f'{self.stuck_timeout:.0f} s. Skipping it.')
            self.wp_index = (self.wp_index + 1) % len(self.waypoints)
            if self.wp_index == 0:
                self.laps += 1
                self.get_logger().info(
                    f'Lap {self.laps} done (battery {self.batt_str()}, '
                    f'{len(self.tracker.confirmed())} objects so far)')
            goal = self.waypoints[self.wp_index]
        self.target = goal
        self.cmd = self.plan_to(goal)

    def corners_home(self):
        """Corners from here back to corner 0, whichever way round the square is shorter."""
        wps, i, k = self.waypoints, self.wp_index, len(self.waypoints)
        forward = [(i + s) % k for s in range((k - i) % k + 1)]        # i, i+1, ..., 0
        backward = list(range((i - 1) % k, -1, -1))                     # i-1, ..., 0

        def length(order):
            pts = [(self.pos.x, self.pos.y)] + [wps[j][:2] for j in order]
            return sum(math.dist(a, b) for a, b in zip(pts, pts[1:]))
        best = forward if length(forward) <= length(backward) else backward
        return [wps[j] for j in best]

    def start_return(self):
        corners = self.corners_home() if self.state == 'PATROL' else []
        self.set_state('RETURN')
        hx, hy, hz, _ = self.home
        self.route = corners + [(hx, hy, hz - self.alt)]
        self.route_index = 0
        path = ' -> '.join(self.label(c) for c in corners)
        self.get_logger().info(
            f'Returning along the square: {path + " -> " if path else ""}home')

    def step_return(self):
        goal = self.route[self.route_index]
        last = self.route_index == len(self.route) - 1
        if not last:
            reached = self.reached(goal)
            if reached or self.stuck(goal):
                if not reached:
                    self.get_logger().warn(
                        f'No progress toward corner {self.label(goal)}. Skipping it.')
                self.route_index += 1
                goal = self.route[self.route_index]
                last = self.route_index == len(self.route) - 1
                self.get_logger().info(
                    f'Return: next {"home" if last else "corner " + self.label(goal)}')
        self.target = goal

        if last and self.reached(goal):
            self.cmd = goal                       # hover exactly over home
            self.hold_ticks += 1
            if self.hold_ticks >= 2 * RATE_HZ:    # settle 2 s, then land
                self.begin_land()
            return
        self.hold_ticks = 0
        if last and self.stuck(goal):
            self.get_logger().warn('Cannot get to home. Holding; take over in QGC if needed.',
                                   throttle_duration_sec=5.0)
        self.cmd = self.plan_to(goal)

    def step_done(self):
        self.finish()
        super().step_done()

    # ---------- end of mission ----------
    def finish(self):
        """Log the objects found and save the map (once)."""
        if self.finished:
            return
        self.finished = True
        objs = self.tracker.confirmed()
        self.get_logger().info(f'{len(objs)} objects found:')
        for obj in objs:
            self.get_logger().info(f'  {obj.describe()}')
        if not self.map_dir:
            return
        folder = os.path.expanduser(self.map_dir)
        os.makedirs(folder, exist_ok=True)
        base = os.path.join(folder, time.strftime('lidar_map_%Y%m%d_%H%M%S'))
        self.map.save(base)
        with open(base + '_objects.csv', 'w') as f:
            f.write('id,kind,center_e_m,center_n_m,size_e_m,size_n_m,map_cells\n')
            for obj in objs:
                (ce, cn), (w, h) = obj.center(), obj.size()
                f.write(f'{obj.id},{obj.kind()},{ce:.2f},{cn:.2f},{w:.2f},{h:.2f},'
                        f'{len(obj.cells)}\n')
        self.get_logger().info(f'Map saved: {base}.pgm/.yaml, objects: {base}_objects.csv')


def main(args=None):
    rclpy.init(args=args)
    node = PatrolLidar()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.finish()      # save the map even if stopped with Ctrl-C
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
