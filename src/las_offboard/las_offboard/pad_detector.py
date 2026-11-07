"""Find the landing pad (an ArUco marker) in the downward camera and publish where it is.

Needs the drone model with a downward camera (x500_lidar_cam_down) and the camera
bridge (lidar_bridge.launch.py ... camera:=true).

  In:   /camera/image_raw, /camera/camera_info   downward camera
        fmu/out/vehicle_attitude                 PX4 attitude (to level the view)
        fmu/out/vehicle_local_position_v1        PX4 position (to place the pad)
  Out:  /landing_pad   geometry_msgs/PointStamped, frame "px4_local_ned": the pad
                       centre in PX4's local NED frame (x north, y east, z down, m),
                       the frame patrol_lidar's setpoints use. Published only on
                       frames where the marker is seen; the stamp is the image's.
        /camera/pad_debug  sensor_msgs/Image: the camera image with the marker
                       outlined and its offset written on it (for RViz).

How: OpenCV finds the marker's four corners; knowing its real size
(marker_size_m), solvePnP gives its position relative to the camera. The camera
looks straight down with image-up = drone-forward, so camera (x right, y down,
z out of the lens) maps to the drone's body FRD frame as forward = -y,
right = x, down = z. The attitude rotates that into NED and the drone's position
is added. Tilt is handled by the rotation, so the estimate stays put while the
drone leans to move.

Default marker: PX4's arucotag model, DICT_4X4_50 ID 0, 0.5 m.

OpenCV note: Ubuntu's OpenCV is built against NumPy 1.x. If a NumPy 2 in
~/.local shadows the system one, importing cv2 fails; this node then restarts
itself once with user site-packages disabled (PYTHONNOUSERSITE=1).
"""
import os
import sys

import numpy as np

# A user-installed NumPy 2 (~/.local) breaks Ubuntu's NumPy-1 OpenCV: restart
# once without user site-packages before cv2 is imported (avoids NumPy's noisy
# "compiled using NumPy 1.x" warning from a failed import).
if (int(np.__version__.split('.')[0]) >= 2 and '/.local/' in np.__file__
        and os.environ.get('PYTHONNOUSERSITE') != '1'):
    os.environ['PYTHONNOUSERSITE'] = '1'
    os.execv(sys.executable, [sys.executable] + sys.argv)

import cv2  # noqa: E402
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition

# Camera position on the drone, body FRD (m): 3 cm under the body centre.
CAMERA_OFFSET_FRD = np.array([0.0, 0.0, 0.03])
# Camera optical frame (x right, y down, z out of lens) -> body FRD.
OPTICAL_TO_FRD = np.array([[0.0, -1.0, 0.0],    # forward = -y
                           [1.0, 0.0, 0.0],     # right   =  x
                           [0.0, 0.0, 1.0]])    # down    =  z


def quat_to_rot(q):
    """Rotation matrix from a PX4 quaternion (w, x, y, z), FRD body -> NED."""
    w, x, y, z = (float(v) for v in q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


class PadDetector(Node):
    def __init__(self):
        super().__init__('pad_detector')
        p = self.declare_parameter
        self.marker_id = p('marker_id', 0).value
        self.size = p('marker_size_m', 0.5).value
        self.debug = p('publish_debug_image', True).value
        dict_name = p('aruco_dictionary', 'DICT_4X4_50').value
        self.dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, dict_name))
        self.params = cv2.aruco.DetectorParameters_create()
        self.bridge = CvBridge()
        self.K = self.dist = None
        self.att = self.pos = None
        self.seen = self.frames = 0

        px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CameraInfo, 'camera/camera_info', self.on_info, 10)
        self.create_subscription(Image, 'camera/image_raw', self.on_image,
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleAttitude, 'fmu/out/vehicle_attitude',
                                 lambda m: setattr(self, 'att', m), px4_qos)
        self.create_subscription(VehicleLocalPosition, 'fmu/out/vehicle_local_position_v1',
                                 lambda m: setattr(self, 'pos', m), px4_qos)
        self.pad_pub = self.create_publisher(PointStamped, 'landing_pad', 10)
        self.debug_pub = self.create_publisher(Image, 'camera/pad_debug', 1)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'Looking for ArUco {dict_name} ID {self.marker_id}, '
                               f'{self.size} m, on /camera/image_raw')

    def on_info(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        self.dist = np.array(msg.d, dtype=np.float64) if len(msg.d) else np.zeros(5)

    def report(self):
        self.get_logger().info(f'pad seen in {self.seen}/{self.frames} frames (last 5 s)')
        self.seen = self.frames = 0

    def on_image(self, msg):
        if self.K is None:
            return
        self.frames += 1
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        found = None
        if ids is not None:
            for c, i in zip(corners, ids.ravel()):
                if i == self.marker_id:
                    found = c
                    break
        offset_frd = None
        if found is not None:
            _, tvec, _ = cv2.aruco.estimatePoseSingleMarkers([found], self.size, self.K, self.dist)
            offset_frd = OPTICAL_TO_FRD @ tvec.reshape(3) + CAMERA_OFFSET_FRD
            self.seen += 1
            if self.att is not None and self.pos is not None:
                ned = (np.array([self.pos.x, self.pos.y, self.pos.z])
                       + quat_to_rot(self.att.q) @ offset_frd)
                out = PointStamped()
                out.header.stamp = msg.header.stamp
                out.header.frame_id = 'px4_local_ned'
                out.point.x, out.point.y, out.point.z = (float(v) for v in ned)
                self.pad_pub.publish(out)
        if self.debug and self.debug_pub.get_subscription_count():
            if found is not None:
                cv2.aruco.drawDetectedMarkers(img, [found], np.array([[self.marker_id]]))
                f, r, d = offset_frd
                cv2.putText(img, f'pad: fwd {f:+.2f} right {r:+.2f} down {d:.2f} m', (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                cv2.putText(img, 'no pad', (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            dbg = self.bridge.cv2_to_imgmsg(img, 'bgr8')
            dbg.header = msg.header
            self.debug_pub.publish(dbg)


def main(args=None):
    rclpy.init(args=args)
    node = PadDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
