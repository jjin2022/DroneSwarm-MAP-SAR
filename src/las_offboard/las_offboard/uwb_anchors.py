"""UWB anchor layout on the robot, shared by uwb_sim (simulator) and uwb_locator (drone).

Robot body frame: origin on the ground under the body centre, x forward, y left,
z up, metres. Must match the anchor markers in models/robot_dock/model.sdf.
Two heights so the drone's height relative to the robot is observable.
"""
import numpy as np

ANCHORS_BODY = np.array([
    [0.52, 0.20, 0.70],     # front-left, high
    [0.52, -0.20, 0.40],    # front-right, low
    [-0.52, 0.20, 0.40],    # rear-left, low
    [-0.52, -0.20, 0.70],   # rear-right, high
])

# Height of the landing platform's top above the robot's ground point (m).
PLATFORM_TOP_M = 0.665


def anchors_world(e, n, yaw, ground_u=0.0):
    """Anchor positions (4, 3) in ENU for a robot at (e, n), heading yaw (rad, ENU)."""
    c, s = np.cos(yaw), np.sin(yaw)
    rot = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    return ANCHORS_BODY @ rot.T + np.array([e, n, ground_u])
