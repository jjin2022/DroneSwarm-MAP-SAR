#!/usr/bin/env python3
"""GPS-loss sanity check for the swarm relative-localization design.

A walking quadruped carries 4 UWB anchors. N drones fly sector patrols around
it. Each drone has drifting odometry (stands in for LIO/VIO: a slowly
wandering velocity bias). GPS is available for the first `--gps-off` seconds,
then lost. Compared estimators, all in the robot's map frame:

  dead_reckon   odometry only after GPS loss
  uwb_robot     EKF: odometry + UWB ranges drone<->robot anchors
  uwb_full      EKF: odometry + robot-anchor ranges + drone<->drone ranges

Ranges have Gaussian noise, random NLOS outliers (positive bias, as in a
forest), dropouts beyond `--uwb-range`, and are chi-square gated. The EKF is
centralized (it would run on the robot); state per drone is position and
odometry velocity bias. This is a 3D point-mass model with no attitude: it
tests observability and geometry, not flight dynamics.

    python3 tools/swarm_uwb_sim.py
    python3 tools/swarm_uwb_sim.py --drones 3 --seed 7 --nlos 0.2
"""
import argparse
import numpy as np


def anchors_body():
    # Anchors on short masts at the robot's corners (m, body frame).
    # Height differences matter: coplanar anchors give a z ambiguity.
    return np.array([[0.40, 0.25, 0.30], [0.40, -0.25, 0.55],
                     [-0.40, 0.25, 0.55], [-0.40, -0.25, 0.30]])


def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def robot_pose(t):
    # Walks a gentle S-curve at ~0.5 m/s. Its own LIO pose defines the frame.
    x = 0.5 * t
    y = 10.0 * np.sin(t / 60.0)
    yaw = np.arctan2(10.0 / 60.0 * np.cos(t / 60.0), 0.5)
    return np.array([x, y, 0.0]), yaw


def drone_pos(i, n, t, robot_xy):
    # Each drone orbits a point in its own sector, 20-60 m from the robot,
    # with the radius breathing in and out (sweep pattern).
    sector = 2 * np.pi * i / n
    rad = 40.0 + 20.0 * np.sin(t / 25.0 + i)
    ang = sector + 0.6 * np.sin(t / 40.0 + 2 * i)
    alt = 8.0 + 3.0 * np.sin(t / 30.0 + i)
    c = robot_xy + np.array([rad * np.cos(ang), rad * np.sin(ang), 0.0])
    return np.array([c[0], c[1], alt])


def run(args, mode, rng_seed):
    rng = np.random.default_rng(rng_seed)
    n, dt = args.drones, 0.1
    steps = int(args.duration / dt)
    A = anchors_body()
    ns = 6 * n
    x = np.zeros(ns)
    P = np.eye(ns) * 1e-2
    bias = np.zeros((n, 3))  # true odometry velocity bias, random walk
    # Initial: drones start docked, so position is known in the robot frame.
    for i in range(n):
        x[6 * i:6 * i + 3] = drone_pos(i, n, 0.0, robot_pose(0.0)[0])
    q_pos, q_bias = (0.05 ** 2) * dt, (args.bias_walk ** 2) * dt
    errs, rel_errs = [], []
    prev = [drone_pos(i, n, 0.0, robot_pose(0.0)[0]) for i in range(n)]
    for k in range(1, steps + 1):
        t = k * dt
        rp, ryaw = robot_pose(t)
        truth = [drone_pos(i, n, t, rp) for i in range(n)]
        # --- propagate with odometry
        F = np.eye(ns)
        Q = np.zeros((ns, ns))
        for i in range(n):
            bias[i] += rng.normal(0, args.bias_walk * np.sqrt(dt), 3)
            bias[i, 2] *= 0.98  # baro/rangefinder keep z bias small
            v_true = (truth[i] - prev[i]) / dt
            v_meas = v_true + bias[i] + rng.normal(0, 0.05, 3)
            s = 6 * i
            x[s:s + 3] += (v_meas - x[s + 3:s + 6]) * dt
            F[s:s + 3, s + 3:s + 6] = -np.eye(3) * dt
            Q[s:s + 3, s:s + 3] = np.eye(3) * q_pos
            Q[s + 3:s + 6, s + 3:s + 6] = np.eye(3) * q_bias
        prev = truth
        P = F @ P @ F.T + Q
        gps = t < args.gps_off
        # --- measurements
        rows, zs, hs, rs = [], [], [], []
        if gps and k % 10 == 0:  # 1 Hz GNSS, 1.5 m sigma
            for i in range(n):
                for d in range(3):
                    H = np.zeros(ns); H[6 * i + d] = 1
                    rows.append(H); zs.append(truth[i][d] + rng.normal(0, 1.5))
                    hs.append(x[6 * i + d]); rs.append(1.5 ** 2)
        if mode != "dead_reckon" and k % 2 == 0:  # 5 Hz per link
            anchors_w = [rp + rot_z(ryaw) @ a for a in A]
            pairs = [(i, ("a", aw)) for i in range(n) for aw in anchors_w]
            if mode == "uwb_full":
                pairs += [(i, ("d", j)) for i in range(n) for j in range(i + 1, n)]
            for i, (kind, other) in pairs:
                pi = truth[i]
                po = other if kind == "a" else truth[other]
                d_true = np.linalg.norm(pi - po)
                if d_true > args.uwb_range:
                    continue
                z = d_true + rng.normal(0, 0.10)
                if rng.random() < args.nlos:
                    z += rng.uniform(0.3, 2.5)
                ei = x[6 * i:6 * i + 3]
                eo = other if kind == "a" else x[6 * other:6 * other + 3]
                diff = ei - eo
                dh = np.linalg.norm(diff) + 1e-9
                H = np.zeros(ns)
                H[6 * i:6 * i + 3] = diff / dh
                if kind == "d":
                    H[6 * other:6 * other + 3] = -diff / dh
                R = 0.15 ** 2
                S = H @ P @ H + R
                if (z - dh) ** 2 / S > 9.0:  # 3-sigma gate rejects NLOS spikes
                    continue
                rows.append(H); zs.append(z); hs.append(dh); rs.append(R)
        if rows:
            # Sequential scalar updates (cheap, and linearize per update).
            for H, z, h, R in zip(rows, zs, hs, rs):
                S = H @ P @ H + R
                K = P @ H / S
                x += K * (z - h)
                P = (np.eye(ns) - np.outer(K, H)) @ P
        if not gps:
            est = [x[6 * i:6 * i + 3] for i in range(n)]
            errs.append(np.mean([np.linalg.norm(est[i] - truth[i]) for i in range(n)]))
            rel = []
            for i in range(n):
                for j in range(i + 1, n):
                    rel.append(np.linalg.norm((est[i] - est[j]) - (truth[i] - truth[j])))
            rel_errs.append(np.mean(rel) if rel else 0.0)
    errs, rel_errs = np.array(errs), np.array(rel_errs)
    return dict(mean=errs.mean(), final=errs[-1], p95=np.percentile(errs, 95),
                rel_mean=rel_errs.mean(), rel_final=rel_errs[-1])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--drones", type=int, default=4)
    ap.add_argument("--duration", type=float, default=900.0, help="s (15 min sortie)")
    ap.add_argument("--gps-off", type=float, default=60.0, help="s when GPS is lost")
    ap.add_argument("--uwb-range", type=float, default=80.0, help="m, forest LOS-ish")
    ap.add_argument("--nlos", type=float, default=0.10, help="outlier probability")
    ap.add_argument("--bias-walk", type=float, default=0.0015,
                    help="odometry velocity-bias random walk, m/s/sqrt(s)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--trials", type=int, default=3)
    a = ap.parse_args()
    print(f"{a.drones} drones, {a.duration:.0f} s, GPS lost at {a.gps_off:.0f} s, "
          f"UWB range {a.uwb_range:.0f} m, NLOS {a.nlos:.0%}, {a.trials} trials\n")
    print(f"{'mode':12s} {'mean err':>9s} {'p95 err':>8s} {'final':>7s} "
          f"{'rel mean':>9s} {'rel final':>9s}   (m, robot frame, after GPS loss)")
    for mode in ("dead_reckon", "uwb_robot", "uwb_full"):
        rs = [run(a, mode, a.seed + t) for t in range(a.trials)]
        m = {k: np.mean([r[k] for r in rs]) for k in rs[0]}
        print(f"{mode:12s} {m['mean']:9.2f} {m['p95']:8.2f} {m['final']:7.2f} "
              f"{m['rel_mean']:9.2f} {m['rel_final']:9.2f}")


if __name__ == "__main__":
    main()
