#!/usr/bin/env python3
"""Generate forest_pillars.sdf: a semi-dense "forest" of plain cylinder trunks.

Trunks are simple cylinders (no meshes, no downloads) so the 2D lidar gets clean
returns. That keeps avoidance and mapping tests separate from the noise of real
tree models with branches and foliage (PX4's own forest.sdf).

Layout rules (all distances in metres, ENU, home/spawn at (0, 0)):
  - trunks fill a square of side --size centred on home
  - at least --min-gap of free space between neighbouring trunk surfaces, so the
    drone (patrol_lidar keeps avoid_radius_m = 0.8 m on each side) fits between
  - nothing within --spawn-clear of home, so takeoff and landing are clear
  - nothing within --corner-clear of the corners of the --patrol square, so
    patrol_lidar can reach every waypoint
  - trunks are 6-10 m tall, so the lidar sees them at any patrol altitude

The physics, lighting, ground and GPS origin are copied from lidar_test.sdf
(everything above its "LIDAR TEST OBSTACLES" marker), so both worlds match.

Usage (from this folder):
  python3 gen_forest.py                      # writes forest_pillars.sdf, seed 1
  python3 gen_forest.py --seed 7 --trees 40  # a different / denser layout
Same seed + options -> same forest, so a layout can be re-created exactly.
"""
import argparse
import math
import os
import random

HERE = os.path.dirname(os.path.abspath(__file__))
BASE_WORLD = os.path.join(HERE, 'lidar_test.sdf')
MARKER = '<!-- ================= LIDAR TEST OBSTACLES ================= -->'


def place_trunks(a, rng):
    """Random trunk positions obeying the layout rules (rejection sampling)."""
    half = a.size / 2.0
    p = a.patrol / 2.0
    corners = [(p, p), (p, -p), (-p, p), (-p, -p)]
    trunks = []  # (x, y, radius, height)
    for _ in range(20000):
        if len(trunks) >= a.trees:
            break
        r = rng.uniform(a.r_min, a.r_max)
        # Keep the whole trunk inside the square.
        x = rng.uniform(-half + r, half - r)
        y = rng.uniform(-half + r, half - r)
        if math.hypot(x, y) < a.spawn_clear + r:
            continue
        if any(math.hypot(x - cx, y - cy) < a.corner_clear + r for cx, cy in corners):
            continue
        if any(math.hypot(x - tx, y - ty) < r + tr + a.min_gap for tx, ty, tr, _ in trunks):
            continue
        trunks.append((x, y, r, rng.uniform(6.0, 10.0)))
    return trunks


# Landing pad: PX4's ArUco marker model (DICT_4X4_50, ID 0, 0.5 x 0.5 m) at the
# spawn point, so the drone takes off from it and can land back on it.
PAD_NOTE = "PX4's arucotag model (ArUco 4x4 ID 0, 0.5 m)"
PAD_SDF = '''
    <include>
      <uri>model://arucotag</uri>
      <name>landing_pad</name>
      <pose>{e:g} {n:g} 0.001 0 0 0</pose>
    </include>'''


# The pad on the back of the robot stand-in (models/robot_dock, linked into PX4's
# models folder). Its platform top is 0.45 m up.
ROBOT_SDF = '''
    <include>
      <uri>model://robot_dock</uri>
      <name>robot_dock</name>
      <pose>{e:g} {n:g} 0 0 0 0</pose>
    </include>'''


def trunk_sdf(i, x, y, r, h):
    # Bark-brown, slightly varied so neighbouring trunks are easy to tell apart.
    shade = 0.30 + 0.10 * ((i * 37) % 7) / 6.0
    rgb = f'{shade:.2f} {shade * 0.65:.2f} {shade * 0.35:.2f} 1'
    geom = f'<cylinder><radius>{r:.2f}</radius><length>{h:.1f}</length></cylinder>'
    return f'''
    <model name="trunk_{i:02d}">
      <static>true</static>
      <pose>{x:.2f} {y:.2f} {h / 2:.2f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry>{geom}</geometry></collision>
        <visual name="visual">
          <geometry>{geom}</geometry>
          <material><ambient>{rgb}</ambient><diffuse>{rgb}</diffuse></material>
        </visual>
      </link>
    </model>'''


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--trees', type=int, default=30, help='trunks to place (fewer if they do not fit)')
    ap.add_argument('--size', type=float, default=15.0, help='side of the forest square')
    ap.add_argument('--min-gap', type=float, default=2.0, help='free space between trunk surfaces')
    ap.add_argument('--r-min', type=float, default=0.12)
    ap.add_argument('--r-max', type=float, default=0.35)
    ap.add_argument('--spawn-clear', type=float, default=2.0, help='clear radius around home')
    ap.add_argument('--patrol', type=float, default=10.0, help='patrol_size_m whose corners are kept clear')
    ap.add_argument('--corner-clear', type=float, default=1.5)
    ap.add_argument('--name', default='forest_pillars', help='world name (= file name and PX4_GZ_WORLD)')
    ap.add_argument('--no-pad', dest='pad', action='store_false',
                    help="leave out the landing pad (PX4's 0.5 m ArUco marker, ID 0)")
    ap.add_argument('--robot', action='store_true',
                    help='put the pad on the robot_dock model (0.45 m up) instead of the '
                         'ground; spawn the drone on it with PX4_GZ_MODEL_POSE="E,N,0.46"')
    ap.add_argument('--pad-pos', type=float, nargs=2, default=(0.0, 0.0), metavar=('E', 'N'),
                    help='pad position (m); default the spawn point. Off the spawn point it '
                         'tests landing on a dock that moved. Keep it inside --spawn-clear.')
    a = ap.parse_args()

    base = open(BASE_WORLD).read()
    if MARKER not in base:
        raise SystemExit(f'Marker not found in {BASE_WORLD}; cannot copy its base world.')
    body = base[base.index('<sdf version'):base.index(MARKER)]
    body = body.replace('<world name="lidar_test">', f'<world name="{a.name}">', 1)

    trunks = place_trunks(a, random.Random(a.seed))
    if len(trunks) < a.trees:
        print(f'Note: only {len(trunks)} of {a.trees} trunks fit with these spacing rules.')

    header = f'''<?xml version="1.0" encoding="UTF-8"?>
<!--
  {a.name}.sdf - GENERATED by gen_forest.py, do not edit by hand.
  Generated with: seed={a.seed} trees={a.trees} size={a.size:g} min-gap={a.min_gap:g}
  (pass the same values as gen_forest.py options to regenerate; XML comments can't contain two dashes)

  {len(trunks)} cylinder trunks in a {a.size:g} x {a.size:g} m square centred on the spawn point,
  >= {a.min_gap:g} m apart (surface to surface), {a.spawn_clear:g} m clear around spawn,
  corners of a {a.patrol:g} m patrol square kept clear. Trunks are 6-10 m tall.
  {(f'Landing pad: on the robot_dock model (pad 0.45 m up) at E={a.pad_pos[0]:g} N={a.pad_pos[1]:g}; spawn the drone with PX4_GZ_MODEL_POSE="{a.pad_pos[0]:g},{a.pad_pos[1]:g},0.46".' if a.robot else f'Landing pad: {PAD_NOTE} at E={a.pad_pos[0]:g} N={a.pad_pos[1]:g}.') if a.pad else 'No landing pad.'}

  Link into PX4 once:
    ln -s ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/{a.name}.sdf \\
          ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/{a.name}.sdf
  Coordinates: ENU, x = East, y = North, metres; spawn at (0, 0).
-->
'''
    pad_sdf = (ROBOT_SDF if a.robot else PAD_SDF).format(e=a.pad_pos[0], n=a.pad_pos[1])
    models = (pad_sdf if a.pad else '') + ''.join(trunk_sdf(i + 1, *t) for i, t in enumerate(trunks))
    out = header + body + f'<!-- ================= FOREST TRUNKS ({len(trunks)}) ================= -->' \
        + models + '\n  </world>\n</sdf>\n'
    path = os.path.join(HERE, f'{a.name}.sdf')
    with open(path, 'w') as f:
        f.write(out)

    area = a.size * a.size
    print(f'Wrote {path}: {len(trunks)} trunks, {len(trunks) / area * 100:.1f} per 100 m^2')
    gaps = []
    for i, (x, y, r, _) in enumerate(trunks):
        gaps.append(min((math.hypot(x - tx, y - ty) - r - tr for j, (tx, ty, tr, _) in enumerate(trunks) if j != i),
                        default=float('inf')))
    if gaps:
        print(f'Closest trunk-to-trunk gap: {min(gaps):.2f} m, mean nearest gap: {sum(gaps) / len(gaps):.2f} m')


if __name__ == '__main__':
    main()
