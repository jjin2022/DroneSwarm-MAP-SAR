#!/usr/bin/env python3
"""Put the landing pad at a random clear spot in the forest.

Reads the trunks from a base world (default forest_pillars.sdf), picks a random
pad position inside the forest with enough room to land, and writes
forest_pillars_random_pad.sdf (same trunks, pad moved). Run it again for a new
spot; the file name stays the same, so PX4 only needs linking once:

    ln -s ~/dev/DroneSwarm-MAP-SAR/src/las_offboard/worlds/forest_pillars_random_pad.sdf \\
          ~/dev/PX4-Autopilot/Tools/simulation/gz/worlds/forest_pillars_random_pad.sdf

It prints the patrol_lidar arguments that tell the drone roughly where the dock
is (dock_e, dock_n). --hint-error adds a random error to that, like a robot that
only knows its own position to within a metre; the camera then has to find the
pad. Keep it under ~1.3 m: from 2 m up the camera sees about +-1.4 m around it.

    python3 random_pad.py                   # new random spot, exact dock position
    python3 random_pad.py --seed 4 --hint-error 0.8

Rules for the spot (metres): inside the forest square (--size), at least
--clearance from every trunk surface (the drone reaches 0.38 m from its centre
and descends without avoidance), and at least --min-dist from the spawn point.
"""
import argparse
import math
import os
import random
import re

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_NAME = 'forest_pillars_random_pad'


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--base', default=os.path.join(HERE, 'forest_pillars.sdf'),
                    help='world to copy the trunks from')
    ap.add_argument('--seed', type=int, help='random seed (default: a new one each run)')
    ap.add_argument('--size', type=float, default=15.0, help='forest square side (m)')
    ap.add_argument('--clearance', type=float, default=1.0,
                    help='min distance from the pad centre to any trunk surface (m)')
    ap.add_argument('--min-dist', type=float, default=2.0,
                    help='min distance from the spawn point (m), so the dock really moved')
    ap.add_argument('--hint-error', type=float, default=0.0,
                    help='random error (m) added to the dock position given to the drone')
    a = ap.parse_args()

    seed = a.seed if a.seed is not None else random.randrange(10 ** 6)
    rng = random.Random(seed)
    base = open(a.base).read()
    trunks = [(float(x), float(y), float(r)) for x, y, r in re.findall(
        r'<model name="trunk_\d+">.*?<pose>(\S+) (\S+) .*?<radius>(\S+)</radius>', base, re.S)]
    if not trunks:
        raise SystemExit(f'No trunks found in {a.base}')

    half = a.size / 2.0 - 0.5            # keep the whole pad inside the forest square
    for _ in range(100000):
        e, n = rng.uniform(-half, half), rng.uniform(-half, half)
        if math.hypot(e, n) < a.min_dist:
            continue
        if min(math.hypot(e - x, n - y) - r for x, y, r in trunks) >= a.clearance:
            break
    else:
        raise SystemExit('No spot found; lower --clearance or --min-dist.')
    room = min(math.hypot(e - x, n - y) - r for x, y, r in trunks)

    pad = (f'    <include>\n      <uri>model://arucotag</uri>\n      <name>landing_pad</name>\n'
           f'      <pose>{e:.2f} {n:.2f} 0.001 0 0 0</pose>\n    </include>')
    world = re.sub(r'<world name="[^"]+">', f'<world name="{OUT_NAME}">', base, count=1)
    if '<name>landing_pad</name>' in world:
        world = re.sub(r'    <include>\s*<uri>model://arucotag</uri>\s*<name>landing_pad</name>'
                       r'.*?</include>', pad, world, count=1, flags=re.S)
    else:
        world = world.replace('  </world>', pad + '\n  </world>', 1)
    world = re.sub(r'  Landing pad: .*?\n',
                   f'  Landing pad: RANDOM (random_pad.py, seed {seed}) at E={e:.2f} N={n:.2f}.\n',
                   world, count=1)
    out = os.path.join(HERE, OUT_NAME + '.sdf')
    with open(out, 'w') as f:
        f.write(world)

    ang = rng.uniform(0.0, 2 * math.pi)
    he, hn = e + a.hint_error * math.cos(ang), n + a.hint_error * math.sin(ang)
    print(f'Wrote {out}')
    print(f'Pad at E={e:.2f} N={n:.2f} ({math.hypot(e, n):.1f} m from spawn, '
          f'{room:.2f} m to the nearest trunk), seed {seed}')
    if a.hint_error:
        print(f'Dock position given to the drone is off by {a.hint_error:.2f} m: '
              f'E={he:.2f} N={hn:.2f}')
    print('\nRun the precision-landing setup (docs/running_sim.md) with '
          f'PX4_GZ_WORLD={OUT_NAME} and world:={OUT_NAME}, and patrol_lidar with:')
    print(f'    -p dock_e:={he:.2f} -p dock_n:={hn:.2f}')


if __name__ == '__main__':
    main()
