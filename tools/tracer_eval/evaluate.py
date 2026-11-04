#!/usr/bin/env python3
"""Score patrol_lidar's ObjectTracer on recorded flights, without flying.

Replays every dataset in data/ (made by make_dataset.py) through the tracer,
exactly as the node runs it once a second, and compares the final objects with
the world's ground truth:

  objects     confirmed objects at the end of the flight
  observable  real objects the final map shows (some are never in lidar view)
  found       observable objects with at least one traced object
  dup         extra traced objects on an already-found real object
  false       traced objects near no real object
  reports     "Object found" messages over the flight (ideally = objects)
  labels      found objects whose kind() is right (post / wall / tree / object)

    python3 evaluate.py                 # current code; exits 1 if a check fails
    python3 evaluate.py --rev HEAD~3    # the tracer as of an older commit

Needs ROS 2 and px4_msgs sourced (patrol_lidar imports them).
"""
import argparse
import glob
import importlib.util
import os
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, '..', '..'))
NODE = 'src/las_offboard/las_offboard/patrol_lidar.py'

# Regression checks: worlds the tracer must get exactly right.
# PX4's leafy `forest` is reported but not checked (canopies still split).
CHECKS = {
    'forest_pillars': dict(missed=0, dup=0, false=0),
    'lidar_test': dict(missed=0, dup=0, false=0),
}


def load_tracer(rev):
    """The patrol_lidar module from the working tree, or from git revision rev."""
    sys.path.insert(0, os.path.join(REPO, 'src', 'las_offboard'))
    path = os.path.join(REPO, NODE)
    if rev:
        src = subprocess.run(['git', '-C', REPO, 'show', f'{rev}:{NODE}'],
                             check=True, capture_output=True, text=True).stdout
        tmp = tempfile.NamedTemporaryFile('w', suffix='.py', delete=False)
        tmp.write(src)
        tmp.close()
        path = tmp.name
    spec = importlib.util.spec_from_file_location('patrol_lidar_under_test', path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Grid:
    """The parts of OccupancyMap the tracer reads, rebuilt from a dataset."""
    def __init__(self, d, l_occupied):
        self.res, self.n, self.origin = float(d['res']), int(d['n']), float(d['origin'])
        self.L_OCCUPIED = l_occupied
        self.logodds = np.zeros((self.n, self.n), dtype=np.float32)

    def set_occupied(self, flat):
        self.logodds.fill(0.0)
        self.logodds.reshape(-1)[flat] = self.L_OCCUPIED + 1.0


def evaluate(path, mod):
    d = np.load(path)
    grid = Grid(d, mod.OccupancyMap.L_OCCUPIED)
    tracer = mod.ObjectTracer()
    off, cells = d['offsets'], d['cells']
    reports = 0
    for i in range(len(off) - 1):
        grid.set_occupied(cells[off[i]:off[i + 1]])
        reports += len(tracer.update(grid))

    truth, kinds, radius = d['truth_xy'], d['truth_kinds'], float(d['radius'])
    objs = tracer.confirmed()
    per = {i: [] for i in range(len(truth))}
    false = 0
    for o in objs:
        dist = [float(np.min(np.hypot(o.points[:, 0] - x, o.points[:, 1] - y)))
                for x, y in truth]
        i = int(np.argmin(dist))
        if dist[i] <= radius:
            per[i].append(o)
        else:
            false += 1
    final = cells[off[-2]:off[-1]]
    fe = grid.origin + (final % grid.n + 0.5) * grid.res
    fn = grid.origin + (final // grid.n + 0.5) * grid.res
    observable = [i for i, (x, y) in enumerate(truth)
                  if final.size and np.min(np.hypot(fe - x, fn - y)) <= radius]
    found = [i for i in observable if per[i]]
    matched = [(o, kinds[i]) for i, v in per.items() for o in v]
    right = sum(1 for o, k in matched if o.kind() in str(k).split('|'))
    return dict(objects=len(objs), observable=len(observable), found=len(found),
                missed=len(observable) - len(found),
                dup=sum(len(v) - 1 for v in per.values() if len(v) > 1),
                false=false, reports=reports,
                labels=f'{right}/{len(matched)}' if matched else '-')


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--rev', help='git revision to evaluate instead of the working tree')
    a = ap.parse_args()
    mod = load_tracer(a.rev)

    cols = ['objects', 'observable', 'found', 'missed', 'dup', 'false', 'reports', 'labels']
    print(f'tracer: {a.rev or "working tree"}')
    print(f'{"dataset":16s}' + ''.join(f'{c:>11s}' for c in cols))
    failed = []
    for path in sorted(glob.glob(os.path.join(HERE, 'data', '*.npz'))):
        name = os.path.splitext(os.path.basename(path))[0]
        s = evaluate(path, mod)
        print(f'{name:16s}' + ''.join(f'{str(s[c]):>11s}' for c in cols))
        for key, limit in CHECKS.get(name, {}).items():
            if s[key] > limit:
                failed.append(f'{name}: {key} = {s[key]} (allowed {limit})')
    if failed:
        print('\nFAILED:\n  ' + '\n  '.join(failed))
        sys.exit(1)
    print('\nAll checks passed.')


if __name__ == '__main__':
    main()
