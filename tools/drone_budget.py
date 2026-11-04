#!/usr/bin/env python3
"""Mass, power and hover-endurance budget for the LAS scout drone.

Every number here is an estimate to be replaced with weighed parts and
bench-measured power. The hover model is momentum theory with a figure of
merit and a drivetrain efficiency, which is good to roughly +/-15% for
FPV-class props; treat the endurance as a sizing check, not a spec.

    python3 tools/drone_budget.py            # all variants
    python3 tools/drone_budget.py --variant baseline
"""
import argparse
import math

G = 9.81
RHO = 1.225  # kg/m^3, sea level. Use ~1.0 for 2000 m.

# (name, grams, avionics watts)
COMMON = [
    ("Flight controller (PX4, ICM-42688-P IMU, baro)", 10, 1.0),
    ("4-in-1 ESC, 55 A, 6S", 20, 0.0),
    ("GNSS + compass (u-blox M10)", 25, 0.5),
    ("Mesh radio (2x2 MIMO, OEM module) + 2 antennas", 25, 4.0),
    ("UWB ranging module (DW3000 class) + antenna", 10, 0.5),
    ("Forward stereo depth camera (global shutter, IMU)", 72, 2.5),
    ("Video camera (12 MP, CSI)", 20, 1.2),
    ("Downward camera (global shutter, dock tags)", 6, 0.4),
    ("Downward flow + ToF rangefinder", 6, 0.3),
    ("RC receiver (safety pilot, ELRS)", 3, 0.1),
    ("Power board (5 V / 12 V regulators, current sensor)", 15, 0.0),
    ("Dock interface: skids, charge contacts, latch plate", 50, 0.0),
    ("Wiring, connectors, fasteners, vibration mounts", 60, 0.0),
]

VARIANTS = {
    # 7-inch FPV class with Mid-360 and Orin NX. The design target.
    "baseline": dict(
        prop_in=7.0,
        parts=[
            ("Frame, 7 in carbon, payload deck", 190, 0.0),
            ("Motors 4x 2807 1300 KV", 208, 0.0),
            ("Props 4x 7 in bi-blade", 36, 0.0),
            ("3D lidar (Livox Mid-360 class) + mount", 280, 6.5),
            ("Companion: Jetson Orin NX 16 GB + carrier + cooler", 118, 13.0),
            ("Battery 6S1P 21700 Li-ion 4.5 Ah + leads", 450, 0.0),
        ],
        cells=6, cell_v=3.6, cell_ah=4.5, parallel=1,
    ),
    # Same payload, 8-inch props and a 6S2P pack: longer sorties, bigger dock.
    "long_endurance": dict(
        prop_in=8.0,
        parts=[
            ("Frame, 8 in carbon, payload deck", 230, 0.0),
            ("Motors 4x 2810 1100 KV", 240, 0.0),
            ("Props 4x 8 in bi-blade", 48, 0.0),
            ("3D lidar (Livox Mid-360 class) + mount", 280, 6.5),
            ("Companion: Jetson Orin NX 16 GB + carrier + cooler", 118, 13.0),
            ("Battery 6S2P 21700 Li-ion 9.0 Ah + leads", 880, 0.0),
        ],
        cells=6, cell_v=3.6, cell_ah=4.5, parallel=2,
    ),
    # Lighter build: VOXL 2-class SoC instead of Orin NX, depth camera dropped
    # (VOXL stereo pair does depth). Less GPU headroom for detection.
    "light": dict(
        prop_in=7.0,
        parts=[
            ("Frame, 7 in carbon, payload deck", 190, 0.0),
            ("Motors 4x 2807 1300 KV", 208, 0.0),
            ("Props 4x 7 in bi-blade", 36, 0.0),
            ("3D lidar (Livox Mid-360 class) + mount", 280, 6.5),
            ("Companion: VOXL 2-class SoC + stereo pair", 30, 7.0),
            ("Battery 6S1P 21700 Li-ion 4.5 Ah + leads", 450, 0.0),
        ],
        cells=6, cell_v=3.6, cell_ah=4.5, parallel=1,
        drop=("Forward stereo depth camera (global shutter, IMU)",),
    ),
}


def budget(v, fm=0.62, drive_eff=0.80, usable=0.80, rho=RHO):
    parts = [p for p in COMMON if p[0] not in v.get("drop", ())] + v["parts"]
    mass_kg = sum(p[1] for p in parts) / 1000.0
    avionics_w = sum(p[2] for p in parts)
    r = v["prop_in"] * 0.0254 / 2.0
    area = 4 * math.pi * r * r
    thrust = mass_kg * G
    ideal_w = thrust ** 1.5 / math.sqrt(2 * rho * area)
    hover_w = ideal_w / (fm * drive_eff)
    wh = v["cells"] * v["cell_v"] * v["cell_ah"] * v["parallel"]
    total_w = hover_w + avionics_w
    minutes = wh * usable / total_w * 60.0
    return parts, mass_kg, avionics_w, hover_w, total_w, wh, minutes


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--variant", choices=sorted(VARIANTS), action="append")
    ap.add_argument("--rho", type=float, default=RHO, help="air density kg/m^3")
    ap.add_argument("--usable", type=float, default=0.80,
                    help="fraction of pack energy used before landing (rest is reserve)")
    a = ap.parse_args()
    for name in a.variant or VARIANTS:
        parts, m, av, hov, tot, wh, mins = budget(VARIANTS[name], usable=a.usable, rho=a.rho)
        print(f"== {name} ({VARIANTS[name]['prop_in']:.0f} in props) ==")
        for p in parts:
            print(f"  {p[1]:5.0f} g  {p[2]:5.1f} W  {p[0]}")
        print(f"  -----")
        print(f"  all-up mass      {m*1000:6.0f} g")
        print(f"  avionics power   {av:6.1f} W")
        print(f"  hover power      {hov:6.1f} W (rotors)")
        print(f"  total power      {tot:6.1f} W")
        print(f"  pack energy      {wh:6.1f} Wh")
        print(f"  hover endurance  {mins:6.1f} min to {100*(1-a.usable):.0f}% reserve\n")


if __name__ == "__main__":
    main()
