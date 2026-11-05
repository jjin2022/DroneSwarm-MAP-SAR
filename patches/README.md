# PX4 patches

Local fixes to `~/dev/PX4-Autopilot` (v1.17) that this project depends on.
Apply, then rebuild SITL:

```bash
cd ~/dev/PX4-Autopilot
git apply ~/dev/DroneSwarm-MAP-SAR/patches/px4-sitl-hrt-double-enqueue.patch
make px4_sitl_default
```

## px4-sitl-hrt-double-enqueue.patch

**Symptom:** in about 1.5% of SITL flights (3 of 199), PX4's simulated battery, system
power and CPU-load messages stop for good at the moment of arming, while every other
topic keeps flowing. PX4 then reports "Battery unhealthy", and patrol_lidar (no recent
battery data counts as low battery) turns back straight after take-off.

**Cause:** on POSIX, `hrt_call_invoke()` (platforms/posix/src/px4/common/drv_hrt.cpp)
takes a due timer entry off the queue, releases the lock, runs its callout, then puts
periodic entries back. If another thread re-arms that entry while the lock is released,
the entry is put in the queue twice. That corrupts the singly-linked list, and the
entries behind it are dropped without any error. On NuttX the callout runs with
interrupts off, so the race can't happen there.

**Fix:** remove the entry if it's already queued before re-entering it, both in
`hrt_call_invoke()` and in `hrt_call_internal()` (`sq_rem` is a no-op when absent).

## Related fixes in this repo (not PX4)

- The runner keeps the computer awake (`systemd-inhibit`); a suspend mid-sortie froze it.
- `patrol_rtl`/`patrol_lidar` and `uwb_locator` measure data age with a monotonic
  clock. After a 14 h suspend the system clock was stepped ~2 s by time sync; on the
  wall clock every message suddenly looked 2 s old and the drone landed in place.
