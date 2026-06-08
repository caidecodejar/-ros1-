# WindowA Arrival Power-Loss Note

Date: 2026-06-05

## Event

The robot was manually keyboard-driven from the current `board_1` area toward `window_A`.
The user reported that the robot had arrived at `window_A`, but the robot then lost power.

## Snapshot Result

An SSH/ROS snapshot was attempted immediately after the report, but the robot was no longer reachable:

```text
Host: 192.168.12.1
Result: connection timed out / unreachable
```

Because the robot was already powered down, the live `map -> base_footprint` TF pose could not be saved.

## Last Known Planned WindowA Waypoint

The existing sampled `window_A` waypoint remains:

```yaml
window_A:
  x: -0.668
  y: 1.259
  theta: 0.996
```

## Recovery After Charging

After the robot is charged:

1. Keep the robot at its physical current position if possible.
2. Reconnect to the robot WiFi.
3. SSH to the robot and restart/sync the ROS1 environment.
4. Reopen RViz/noVNC.
5. If the robot is still physically at `window_A`, publish or verify localization near:

```yaml
x: -0.668
y: 1.259
theta: 0.996
```

If the robot is moved during charging, place it back at the known start point and republish the start pose before continuing.
