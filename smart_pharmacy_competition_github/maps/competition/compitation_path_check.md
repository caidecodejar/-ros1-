# Competition Map Path Check

- Source folder: `maps/competition`
- YAML image field: `compitation..pgm`; exists: `False`
- Fixed YAML preview: `maps/competition\compitation.fixed.yaml`
- Map: 128 x 152, resolution 0.050 m/pixel, origin [-3.405656, -1.957087, 0.000]
- Cell stats: free=4912, unknown=12942, occupied=1602
- Assumed loop route: start -> A -> B -> C -> 1 -> 2 -> 3 -> 4 -> start

## Points
- start: world=(0.000, 0.000), pixel=(68, 112), cell=free, nearest_free_distance=0.0
- A: world=(0.700, 2.550), pixel=(82, 61), cell=free, nearest_free_distance=0.0
- B: world=(1.501, 3.050), pixel=(98, 51), cell=free, nearest_free_distance=0.0
- C: world=(1.420, 2.060), pixel=(97, 71), cell=free, nearest_free_distance=0.0
- 1: world=(-1.584, 2.530), pixel=(36, 61), cell=free, nearest_free_distance=0.0
- 2: world=(-0.830, 1.820), pixel=(52, 75), cell=free, nearest_free_distance=0.0
- 3: world=(-1.615, 1.580), pixel=(36, 80), cell=free, nearest_free_distance=0.0
- 4: world=(-0.807, 0.980), pixel=(52, 92), cell=free, nearest_free_distance=0.0

## Segments
- start->A: reachable, path_len=3.59m, straight_clear=False
- A->B: reachable, path_len=1.01m, straight_clear=True
- B->C: reachable, path_len=1.02m, straight_clear=True
- C->1: reachable, path_len=4.55m, straight_clear=False
- 1->2: NOT reachable, reason=no path in free grid after obstacle inflation, straight_clear=False
- 2->3: NOT reachable, reason=no path in free grid after obstacle inflation, straight_clear=False
- 3->4: NOT reachable, reason=no path in free grid after obstacle inflation, straight_clear=False
- 4->start: NOT reachable, reason=no path in free grid after obstacle inflation, straight_clear=False

## Verdict
- The map will not load in ROS as-is because `compitation.yaml` references `compitation..pgm`, while the actual file is `compitation.pgm`.
- Grid-level route check does not fully pass; at least one segment cannot be planned under conservative unknown/obstacle handling.

