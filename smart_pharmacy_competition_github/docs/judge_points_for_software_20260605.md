# Judge software point coordinates 2026-06-05

Use this section for the judge software.

```text
start: [0.000, 0.000]
A:     [0.700, 2.550]
B:     [1.501, 3.050]
C:     [1.420, 2.060]
1:     [-1.584, 2.530]
2:     [-0.830, 1.820]
3:     [-1.615, 1.580]
4:     [-0.807, 0.980]
```

Table:

| Point | X | Y |
| --- | ---: | ---: |
| start | 0.000 | 0.000 |
| A | 0.700 | 2.550 |
| B | 1.501 | 3.050 |
| C | 1.420 | 2.060 |
| 1 | -1.584 | 2.530 |
| 2 | -0.830 | 1.820 |
| 3 | -1.615 | 1.580 |
| 4 | -0.807 | 0.980 |

Source:

```text
Original point-info text file in the final point/map folder.
```

The source file is encoded incorrectly in the local terminal, so this file is the clean readable copy.

Map recalculation check:

```text
map:        compitation.yaml in the final point/map folder
image:      compitation.pgm
resolution: 0.05 m/pixel
origin:     [-3.405656, -1.957087, 0]
image size: 128 x 152 px
```

Recalculated raw map coordinates from pixel markers:

| Point | Pixel X | Pixel Y | Raw X | Raw Y |
| --- | ---: | ---: | ---: | ---: |
| start | 68 | 112 | -0.006 | 0.043 |
| A | 82 | 61 | 0.694 | 2.593 |
| B | 98 | 51 | 1.494 | 3.093 |
| C | 97 | 71 | 1.444 | 2.093 |
| 1 | 36 | 61 | -1.606 | 2.593 |
| 2 | 52 | 75 | -0.806 | 1.893 |
| 3 | 36 | 80 | -1.606 | 1.643 |
| 4 | 52 | 92 | -0.806 | 1.043 |

Important:

```text
The judge software should use the first coordinate block, not ROS waypoints_real.yaml.
ROS waypoints are for robot navigation and use a different map origin/frame.
```
