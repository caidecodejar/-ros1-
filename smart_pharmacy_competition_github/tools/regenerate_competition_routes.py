#!/usr/bin/env python
from __future__ import annotations

import csv
from dataclasses import dataclass
from datetime import date
import heapq
import math
from pathlib import Path
import shutil
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

from PIL import Image, ImageDraw, ImageFont
import yaml


ROOT = Path(__file__).resolve().parents[1]
DOC_DIR = ROOT / "docs" / "waypoint_sampling_20260603"
ROS_CONFIG_DIR = ROOT / "ros1_smart_pharmacy_patrol" / "config"

MAP_YAML = DOC_DIR / "final_rviz_map_20260603.local.yaml"
MAP_IMAGE = DOC_DIR / "final_rviz_map_20260603.pgm"
WAYPOINTS = DOC_DIR / "waypoints_real.yaml"
CLEAN_WAYPOINTS_CSV = DOC_DIR / "competition_clean_waypoints.csv"

ROBOT_MAP_YAML = "/home/EPRobot/smart_pharmacy_patrol_runtime/rviz_saved_map/final_rviz_map_20260603.yaml"
ROBOT_WAYPOINTS = "/home/EPRobot/robot_ws/src/ros1_smart_pharmacy_patrol/config/waypoints_real.yaml"

ROUTES = {
    "default_AB_to_lab1": {
        "description": "Matches patrol_mission.py defaults: board1=AB:1, board2=FREE.",
        "board1_payload": "AB:1",
        "board2_payload": "FREE",
        "sequence": ["start", "board_1", "window_A", "window_B", "board_2", "lab_window_1", "start"],
    },
    "sampled_manual_loop": {
        "description": "Route order followed during the latest manual sampling session, simplified without the repeated lab_window_3 resample.",
        "sequence": ["start", "board_1", "window_C", "board_2", "lab_window_1", "lab_window_2", "lab_window_3", "lab_window_4", "start"],
    },
    "all_targets_coverage": {
        "description": "Coverage route for checking every sampled stop. It is a goal order, not a precomputed collision-free global path.",
        "sequence": ["start", "board_1", "window_A", "window_B", "window_C", "board_2", "lab_window_1", "lab_window_2", "lab_window_3", "lab_window_4", "start"],
    },
}


@dataclass(frozen=True)
class MapInfo:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float


GridPoint = Tuple[int, int]
WorldPoint = Tuple[float, float]


def load_yaml(path: Path):
    with path.open("r", encoding="utf-8-sig") as handle:
        return yaml.safe_load(handle) or {}


def write_yaml(path: Path, data) -> None:
    with path.open("w", encoding="utf-8", newline="\n") as handle:
        yaml.safe_dump(data, handle, allow_unicode=True, sort_keys=False, default_flow_style=False)


def load_map() -> Tuple[MapInfo, List[List[int]], List[List[bool]]]:
    meta = load_yaml(MAP_YAML)
    resolution = float(meta["resolution"])
    origin = meta["origin"]
    image = Image.open(MAP_IMAGE).convert("L")
    width, height = image.size
    values = list(image.getdata())

    grid: List[List[int]] = []
    free: List[List[bool]] = []
    for y in range(height):
        row = values[y * width : (y + 1) * width]
        grid.append(row)
        # ROS map pixels: 254 free, 205 unknown, 0 occupied. Keep A* conservative:
        # only known-free cells are used, then endpoints may snap to nearest free cell.
        free.append([value >= 250 for value in row])
    return MapInfo(width, height, resolution, float(origin[0]), float(origin[1])), grid, free


def world_to_grid(info: MapInfo, x: float, y: float) -> GridPoint:
    gx = int(round((x - info.origin_x) / info.resolution))
    gy_map = int(round((y - info.origin_y) / info.resolution))
    gy = info.height - 1 - gy_map
    return max(0, min(info.width - 1, gx)), max(0, min(info.height - 1, gy))


def grid_to_world(info: MapInfo, gx: int, gy: int) -> WorldPoint:
    x = info.origin_x + gx * info.resolution
    y = info.origin_y + (info.height - 1 - gy) * info.resolution
    return round(x, 3), round(y, 3)


def neighbors(point: GridPoint) -> Iterable[Tuple[GridPoint, float]]:
    x, y = point
    for dx, dy, cost in (
        (-1, 0, 1.0),
        (1, 0, 1.0),
        (0, -1, 1.0),
        (0, 1, 1.0),
        (-1, -1, math.sqrt(2.0)),
        (-1, 1, math.sqrt(2.0)),
        (1, -1, math.sqrt(2.0)),
        (1, 1, math.sqrt(2.0)),
    ):
        yield (x + dx, y + dy), cost


def in_bounds(info: MapInfo, point: GridPoint) -> bool:
    x, y = point
    return 0 <= x < info.width and 0 <= y < info.height


def nearest_free(info: MapInfo, free: List[List[bool]], point: GridPoint) -> Tuple[GridPoint, float]:
    if in_bounds(info, point) and free[point[1]][point[0]]:
        return point, 0.0

    best: Optional[Tuple[float, GridPoint]] = None
    max_radius = max(info.width, info.height)
    px, py = point
    for radius in range(1, max_radius + 1):
        for y in range(max(0, py - radius), min(info.height - 1, py + radius) + 1):
            for x in (px - radius, px + radius):
                if 0 <= x < info.width and free[y][x]:
                    dist = math.hypot(x - px, y - py)
                    if best is None or dist < best[0]:
                        best = (dist, (x, y))
        for x in range(max(0, px - radius), min(info.width - 1, px + radius) + 1):
            for y in (py - radius, py + radius):
                if 0 <= y < info.height and free[y][x]:
                    dist = math.hypot(x - px, y - py)
                    if best is None or dist < best[0]:
                        best = (dist, (x, y))
        if best is not None:
            return best[1], round(best[0] * info.resolution, 3)
    raise RuntimeError("no free cell exists in map")


def astar(info: MapInfo, free: List[List[bool]], start: GridPoint, goal: GridPoint) -> List[GridPoint]:
    def heuristic(a: GridPoint, b: GridPoint) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    open_heap: List[Tuple[float, int, GridPoint]] = []
    heapq.heappush(open_heap, (heuristic(start, goal), 0, start))
    came_from: Dict[GridPoint, GridPoint] = {}
    g_score: Dict[GridPoint, float] = {start: 0.0}
    counter = 0

    while open_heap:
        _, _, current = heapq.heappop(open_heap)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))

        for nxt, move_cost in neighbors(current):
            if not in_bounds(info, nxt):
                continue
            if not free[nxt[1]][nxt[0]]:
                continue
            tentative = g_score[current] + move_cost
            if tentative < g_score.get(nxt, float("inf")):
                came_from[nxt] = current
                g_score[nxt] = tentative
                counter += 1
                heapq.heappush(open_heap, (tentative + heuristic(nxt, goal), counter, nxt))
    raise RuntimeError("A* failed from %s to %s" % (start, goal))


def rdp(points: Sequence[WorldPoint], epsilon: float) -> List[WorldPoint]:
    if len(points) <= 2:
        return list(points)

    def perpendicular_distance(point: WorldPoint, start: WorldPoint, end: WorldPoint) -> float:
        px, py = point
        sx, sy = start
        ex, ey = end
        dx = ex - sx
        dy = ey - sy
        if dx == 0 and dy == 0:
            return math.hypot(px - sx, py - sy)
        return abs(dy * px - dx * py + ex * sy - ey * sx) / math.hypot(dx, dy)

    max_dist = 0.0
    index = 0
    for i in range(1, len(points) - 1):
        dist = perpendicular_distance(points[i], points[0], points[-1])
        if dist > max_dist:
            index = i
            max_dist = dist
    if max_dist > epsilon:
        left = rdp(points[: index + 1], epsilon)
        right = rdp(points[index:], epsilon)
        return left[:-1] + right
    return [points[0], points[-1]]


def path_length(points: Sequence[WorldPoint]) -> float:
    return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(points, points[1:]))


def route_straight_total(waypoints: Dict, sequence: Sequence[str]) -> float:
    total = 0.0
    for start, goal in zip(sequence, sequence[1:]):
        a = waypoints[start]
        b = waypoints[goal]
        total += math.hypot(float(b["x"]) - float(a["x"]), float(b["y"]) - float(a["y"]))
    return round(total, 3)


def load_waypoint_stability() -> Dict[str, Dict[str, str]]:
    result: Dict[str, Dict[str, str]] = {}
    with CLEAN_WAYPOINTS_CSV.open("r", encoding="utf-8-sig", newline="") as handle:
        for row in csv.DictReader(handle):
            result[row["name"]] = {
                "stability": row["stability"],
                "x_range_m": float(row["x_range_m"]),
                "y_range_m": float(row["y_range_m"]),
                "yaw_std_deg": float(row["yaw_std_deg"]),
            }
    return result


def build_route_files(info: MapInfo, free: List[List[bool]]) -> Tuple[Dict, Dict, List[Dict], List[Dict]]:
    waypoints = load_yaml(WAYPOINTS)["waypoints"]
    stability = load_waypoint_stability()

    route_yaml = {
        "generated": str(date.today()),
        "purpose": "Clean ROS1 waypoint route for offline smart-pharmacy autonomous navigation competition testing.",
        "map": {
            "robot_yaml": ROBOT_MAP_YAML,
            "local_yaml": str(MAP_YAML),
            "image": str(MAP_IMAGE),
            "resolution": info.resolution,
            "origin": [info.origin_x, info.origin_y, 0.0],
            "size_px": [info.width, info.height],
            "size_m": [round(info.width * info.resolution, 3), round(info.height * info.resolution, 3)],
        },
        "cleaning_policy": {
            "source": "Stationary TF median samples from map->base_footprint plus /scan_filtered quality stats.",
            "jump_removal": "Raw TF samples are not used as route points. Each stop is represented by its median x/y and circular-mean yaw.",
            "stability_gate": "good and usable samples are retained; unstable samples must be re-sampled before competition use.",
            "planner_note": "Sequences are move_base goal orders. The global planner still computes collision-free paths on the map between adjacent goals.",
        },
        "waypoints": {},
        "route_sets": {},
        "launch_command": (
            "roslaunch ros1_smart_pharmacy_patrol patrol.launch send_goals:=true "
            "waypoints:=%s board1:=AB:1 board2:=FREE" % ROBOT_WAYPOINTS
        ),
    }

    for name, point in waypoints.items():
        entry = {
            "x": float(point["x"]),
            "y": float(point["y"]),
            "theta": float(point["theta"]),
        }
        entry.update(stability.get(name, {}))
        route_yaml["waypoints"][name] = entry

    segment_rows: List[Dict] = []
    for route_name, route in ROUTES.items():
        sequence = route["sequence"]
        route_entry = dict(route)
        route_entry["straight_segment_total_m"] = route_straight_total(waypoints, sequence)
        route_yaml["route_sets"][route_name] = route_entry
        for step, (start, goal) in enumerate(zip(sequence, sequence[1:]), start=1):
            a = waypoints[start]
            b = waypoints[goal]
            dx = float(b["x"]) - float(a["x"])
            dy = float(b["y"]) - float(a["y"])
            heading = math.atan2(dy, dx)
            segment_rows.append(
                {
                    "route": route_name,
                    "step": step,
                    "from": start,
                    "to": goal,
                    "from_x": float(a["x"]),
                    "from_y": float(a["y"]),
                    "to_x": float(b["x"]),
                    "to_y": float(b["y"]),
                    "distance_m": math.hypot(dx, dy),
                    "heading_rad": heading,
                    "heading_deg": math.degrees(heading),
                }
            )

    planned_yaml = {
        "generated": str(date.today()),
        "planner": "offline A* on saved occupancy map free cells, simplified with RDP epsilon 0.04m",
        "map": {
            "yaml": str(DOC_DIR / "final_rviz_map_20260603.yaml"),
            "image": str(MAP_IMAGE),
            "resolution": info.resolution,
            "origin": [info.origin_x, info.origin_y, 0.0],
        },
        "note": "These paths are cleaned visualization/reference polylines. Runtime ROS move_base should still plan between the same waypoint goals using costmaps.",
        "planned_routes": {},
    }
    point_rows: List[Dict] = []

    for route_name, route in ROUTES.items():
        segments = []
        grid_total = 0.0
        for step, (start_name, goal_name) in enumerate(zip(route["sequence"], route["sequence"][1:]), start=1):
            start_wp = waypoints[start_name]
            goal_wp = waypoints[goal_name]
            start_raw = world_to_grid(info, float(start_wp["x"]), float(start_wp["y"]))
            goal_raw = world_to_grid(info, float(goal_wp["x"]), float(goal_wp["y"]))
            start_grid, start_snap = nearest_free(info, free, start_raw)
            goal_grid, goal_snap = nearest_free(info, free, goal_raw)
            grid_path = astar(info, free, start_grid, goal_grid)
            world_path = [grid_to_world(info, x, y) for x, y in grid_path]
            clean_path = rdp(world_path, 0.04)
            length = path_length(world_path)
            grid_total += length
            segment = {
                "step": step,
                "from": start_name,
                "to": goal_name,
                "snap_from_m": start_snap,
                "snap_to_m": goal_snap,
                "grid_length_m": round(length, 3),
                "raw_point_count": len(world_path),
                "clean_point_count": len(clean_path),
                "clean_path": [[x, y] for x, y in clean_path],
            }
            segments.append(segment)
            for index, (x, y) in enumerate(clean_path):
                point_rows.append(
                    {
                        "route": route_name,
                        "step": step,
                        "from": start_name,
                        "to": goal_name,
                        "point_index": index,
                        "x": x,
                        "y": y,
                    }
                )
        planned_yaml["planned_routes"][route_name] = {
            "sequence": route["sequence"],
            "grid_total_m": round(grid_total, 3),
            "segments": segments,
        }

    return route_yaml, planned_yaml, segment_rows, point_rows


def write_csv(path: Path, rows: Sequence[Dict], fields: Sequence[str]) -> None:
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def draw_route(info: MapInfo, route_name: str, planned_yaml: Dict, suffix: str) -> None:
    image = Image.open(MAP_IMAGE).convert("RGB")
    draw = ImageDraw.Draw(image)
    scale = 3
    image = image.resize((image.width * scale, image.height * scale), Image.Resampling.NEAREST)
    draw = ImageDraw.Draw(image)

    def px(point: WorldPoint) -> Tuple[int, int]:
        gx, gy = world_to_grid(info, point[0], point[1])
        return gx * scale, gy * scale

    colors = ["#f94144", "#f3722c", "#f9c74f", "#43aa8b", "#577590", "#9b5de5", "#00bbf9", "#90be6d", "#f15bb5", "#00f5d4"]
    route = planned_yaml["planned_routes"][route_name]
    for idx, segment in enumerate(route["segments"]):
        points = [tuple(item) for item in segment["clean_path"]]
        color = colors[idx % len(colors)]
        if len(points) >= 2:
            draw.line([px(point) for point in points], fill=color, width=max(2, 2 * scale))
        for point in points:
            x, y = px(point)
            r = 2 * scale
            draw.ellipse((x - r, y - r, x + r, y + r), fill=color, outline="white")

    waypoints = load_yaml(WAYPOINTS)["waypoints"]
    for name in route["sequence"]:
        point = waypoints[name]
        x, y = px((float(point["x"]), float(point["y"])))
        r = 4 * scale
        draw.ellipse((x - r, y - r, x + r, y + r), fill="white", outline="black", width=2)
        draw.text((x + 5 * scale, y - 5 * scale), name, fill="black")

    image.save(DOC_DIR / f"competition_route_{suffix}_astar.png")


def draw_straight_route(info: MapInfo, route_name: str, suffix: str) -> None:
    image = Image.open(MAP_IMAGE).convert("RGB")
    scale = 3
    image = image.resize((image.width * scale, image.height * scale), Image.Resampling.NEAREST)
    draw = ImageDraw.Draw(image)

    def px(point: WorldPoint) -> Tuple[int, int]:
        gx, gy = world_to_grid(info, point[0], point[1])
        return gx * scale, gy * scale

    waypoints = load_yaml(WAYPOINTS)["waypoints"]
    sequence = ROUTES[route_name]["sequence"]
    points = [(float(waypoints[name]["x"]), float(waypoints[name]["y"])) for name in sequence]
    draw.line([px(point) for point in points], fill="#2b6cff", width=max(2, 2 * scale))
    for name, point in zip(sequence, points):
        x, y = px(point)
        r = 4 * scale
        draw.ellipse((x - r, y - r, x + r, y + r), fill="white", outline="black", width=2)
        draw.text((x + 5 * scale, y - 5 * scale), name, fill="black")
    image.save(DOC_DIR / f"competition_route_{suffix}.png")


def sync_to_ros_config() -> None:
    ROS_CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    for name in ("waypoints_real.yaml", "competition_navigation_route.yaml", "competition_navigation_planned_paths.yaml"):
        shutil.copy2(DOC_DIR / name, ROS_CONFIG_DIR / name)


def main() -> None:
    info, _, free = load_map()
    route_yaml, planned_yaml, segment_rows, point_rows = build_route_files(info, free)

    write_yaml(DOC_DIR / "competition_navigation_route.yaml", route_yaml)
    write_yaml(DOC_DIR / "competition_navigation_planned_paths.yaml", planned_yaml)
    write_csv(
        DOC_DIR / "competition_navigation_route_segments.csv",
        segment_rows,
        ["route", "step", "from", "to", "from_x", "from_y", "to_x", "to_y", "distance_m", "heading_rad", "heading_deg"],
    )
    write_csv(
        DOC_DIR / "competition_navigation_planned_path_points.csv",
        point_rows,
        ["route", "step", "from", "to", "point_index", "x", "y"],
    )
    draw_straight_route(info, "default_AB_to_lab1", "default_AB_lab1")
    draw_straight_route(info, "all_targets_coverage", "all_targets_cleaned")
    draw_route(info, "default_AB_to_lab1", planned_yaml, "default_AB_lab1")
    draw_route(info, "sampled_manual_loop", planned_yaml, "sampled_manual_loop")
    draw_route(info, "all_targets_coverage", planned_yaml, "all_targets")
    sync_to_ros_config()

    print("Regenerated route files in:", DOC_DIR)
    print("Synced ROS config to:", ROS_CONFIG_DIR)
    for route_name, route in planned_yaml["planned_routes"].items():
        print("%s: %s, %.3fm" % (route_name, " -> ".join(route["sequence"]), route["grid_total_m"]))


if __name__ == "__main__":
    main()
