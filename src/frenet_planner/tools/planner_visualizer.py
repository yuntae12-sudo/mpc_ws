#!/usr/bin/env python3
"""Live Frenet planner candidate viewer (ROS-free).

The planner atomically updates a JSON snapshot.  This tool polls that snapshot
and renders a global overview plus an ego-centred local candidate view.
"""

import argparse
import json
import math
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.collections import LineCollection
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
from matplotlib.patches import Polygon, Rectangle
from matplotlib.transforms import Affine2D


COLORS = {
    "valid": "#42c96b",
    "curvature": "#f0a12b",
    "collision": "#e64b4b",
}


def load_global_path(path: Path):
    points = []
    with path.open(encoding="utf-8") as stream:
        for line in stream:
            line = line.strip().replace(",", " ")
            if not line:
                continue
            fields = line.split()
            try:
                points.append((float(fields[0]), float(fields[1])))
            except (ValueError, IndexError):
                continue
    return points


def vehicle_polygon(x, y, yaw, length, width):
    hl, hw = length * 0.5, width * 0.5
    local = [(hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw)]
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * px - s * py, y + s * px + c * py) for px, py in local]


class PlannerVisualizer:
    def __init__(self, snapshot_path, global_path, local_radius, view_mode):
        self.snapshot_path = snapshot_path
        self.global_path = global_path
        self.local_radius = local_radius
        self.view_mode = view_mode
        self.history = []
        self.last_mtime_ns = -1
        self.snapshot = None
        self.local_view_initialized = False

        self.fig, (self.ax_global, self.ax_local) = plt.subplots(1, 2, figsize=(15, 8))
        self.fig.canvas.manager.set_window_title("Frenet Planner Candidate Viewer")
        self.artists = []
        self._initialize_axes()
        self._initialize_artists()
        self._initialize_legend()
        self.fig.suptitle("Frenet Planner — generated, rejected, valid and selected paths")
        self.fig.tight_layout()

    def read_snapshot(self):
        try:
            stat = self.snapshot_path.stat()
            if stat.st_mtime_ns == self.last_mtime_ns:
                return False
            with self.snapshot_path.open(encoding="utf-8") as stream:
                snapshot = json.load(stream)
            self.snapshot = snapshot
            self.last_mtime_ns = stat.st_mtime_ns
            ego = snapshot["ego"]
            self.history.append((ego["x"], ego["y"]))
            if len(self.history) > 4000:
                self.history = self.history[-4000:]
            return True
        except (FileNotFoundError, json.JSONDecodeError, KeyError, OSError):
            return False

    def _initialize_axes(self):
        for ax, title in ((self.ax_global, "Global overview"),
                          (self.ax_local, "Candidate detail")):
            ax.set_title(title)
            ax.set_aspect("equal", adjustable="box")
            ax.grid(True, alpha=0.2)
            ax.set_xlabel("x [m]")
            ax.set_ylabel("y [m]")

        if self.global_path:
            gx, gy = zip(*self.global_path)
            self.ax_global.plot(gx, gy, color="#777777", linewidth=1.0, alpha=0.65)
            self.local_reference, = self.ax_local.plot(
                gx, gy, color="#999999", linewidth=0.8, alpha=0.45)
            margin = 15.0
            self.ax_global.set_xlim(min(gx) - margin, max(gx) + margin)
            self.ax_global.set_ylim(min(gy) - margin, max(gy) + margin)

        r = self.local_radius
        self.ax_local.set_xlim(-r, r)
        self.ax_local.set_ylim(-r, r)

        if not self.global_path:
            self.local_reference, = self.ax_local.plot(
                [], [], color="#999999", linewidth=0.8, alpha=0.45)

    def _initialize_artists(self):
        self.global_history, = self.ax_global.plot([], [], color="#168aad", linewidth=1.5)
        self.local_history, = self.ax_local.plot([], [], color="#168aad", linewidth=1.5)
        self.global_selected, = self.ax_global.plot([], [], color="#d000ff", linewidth=3.0)
        self.local_selected, = self.ax_local.plot([], [], color="#d000ff", linewidth=3.0)

        self.candidate_collections = {}
        for status, color in COLORS.items():
            collection = LineCollection(
                [], colors=color,
                linewidths=0.65 if status != "valid" else 0.9,
                alpha=0.32 if status != "valid" else 0.45)
            self.ax_local.add_collection(collection)
            self.candidate_collections[status] = collection

        self.collision_scatter = self.ax_local.scatter(
            [], [], marker="x", s=18, color="#9d0208", alpha=0.7)
        self.merge_target_global = self.ax_global.scatter(
            [], [], marker="*", s=130, color="#7b2cbf", edgecolor="white", zorder=8)
        self.merge_target_local = self.ax_local.scatter(
            [], [], marker="*", s=130, color="#7b2cbf", edgecolor="white", zorder=8)

        empty_vehicle = vehicle_polygon(0.0, 0.0, 0.0, 4.5, 1.9)
        self.global_ego = Polygon(empty_vehicle, closed=True, facecolor="#0077b6",
                                  edgecolor="#023e8a", alpha=0.9)
        self.local_ego = Polygon(empty_vehicle, closed=True, facecolor="#0077b6",
                                 edgecolor="#023e8a", alpha=0.9)
        self.ax_global.add_patch(self.global_ego)
        self.ax_local.add_patch(self.local_ego)

        self.obstacle_patches = {self.ax_global: [], self.ax_local: []}
        self.obstacle_labels = []
        for ax in (self.ax_global, self.ax_local):
            for _ in range(20):
                rect = Rectangle((0.0, 0.0), 0.0, 0.0, visible=False,
                                 facecolor="#d62828", edgecolor="#780000", alpha=0.65)
                ax.add_patch(rect)
                self.obstacle_patches[ax].append(rect)
        for _ in range(20):
            label = self.ax_local.text(0.0, 0.0, "", fontsize=7,
                                       color="#590000", visible=False)
            self.obstacle_labels.append(label)

        self.info_text = self.ax_local.text(
            0.015, 0.985, f"Waiting for\n{self.snapshot_path}",
            transform=self.ax_local.transAxes, va="top", ha="left", fontsize=9,
            bbox=dict(boxstyle="round", facecolor="white", alpha=0.82))

        self.artists = [
            self.global_history, self.local_history,
            self.global_selected, self.local_selected,
            self.local_reference,
            *self.candidate_collections.values(), self.collision_scatter,
            self.merge_target_global, self.merge_target_local,
            self.global_ego, self.local_ego, self.info_text,
            *self.obstacle_patches[self.ax_global],
            *self.obstacle_patches[self.ax_local], *self.obstacle_labels,
        ]

    def _initialize_legend(self):
        handles = [
            Line2D([], [], color="#777777", linewidth=1.0, label="global path"),
            Line2D([], [], color="#168aad", linewidth=1.5, label="ego history"),
            Line2D([], [], color=COLORS["valid"], linewidth=1.0, label="valid"),
            Line2D([], [], color=COLORS["curvature"], linewidth=1.0, label="curvature reject"),
            Line2D([], [], color=COLORS["collision"], linewidth=1.0, label="collision reject"),
            Line2D([], [], color="#d000ff", linewidth=3.0, label="selected"),
            Patch(facecolor="#0077b6", edgecolor="#023e8a", label="ego"),
            Patch(facecolor="#d62828", edgecolor="#780000", label="obstacle"),
            Line2D([], [], marker="*", color="none", markerfacecolor="#7b2cbf",
                   markeredgecolor="white", markersize=11, label="merge conflict point"),
        ]
        self.ax_local.legend(handles=handles, loc="lower right", fontsize=8)

    def _update_obstacles(self, ax, obstacles, labels=False, origin=(0.0, 0.0), merge=None):
        patches = self.obstacle_patches[ax]
        origin_x, origin_y = origin
        for index, patch in enumerate(patches):
            if index >= len(obstacles):
                patch.set_visible(False)
                if labels:
                    self.obstacle_labels[index].set_visible(False)
                continue
            obj = obstacles[index]
            width = max(float(obj.get("width", 0.0)), 0.3)
            length = max(float(obj.get("length", 0.0)), 0.3)
            patch.set_xy((-length / 2.0, -width / 2.0))
            patch.set_width(length)
            patch.set_height(width)
            patch.set_transform(
                Affine2D().rotate(float(obj.get("heading", 0.0)))
                .translate(float(obj["x"]) - origin_x,
                           float(obj["y"]) - origin_y) + ax.transData)
            obj_id = int(obj.get("id", -1))
            if merge and merge.get("active") and obj_id == int(merge.get("sa_id", -2)):
                patch.set_facecolor("#ffb703")
                patch.set_edgecolor("#9c6500")
            elif merge and merge.get("active") and obj_id == int(merge.get("sb_id", -2)):
                patch.set_facecolor("#3a86ff")
                patch.set_edgecolor("#003f88")
            else:
                patch.set_facecolor("#d62828")
                patch.set_edgecolor("#780000")
            patch.set_visible(True)
            if labels:
                label = self.obstacle_labels[index]
                label.set_position((float(obj["x"]) - origin_x,
                                    float(obj["y"]) - origin_y))
                role = ""
                if merge and merge.get("active"):
                    if obj_id == int(merge.get("sa_id", -2)):
                        role = " preceding"
                    elif obj_id == int(merge.get("sb_id", -2)):
                        role = " following"
                label.set_text(f"{obj.get('id', '?')}{role}")
                label.set_visible(True)

    def update(self, _frame):
        if not self.read_snapshot():
            return self.artists

        snapshot = self.snapshot
        ego = snapshot["ego"]
        stats = snapshot.get("stats", {})
        candidates = snapshot.get("candidates", [])
        fixed_origin = (ego["x"], ego["y"]) if self.view_mode == "fixed" else (0.0, 0.0)
        origin_x, origin_y = fixed_origin

        def to_local(points):
            if self.view_mode != "fixed":
                return points
            return [(point[0] - origin_x, point[1] - origin_y) for point in points]

        hx, hy = zip(*self.history) if self.history else ([], [])
        self.global_history.set_data(hx, hy)
        local_history = to_local(self.history)
        lhx, lhy = zip(*local_history) if local_history else ([], [])
        self.local_history.set_data(lhx, lhy)

        if self.global_path:
            local_reference = to_local(self.global_path)
            rgx, rgy = zip(*local_reference)
            self.local_reference.set_data(rgx, rgy)

        grouped = {key: [] for key in COLORS}
        selected = []
        collision_points = []
        for candidate in candidates:
            xy = candidate.get("xy", [])
            if len(xy) < 2:
                continue
            if candidate.get("selected"):
                selected = to_local(xy)
            else:
                grouped.setdefault(candidate.get("status", "valid"), []).append(to_local(xy))
            collision_index = candidate.get("collision_sample_index", -1)
            if candidate.get("status") == "collision" and 0 <= collision_index < len(xy):
                collision_points.append(to_local([xy[collision_index]])[0])

        for status, collection in self.candidate_collections.items():
            collection.set_segments(grouped.get(status, []))

        if selected:
            sx, sy = zip(*selected)
        else:
            sx, sy = [], []
        if selected:
            world_selected = next(
                (candidate.get("xy", []) for candidate in candidates
                 if candidate.get("selected")), [])
            wsx, wsy = zip(*world_selected) if world_selected else ([], [])
        else:
            wsx, wsy = [], []
        self.global_selected.set_data(wsx, wsy)
        self.local_selected.set_data(sx, sy)
        self.collision_scatter.set_offsets(collision_points if collision_points else [(math.nan, math.nan)])

        ego_xy = vehicle_polygon(ego["x"], ego["y"], ego["yaw"], 4.5, 1.9)
        self.global_ego.set_xy(ego_xy)
        self.local_ego.set_xy(to_local(ego_xy))
        obstacles = snapshot.get("obstacles", [])[:20]
        merge = snapshot.get("merge", {})
        self._update_obstacles(self.ax_global, obstacles, merge=merge)
        self._update_obstacles(self.ax_local, obstacles, labels=True,
                               origin=fixed_origin, merge=merge)

        if merge.get("active"):
            target_world = [(float(merge.get("target_x", 0.0)),
                             float(merge.get("target_y", 0.0)))]
            self.merge_target_global.set_offsets(target_world)
            self.merge_target_local.set_offsets(to_local(target_world))
        else:
            empty = [(math.nan, math.nan)]
            self.merge_target_global.set_offsets(empty)
            self.merge_target_local.set_offsets(empty)

        # fixed는 모든 local artist를 ego 기준 상대좌표로 변환하므로 축 역시
        # 항상 [-r,+r]에 고정해야 ego=(0,0)가 정확히 중앙에 온다. 예전 조건은
        # fixed의 첫 프레임에도 world 좌표 ego±r로 축을 바꿔, 상대좌표로 그린 ego가
        # 화면 가장자리로 밀리는 원인이었다. world 좌표 축 이동은 follow에서만 한다.
        if self.view_mode == "follow":
            r = self.local_radius
            self.ax_local.set_xlim(ego["x"] - r, ego["x"] + r)
            self.ax_local.set_ylim(ego["y"] - r, ego["y"] + r)
            self.local_view_initialized = True

        curv_rejected = stats.get("combined_total", 0) - stats.get("after_curvature", 0)
        collision_rejected = stats.get("after_curvature", 0) - stats.get("after_collision", 0)
        lat_rejected = stats.get("lateral_total", 0) - stats.get("lateral_valid", 0)
        lon_rejected = stats.get("longitudinal_total", 0) - stats.get("longitudinal_valid", 0)
        info = (
            f"mode: {snapshot.get('mode', '?')}"
            f" / {snapshot.get('phase', 'NONE')}\n"
            f"speed: {ego.get('v', 0.0):.2f} m/s"
            f"   target: {snapshot.get('target_speed', 0.0):.2f} m/s"
            f"   d: {ego.get('d', 0.0):.2f} m\n"
            f"combined: {stats.get('combined_total', 0)}   valid: {stats.get('after_collision', 0)}\n"
            f"rejected  lat_acc:{lat_rejected}  lon_acc:{lon_rejected}\n"
            f"          curvature:{curv_rejected}  collision:{collision_rejected}"
        )
        if merge.get("active"):
            conflict_gap = float(merge.get("conflict_s", 0.0)) - float(ego.get("s", 0.0))
            merge_type = str(merge.get("type", "ROUNDABOUT"))
            info += (
                f"\n{merge_type} MERGE safe:{merge.get('gap_safe', False)}"
                f" conflict_gap:{conflict_gap:.1f} m"
                f" entry_t:{float(merge.get('entry_time', 0.0)):.1f} s"
                f"\npreceding:{merge.get('sa_id', '?')}"
                f"  following:{merge.get('sb_id', '?')}"
            )
        self.info_text.set_text(info)
        return self.artists

    def run(self):
        self.animation = FuncAnimation(self.fig, self.update, interval=150,
                                       cache_frame_data=False,
                                       blit=(self.view_mode == "fixed"))
        plt.show()


def main():
    script_dir = Path(__file__).resolve().parent
    default_global = script_dir.parent / "src/config/2026_molit_comp_global_path.txt"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--snapshot", type=Path,
                        default=Path("/tmp/frenet_planner_debug.json"))
    parser.add_argument("--global-path", type=Path, default=default_global)
    parser.add_argument("--local-radius", type=float, default=60.0,
                        help="half-width of the local view in metres (default: 60)")
    parser.add_argument("--view", choices=("fixed", "follow"), default="fixed",
                        help="fixed uses a flicker-free ego-centred viewport; "
                             "follow pans world-coordinate axes with ego")
    args = parser.parse_args()

    global_path = load_global_path(args.global_path)
    PlannerVisualizer(args.snapshot, global_path, args.local_radius, args.view).run()


if __name__ == "__main__":
    main()
