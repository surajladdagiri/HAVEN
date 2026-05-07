#!/usr/bin/env python3.11
"""Live 3D viewer for HAVEN's iPhone AR mesh stream.

The iPhone app publishes TCP packets on port 8080. This viewer connects to the
phone, consumes mesh + camera packets, renders a lightweight point cloud, and
keeps the desktop camera centered on the phone's current pose.

It also includes a `--demo` mode so the desktop viewer can be exercised without
an active phone connection.
"""

from __future__ import annotations

import argparse
import math
import os
import socket
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Iterator, Optional

import numpy as np

MESH_HEADER = b"MESH--------------------------------"
CAMERA_HEADER = b"CAMERA------------------------------"
LEGACY_POSE_HEADER = b"POSE--------------------------------"
HEADER_SIZE = 36
COUNT_SIZE = 4
UUID_SIZE = 36
MAX_FLOAT_COUNT = 4_000_000


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Render HAVEN's live iPhone AR mesh stream on a desktop."
    )
    parser.add_argument("--host", help="iPhone IP address shown in the HAVEN app.")
    parser.add_argument("--port", type=int, default=8080, help="TCP port exposed by the iPhone app.")
    parser.add_argument("--demo", action="store_true", help="Run a synthetic in-process demo instead of connecting to a phone.")
    parser.add_argument(
        "--backend",
        choices=("auto", "open3d", "matplotlib"),
        default="auto",
        help="Desktop renderer backend. 'auto' prefers Open3D for live viewing.",
    )
    parser.add_argument(
        "--follow-mode",
        choices=("chase", "first_person"),
        default="chase",
        help="Camera-follow style. 'chase' is easier to demo and debug; 'first_person' is closer to the phone view.",
    )
    parser.add_argument("--voxel-size", type=float, default=0.08, help="Point cloud voxel size in meters.")
    parser.add_argument("--max-points", type=int, default=18_000, help="Maximum rendered points after downsampling.")
    parser.add_argument("--max-points-per-anchor", type=int, default=4_000, help="Maximum kept points per anchor update.")
    parser.add_argument("--anchor-ttl", type=float, default=20.0, help="Seconds before stale anchors are dropped.")
    parser.add_argument("--view-radius", type=float, default=3.2, help="Half-width of the follow camera volume in meters.")
    parser.add_argument("--view-ahead", type=float, default=1.4, help="Meters to bias the desktop camera ahead of the phone.")
    parser.add_argument("--chase-distance", type=float, default=2.8, help="Meters behind the phone for chase mode.")
    parser.add_argument("--chase-height", type=float, default=1.1, help="Meters above the phone for chase mode.")
    parser.add_argument("--refresh-ms", type=int, default=60, help="Desktop render refresh interval in milliseconds.")
    parser.add_argument("--point-size", type=float, default=5.0, help="Rendered point size.")
    parser.add_argument("--headless", action="store_true", help="Use a non-interactive backend.")
    parser.add_argument("--snapshot", type=Path, help="Save a PNG snapshot instead of opening an interactive window.")
    parser.add_argument("--snapshot-frames", type=int, default=10, help="Frames to render before saving a snapshot.")
    return parser


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-6:
        return vector.copy()
    return vector / norm


def plot_space_from_arkit(points_xyz: np.ndarray) -> np.ndarray:
    """Map ARKit's (x, y, z) to plot space with Z-up."""
    return np.column_stack((points_xyz[:, 0], points_xyz[:, 2], points_xyz[:, 1]))


def plot_vector_from_arkit(vector_xyz: np.ndarray) -> np.ndarray:
    return np.array([vector_xyz[0], vector_xyz[2], vector_xyz[1]], dtype=np.float32)


def class_priority(class_id: int) -> int:
    if class_id == 7:
        return 3
    if class_id in {0, 1, 4, 5, 6}:
        return 2
    if class_id == 2:
        return 1
    return 0


def color_for_class(class_id: int) -> np.ndarray:
    if class_id == 7:
        return np.array([1.00, 0.67, 0.15, 0.95], dtype=np.float32)
    if class_id == 2:
        return np.array([0.16, 0.85, 0.32, 0.72], dtype=np.float32)
    if class_id in {0, 1, 4, 5, 6}:
        return np.array([0.95, 0.28, 0.24, 0.84], dtype=np.float32)
    return np.array([0.70, 0.70, 0.78, 0.55], dtype=np.float32)


def looks_like_uuid_ascii(block: bytes) -> bool:
    if len(block) != UUID_SIZE:
        return False
    hyphen_positions = {8, 13, 18, 23}
    for index, value in enumerate(block):
        if index in hyphen_positions:
            if value != ord("-"):
                return False
            continue
        if not (
            ord("0") <= value <= ord("9")
            or ord("a") <= value <= ord("f")
            or ord("A") <= value <= ord("F")
        ):
            return False
    return True


@dataclass
class MeshMessage:
    anchor_id: str
    floats: np.ndarray


@dataclass
class CameraMessage:
    matrix: np.ndarray


@dataclass
class AnchorCloud:
    points: np.ndarray
    colors: np.ndarray
    updated_at: float


@dataclass
class ViewerSnapshot:
    points: np.ndarray
    colors: np.ndarray
    camera_position: np.ndarray
    camera_forward: np.ndarray
    camera_up: np.ndarray
    anchors: int
    packets: int
    status: str


class HavenStreamParser:
    """Incrementally decodes the iPhone stream over a TCP byte stream."""

    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, chunk: bytes) -> Iterator[MeshMessage | CameraMessage]:
        self.buffer.extend(chunk)

        while True:
            message = self._extract_one()
            if message is None:
                return
            yield message

    def _extract_one(self) -> MeshMessage | CameraMessage | None:
        if len(self.buffer) < HEADER_SIZE + COUNT_SIZE:
            return None

        if self.buffer.startswith(MESH_HEADER):
            return self._extract_mesh_packet()

        if self.buffer.startswith(CAMERA_HEADER):
            return self._extract_camera_packet(CAMERA_HEADER)

        if self.buffer.startswith(LEGACY_POSE_HEADER):
            return self._extract_camera_packet(LEGACY_POSE_HEADER)

        if looks_like_uuid_ascii(bytes(self.buffer[:UUID_SIZE])):
            return self._extract_legacy_mesh_packet()

        del self.buffer[0]
        return self._extract_one() if len(self.buffer) >= HEADER_SIZE + COUNT_SIZE else None

    def _extract_mesh_packet(self) -> MeshMessage | None:
        minimum_size = HEADER_SIZE + UUID_SIZE + COUNT_SIZE
        if len(self.buffer) < minimum_size:
            return None

        count = int.from_bytes(
            self.buffer[HEADER_SIZE + UUID_SIZE : minimum_size],
            byteorder="little",
            signed=True,
        )
        if not self._is_valid_mesh_count(count):
            del self.buffer[0]
            return self._extract_one() if len(self.buffer) >= HEADER_SIZE + COUNT_SIZE else None

        payload_size = count * 4
        total_size = minimum_size + payload_size
        if len(self.buffer) < total_size:
            return None

        anchor_id = bytes(self.buffer[HEADER_SIZE : HEADER_SIZE + UUID_SIZE]).decode("utf-8", errors="replace")
        payload = bytes(self.buffer[minimum_size:total_size])
        del self.buffer[:total_size]
        floats = np.frombuffer(payload, dtype="<f4").copy()
        return MeshMessage(anchor_id=anchor_id, floats=floats)

    def _extract_camera_packet(self, header: bytes) -> CameraMessage | None:
        count = int.from_bytes(
            self.buffer[HEADER_SIZE : HEADER_SIZE + COUNT_SIZE],
            byteorder="little",
            signed=True,
        )
        if count not in {3, 16}:
            del self.buffer[0]
            return self._extract_one() if len(self.buffer) >= HEADER_SIZE + COUNT_SIZE else None

        total_size = HEADER_SIZE + COUNT_SIZE + count * 4
        if len(self.buffer) < total_size:
            return None

        payload = bytes(self.buffer[HEADER_SIZE + COUNT_SIZE : total_size])
        del self.buffer[:total_size]
        floats = np.frombuffer(payload, dtype="<f4").copy()

        if header == LEGACY_POSE_HEADER:
            matrix = np.eye(4, dtype=np.float32)
            matrix[:3, 3] = floats[:3]
            return CameraMessage(matrix=matrix)

        return CameraMessage(matrix=floats.reshape((4, 4)))

    def _extract_legacy_mesh_packet(self) -> MeshMessage | None:
        count = int.from_bytes(
            self.buffer[UUID_SIZE : UUID_SIZE + COUNT_SIZE],
            byteorder="little",
            signed=True,
        )
        if not self._is_valid_mesh_count(count):
            del self.buffer[0]
            return self._extract_one() if len(self.buffer) >= HEADER_SIZE + COUNT_SIZE else None

        total_size = UUID_SIZE + COUNT_SIZE + count * 4
        if len(self.buffer) < total_size:
            return None

        anchor_id = bytes(self.buffer[:UUID_SIZE]).decode("utf-8", errors="replace")
        payload = bytes(self.buffer[UUID_SIZE + COUNT_SIZE : total_size])
        del self.buffer[:total_size]
        floats = np.frombuffer(payload, dtype="<f4").copy()
        return MeshMessage(anchor_id=anchor_id, floats=floats)

    @staticmethod
    def _is_valid_mesh_count(count: int) -> bool:
        return 0 <= count <= MAX_FLOAT_COUNT and count % 4 == 0


class LiveSceneState:
    def __init__(self, voxel_size: float, max_points_per_anchor: int, anchor_ttl: float) -> None:
        self.voxel_size = float(voxel_size)
        self.max_points_per_anchor = max(1, int(max_points_per_anchor))
        self.anchor_ttl = float(anchor_ttl)
        self.lock = threading.Lock()
        self.anchor_clouds: Dict[str, AnchorCloud] = {}
        self.camera_matrix = np.eye(4, dtype=np.float32)
        self.packet_count = 0
        self.status = "Idle"
        self.last_camera_update = 0.0

    def set_status(self, status: str) -> None:
        with self.lock:
            self.status = status

    def update_mesh(self, anchor_id: str, floats: np.ndarray) -> None:
        points, colors = self._reduce_anchor_cloud(floats)
        now = time.time()
        with self.lock:
            self.anchor_clouds[anchor_id] = AnchorCloud(points=points, colors=colors, updated_at=now)
            self.packet_count += 1

    def update_camera(self, matrix: np.ndarray) -> None:
        with self.lock:
            self.camera_matrix = matrix.astype(np.float32, copy=True)
            self.packet_count += 1
            self.last_camera_update = time.time()

    def snapshot(self, max_points: int) -> ViewerSnapshot:
        with self.lock:
            now = time.time()
            stale = [anchor_id for anchor_id, cloud in self.anchor_clouds.items() if now - cloud.updated_at > self.anchor_ttl]
            for anchor_id in stale:
                del self.anchor_clouds[anchor_id]

            sorted_clouds = sorted(self.anchor_clouds.values(), key=lambda cloud: cloud.updated_at, reverse=True)
            if sorted_clouds:
                points = np.concatenate([cloud.points for cloud in sorted_clouds if cloud.points.size], axis=0)
                colors = np.concatenate([cloud.colors for cloud in sorted_clouds if cloud.colors.size], axis=0)
            else:
                points = np.empty((0, 3), dtype=np.float32)
                colors = np.empty((0, 4), dtype=np.float32)

            if points.shape[0] > max_points:
                stride = max(1, math.ceil(points.shape[0] / max_points))
                points = points[::stride]
                colors = colors[::stride]

            matrix = self.camera_matrix.copy()
            status = self.status
            packets = self.packet_count
            anchors = len(self.anchor_clouds)

        position_arkit = matrix[:3, 3]
        rotation = matrix[:3, :3]
        forward_arkit = -rotation[:, 2]
        up_arkit = rotation[:, 1]

        camera_position = plot_vector_from_arkit(position_arkit)
        camera_forward = normalize(plot_vector_from_arkit(forward_arkit))
        camera_up = normalize(plot_vector_from_arkit(up_arkit))

        if np.linalg.norm(camera_forward) < 1e-6:
            camera_forward = np.array([0.0, 1.0, 0.0], dtype=np.float32)
        if np.linalg.norm(camera_up) < 1e-6:
            camera_up = np.array([0.0, 0.0, 1.0], dtype=np.float32)

        return ViewerSnapshot(
            points=points,
            colors=colors,
            camera_position=camera_position,
            camera_forward=camera_forward,
            camera_up=camera_up,
            anchors=anchors,
            packets=packets,
            status=status,
        )

    def _reduce_anchor_cloud(self, floats: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if floats.size == 0:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 4), dtype=np.float32)

        raw = floats.reshape((-1, 4))
        if raw.shape[0] > 25_000:
            stride = max(1, math.ceil(raw.shape[0] / 25_000))
            raw = raw[::stride]

        voxel_map: Dict[tuple[int, int, int], tuple[float, float, float, int]] = {}
        for sample in raw:
            class_id = int(sample[3])
            plot_point = plot_vector_from_arkit(sample[:3])
            quantized = (
                int(round(float(plot_point[0]) / self.voxel_size)),
                int(round(float(plot_point[1]) / self.voxel_size)),
                int(round(float(plot_point[2]) / self.voxel_size)),
            )

            previous = voxel_map.get(quantized)
            if previous is not None and class_priority(class_id) < class_priority(previous[3]):
                continue

            voxel_map[quantized] = (
                quantized[0] * self.voxel_size,
                quantized[1] * self.voxel_size,
                quantized[2] * self.voxel_size,
                class_id,
            )

        if not voxel_map:
            return np.empty((0, 3), dtype=np.float32), np.empty((0, 4), dtype=np.float32)

        reduced = np.asarray(list(voxel_map.values()), dtype=np.float32)
        if reduced.shape[0] > self.max_points_per_anchor:
            stride = max(1, math.ceil(reduced.shape[0] / self.max_points_per_anchor))
            reduced = reduced[::stride]

        points = reduced[:, :3]
        colors = np.stack([color_for_class(int(class_id)) for class_id in reduced[:, 3]], axis=0)
        return points, colors


class PhoneStreamWorker(threading.Thread):
    def __init__(self, host: str, port: int, state: LiveSceneState, stop_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self.host = host
        self.port = port
        self.state = state
        self.stop_event = stop_event

    def run(self) -> None:
        parser = HavenStreamParser()

        while not self.stop_event.is_set():
            try:
                self.state.set_status(f"Connecting to {self.host}:{self.port}...")
                with socket.create_connection((self.host, self.port), timeout=5.0) as sock:
                    sock.settimeout(1.0)
                    self.state.set_status(f"Streaming from {self.host}:{self.port}")

                    while not self.stop_event.is_set():
                        try:
                            chunk = sock.recv(131_072)
                        except socket.timeout:
                            continue

                        if not chunk:
                            break

                        for message in parser.feed(chunk):
                            if isinstance(message, MeshMessage):
                                self.state.update_mesh(message.anchor_id, message.floats)
                            else:
                                self.state.update_camera(message.matrix)

                if not self.stop_event.is_set():
                    self.state.set_status(f"Viewer disconnected from {self.host}:{self.port}; retrying...")
            except OSError as exc:
                self.state.set_status(f"Connect failed: {exc}")

            if not self.stop_event.is_set():
                time.sleep(1.0)


class DemoWorker(threading.Thread):
    def __init__(self, state: LiveSceneState, stop_event: threading.Event) -> None:
        super().__init__(daemon=True)
        self.state = state
        self.stop_event = stop_event

    def run(self) -> None:
        self.state.set_status("Demo mode")
        self.state.update_mesh("demo-room", self._build_demo_room())

        while not self.stop_event.is_set():
            t = time.time()
            position = np.array([
                math.sin(t * 0.35) * 1.4,
                1.55 + math.sin(t * 0.5) * 0.08,
                -2.4 + math.cos(t * 0.35) * 1.4,
            ], dtype=np.float32)
            target = np.array([0.0, 1.35, -2.4], dtype=np.float32)
            self.state.update_camera(build_camera_matrix(position, target))
            time.sleep(1.0 / 30.0)

    @staticmethod
    def _build_demo_room() -> np.ndarray:
        points = []

        for x in np.linspace(-2.5, 2.5, 70):
            for z in np.linspace(-5.0, 0.2, 80):
                points.append((x, 0.0, z, 2))

        for y in np.linspace(0.0, 2.7, 30):
            for x in np.linspace(-2.5, 2.5, 55):
                points.append((x, y, -5.0, 1))
            for z in np.linspace(-5.0, 0.2, 55):
                points.append((-2.5, y, z, 1))
                points.append((2.5, y, z, 1))

        for y in np.linspace(0.0, 2.2, 22):
            for z in np.linspace(-0.15, 0.15, 15):
                points.append((0.0, y, z - 2.4, 7))

        return np.asarray(points, dtype=np.float32).reshape((-1,))


def build_camera_matrix(position: np.ndarray, target: np.ndarray) -> np.ndarray:
    forward = normalize(target - position)
    up_guess = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    right = normalize(np.cross(forward, up_guess))
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    up = normalize(np.cross(right, forward))

    # ARKit camera transform columns are right, up, backward, translation.
    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, 0] = right
    matrix[:3, 1] = up
    matrix[:3, 2] = -forward
    matrix[:3, 3] = position
    return matrix


class MatplotlibSceneViewer:
    def __init__(self, state: LiveSceneState, args: argparse.Namespace) -> None:
        self.state = state
        self.args = args

        mpl_config_dir = Path("/private/tmp/haven-mplconfig")
        mpl_config_dir.mkdir(parents=True, exist_ok=True)
        os.environ.setdefault("MPLCONFIGDIR", str(mpl_config_dir))

        import matplotlib

        if args.headless or args.snapshot:
            matplotlib.use("Agg")

        import matplotlib.pyplot as plt
        from matplotlib.animation import FuncAnimation

        self.plt = plt
        self.FuncAnimation = FuncAnimation
        self.fig = plt.figure(figsize=(11, 8))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_facecolor("#0b0d13")
        self.fig.patch.set_facecolor("#0b0d13")
        self.ax.set_xlabel("X (m)")
        self.ax.set_ylabel("Z (m)")
        self.ax.set_zlabel("Y height (m)")
        self.ax.set_title("HAVEN Phone Stream Viewer", color="white", pad=16)
        self._style_axes()

        self.cloud = self.ax.scatter([], [], [], s=args.point_size, c=[], depthshade=False)
        self.camera_dot = self.ax.scatter([0.0], [0.0], [0.0], s=70, c=["#4db7ff"], depthshade=False)
        self.forward_line, = self.ax.plot([], [], [], color="#4db7ff", linewidth=2.5)
        self.up_line, = self.ax.plot([], [], [], color="#d7f6ff", linewidth=1.5)
        self.status_text = self.fig.text(0.02, 0.02, "", color="white", family="monospace", fontsize=9)

        self.animation = None
        if not args.snapshot:
            self.animation = self.FuncAnimation(
                self.fig,
                self._draw_frame,
                interval=max(15, int(args.refresh_ms)),
                blit=False,
                cache_frame_data=False,
            )

    def show(self) -> None:
        self.plt.show()

    def save_snapshot(self, path: Path, frames: int) -> None:
        for frame_index in range(max(1, frames)):
            self._draw_frame(frame_index)
            time.sleep(max(0.001, self.args.refresh_ms / 1000.0))

        path.parent.mkdir(parents=True, exist_ok=True)
        self.fig.savefig(path, dpi=180, facecolor=self.fig.get_facecolor(), bbox_inches="tight")

    def _draw_frame(self, _frame_index: int):
        snapshot = self.state.snapshot(max_points=self.args.max_points)

        if snapshot.points.size:
            self.cloud._offsets3d = (
                snapshot.points[:, 0],
                snapshot.points[:, 1],
                snapshot.points[:, 2],
            )
            self.cloud.set_color(snapshot.colors)
            self.cloud.set_sizes(np.full(snapshot.points.shape[0], self.args.point_size, dtype=np.float32))
        else:
            self.cloud._offsets3d = ([], [], [])
            self.cloud.set_color([])

        camera_tip = snapshot.camera_position + snapshot.camera_forward * 0.8
        camera_up = snapshot.camera_position + snapshot.camera_up * 0.35

        self.camera_dot._offsets3d = (
            [snapshot.camera_position[0]],
            [snapshot.camera_position[1]],
            [snapshot.camera_position[2]],
        )
        self.forward_line.set_data_3d(
            [snapshot.camera_position[0], camera_tip[0]],
            [snapshot.camera_position[1], camera_tip[1]],
            [snapshot.camera_position[2], camera_tip[2]],
        )
        self.up_line.set_data_3d(
            [snapshot.camera_position[0], camera_up[0]],
            [snapshot.camera_position[1], camera_up[1]],
            [snapshot.camera_position[2], camera_up[2]],
        )

        self._apply_follow_view(snapshot)
        self.status_text.set_text(
            f"{snapshot.status}\nanchors={snapshot.anchors}  packets={snapshot.packets}  points={snapshot.points.shape[0]}"
        )
        return self.cloud, self.camera_dot, self.forward_line, self.up_line, self.status_text

    def _apply_follow_view(self, snapshot: ViewerSnapshot) -> None:
        center = snapshot.camera_position + snapshot.camera_forward * float(self.args.view_ahead)
        radius = float(self.args.view_radius)

        self.ax.set_xlim(center[0] - radius, center[0] + radius)
        self.ax.set_ylim(center[1] - radius, center[1] + radius)
        self.ax.set_zlim(center[2] - radius * 0.55, center[2] + radius * 0.75)

        azim = math.degrees(math.atan2(snapshot.camera_forward[1], snapshot.camera_forward[0])) - 90.0
        elev = math.degrees(
            math.atan2(
                snapshot.camera_forward[2],
                max(1e-5, np.linalg.norm(snapshot.camera_forward[:2])),
            )
        )

        try:
            self.ax.view_init(elev=elev, azim=azim, roll=0.0)
        except TypeError:
            self.ax.view_init(elev=elev, azim=azim)

    def _style_axes(self) -> None:
        self.ax.tick_params(colors="white", labelsize=8)
        self.ax.xaxis.label.set_color("white")
        self.ax.yaxis.label.set_color("white")
        self.ax.zaxis.label.set_color("white")
        self.ax.grid(color="#334155", alpha=0.25)

        for axis in (self.ax.xaxis, self.ax.yaxis, self.ax.zaxis):
            axis.pane.set_facecolor((0.06, 0.08, 0.11, 0.18))
            axis.pane.set_edgecolor((0.40, 0.47, 0.56, 0.28))


class Open3DSceneViewer:
    def __init__(self, state: LiveSceneState, args: argparse.Namespace) -> None:
        import open3d as o3d

        self.o3d = o3d
        self.state = state
        self.args = args
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="HAVEN Phone Stream Viewer", width=1440, height=900)

        render = self.vis.get_render_option()
        render.background_color = np.array([0.04, 0.05, 0.08], dtype=np.float64)
        render.point_size = float(max(1.0, args.point_size))
        render.show_coordinate_frame = True
        self.last_log_at = 0.0

        self.cloud = o3d.geometry.PointCloud()
        self.camera_lines = o3d.geometry.LineSet()
        self.camera_lines.lines = o3d.utility.Vector2iVector(
            np.array([[0, 1], [0, 2], [0, 3]], dtype=np.int32)
        )
        self.camera_lines.colors = o3d.utility.Vector3dVector(
            np.array(
                [
                    [0.30, 0.72, 1.00],
                    [0.86, 0.30, 0.95],
                    [0.85, 0.96, 1.00],
                ],
                dtype=np.float64,
            )
        )
        self.phone_dot = o3d.geometry.PointCloud()
        self.world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)

        self.vis.add_geometry(self.cloud)
        self.vis.add_geometry(self.camera_lines)
        self.vis.add_geometry(self.phone_dot)
        self.vis.add_geometry(self.world_frame)

    def show(self) -> None:
        try:
            while True:
                self._draw_frame()
                if not self.vis.poll_events():
                    break
                self.vis.update_renderer()
                time.sleep(max(0.01, self.args.refresh_ms / 1000.0))
        finally:
            self.vis.destroy_window()

    def _draw_frame(self) -> None:
        snapshot = self.state.snapshot(max_points=self.args.max_points)

        if snapshot.points.size:
            self.cloud.points = self.o3d.utility.Vector3dVector(snapshot.points.astype(np.float64))
            self.cloud.colors = self.o3d.utility.Vector3dVector(snapshot.colors[:, :3].astype(np.float64))
        else:
            self.cloud.points = self.o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
            self.cloud.colors = self.o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
        self.vis.update_geometry(self.cloud)

        right = normalize(np.cross(snapshot.camera_forward, snapshot.camera_up))
        if np.linalg.norm(right) < 1e-6:
            right = np.array([1.0, 0.0, 0.0], dtype=np.float32)

        origin = snapshot.camera_position
        line_points = np.vstack(
            [
                origin,
                origin + snapshot.camera_forward * 0.9,
                origin + right * 0.35,
                origin + snapshot.camera_up * 0.35,
            ]
        ).astype(np.float64)
        self.camera_lines.points = self.o3d.utility.Vector3dVector(line_points)
        self.vis.update_geometry(self.camera_lines)

        self.phone_dot.points = self.o3d.utility.Vector3dVector(origin.reshape(1, 3).astype(np.float64))
        self.phone_dot.colors = self.o3d.utility.Vector3dVector(
            np.array([[0.30, 0.72, 1.00]], dtype=np.float64)
        )
        self.vis.update_geometry(self.phone_dot)

        view = self.vis.get_view_control()
        lookat = origin + snapshot.camera_forward * float(self.args.view_ahead)

        if self.args.follow_mode == "first_person":
            front = -snapshot.camera_forward
            up = snapshot.camera_up
            zoom = max(0.20, min(0.75, 1.0 / max(1.5, float(self.args.view_radius))))
        else:
            chase_eye = (
                origin
                - snapshot.camera_forward * float(self.args.chase_distance)
                + snapshot.camera_up * float(self.args.chase_height)
            )
            front = normalize(chase_eye - lookat)
            up = np.array([0.0, 0.0, 1.0], dtype=np.float32)
            zoom = max(0.28, min(0.72, 1.35 / max(1.8, float(self.args.view_radius))))

        view.set_lookat(lookat.astype(np.float64))
        view.set_front(front.astype(np.float64))
        view.set_up(up.astype(np.float64))
        view.set_zoom(zoom)

        now = time.time()
        if now - self.last_log_at >= 2.0:
            self.last_log_at = now
            print(
                f"[viewer] status='{snapshot.status}' anchors={snapshot.anchors} "
                f"points={snapshot.points.shape[0]} packets={snapshot.packets} "
                f"camera=({origin[0]:.2f}, {origin[1]:.2f}, {origin[2]:.2f})"
            )


def build_viewer(state: LiveSceneState, args: argparse.Namespace):
    if args.snapshot or args.headless:
        return MatplotlibSceneViewer(state=state, args=args)

    if args.backend in {"auto", "open3d"}:
        try:
            return Open3DSceneViewer(state=state, args=args)
        except ImportError:
            if args.backend == "open3d":
                raise

    return MatplotlibSceneViewer(state=state, args=args)


def main() -> int:
    args = build_arg_parser().parse_args()

    if not args.demo and not args.host:
        raise SystemExit("--host is required unless --demo is used.")

    state = LiveSceneState(
        voxel_size=args.voxel_size,
        max_points_per_anchor=args.max_points_per_anchor,
        anchor_ttl=args.anchor_ttl,
    )
    stop_event = threading.Event()

    worker: threading.Thread
    if args.demo:
        worker = DemoWorker(state=state, stop_event=stop_event)
    else:
        worker = PhoneStreamWorker(host=args.host, port=args.port, state=state, stop_event=stop_event)
    worker.start()

    viewer = build_viewer(state=state, args=args)

    try:
        if args.snapshot:
            viewer.save_snapshot(args.snapshot, frames=args.snapshot_frames)
        else:
            viewer.show()
    finally:
        stop_event.set()
        worker.join(timeout=1.5)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
