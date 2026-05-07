#!/usr/bin/env python3.11
"""Reference-style live surface viewer for HAVEN's iPhone AR stream.

This version follows the working reconstruction approach from the user's
reference script: keep each streamed AR mesh anchor as its own TriangleMesh and
render the exact face triplets as sent by the phone. On top of that, it keeps a
live phone pose, camera-follow behavior, a 3D trajectory trail, and a 2D map.
"""

from __future__ import annotations

import argparse
import select
import socket
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Deque, Dict, Iterator

import cv2
import numpy as np
import open3d as o3d

MESH_HEADER = b"MESH--------------------------------"
CAMERA_HEADER = b"CAMERA------------------------------"
LEGACY_POSE_HEADER = b"POSE--------------------------------"
HEADER_SIZE = 36
UUID_SIZE = 36
COUNT_SIZE = 4
MAX_FLOAT_COUNT = 4_000_000

COLOR_MAP: dict[int, list[float]] = {
    0: [0.50, 0.50, 0.50],
    1: [0.80, 0.20, 0.20],
    2: [0.20, 0.80, 0.20],
    3: [0.20, 0.20, 0.80],
    4: [0.80, 0.80, 0.20],
    5: [0.80, 0.20, 0.80],
    6: [0.20, 0.80, 0.80],
    7: [0.90, 0.50, 0.10],
}


@dataclass
class MeshPacket:
    anchor_id: str
    floats: np.ndarray


@dataclass
class CameraPacket:
    matrix: np.ndarray


@dataclass
class CameraState:
    matrix: np.ndarray = field(default_factory=lambda: np.eye(4, dtype=np.float32))
    valid: bool = False
    packet_count: int = 0


class StreamPacketParser:
    def __init__(self) -> None:
        self.buffer = bytearray()

    def feed(self, chunk: bytes) -> Iterator[MeshPacket | CameraPacket]:
        self.buffer.extend(chunk)
        while True:
            packet = self._extract_one()
            if packet is None:
                return
            yield packet

    def _extract_one(self) -> MeshPacket | CameraPacket | None:
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

    def _extract_mesh_packet(self) -> MeshPacket | None:
        minimum_size = HEADER_SIZE + UUID_SIZE + COUNT_SIZE
        if len(self.buffer) < minimum_size:
            return None

        float_count = int.from_bytes(
            self.buffer[HEADER_SIZE + UUID_SIZE : minimum_size],
            byteorder="little",
            signed=True,
        )
        if not self._is_valid_mesh_count(float_count):
            del self.buffer[0]
            return self._extract_one() if len(self.buffer) >= HEADER_SIZE + COUNT_SIZE else None

        payload_size = float_count * 4
        total_size = minimum_size + payload_size
        if len(self.buffer) < total_size:
            return None

        anchor_id = bytes(self.buffer[HEADER_SIZE : HEADER_SIZE + UUID_SIZE]).decode("utf-8", errors="replace")
        payload = bytes(self.buffer[minimum_size:total_size])
        del self.buffer[:total_size]
        floats = np.frombuffer(payload, dtype="<f4").copy()
        return MeshPacket(anchor_id=anchor_id, floats=floats)

    def _extract_camera_packet(self, header: bytes) -> CameraPacket | None:
        float_count = int.from_bytes(
            self.buffer[HEADER_SIZE : HEADER_SIZE + COUNT_SIZE],
            byteorder="little",
            signed=True,
        )
        if float_count not in {3, 16}:
            del self.buffer[0]
            return self._extract_one() if len(self.buffer) >= HEADER_SIZE + COUNT_SIZE else None

        total_size = HEADER_SIZE + COUNT_SIZE + float_count * 4
        if len(self.buffer) < total_size:
            return None

        payload = bytes(self.buffer[HEADER_SIZE + COUNT_SIZE : total_size])
        del self.buffer[:total_size]
        floats = np.frombuffer(payload, dtype="<f4").copy()

        if header == LEGACY_POSE_HEADER:
            matrix = np.eye(4, dtype=np.float32)
            matrix[:3, 3] = floats[:3]
            return CameraPacket(matrix=matrix)

        return CameraPacket(matrix=floats.reshape((4, 4)))

    def _extract_legacy_mesh_packet(self) -> MeshPacket | None:
        total_header = UUID_SIZE + COUNT_SIZE
        float_count = int.from_bytes(
            self.buffer[UUID_SIZE:total_header],
            byteorder="little",
            signed=True,
        )
        if not self._is_valid_mesh_count(float_count):
            del self.buffer[0]
            return self._extract_one() if len(self.buffer) >= HEADER_SIZE + COUNT_SIZE else None

        total_size = total_header + float_count * 4
        if len(self.buffer) < total_size:
            return None

        anchor_id = bytes(self.buffer[:UUID_SIZE]).decode("utf-8", errors="replace")
        payload = bytes(self.buffer[total_header:total_size])
        del self.buffer[:total_size]
        floats = np.frombuffer(payload, dtype="<f4").copy()
        return MeshPacket(anchor_id=anchor_id, floats=floats)

    @staticmethod
    def _is_valid_mesh_count(float_count: int) -> bool:
        return 0 <= float_count <= MAX_FLOAT_COUNT and float_count % 4 == 0


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Render HAVEN's live AR mesh surfaces on a desktop.")
    parser.add_argument("--host", help="iPhone IP address shown in the HAVEN app.")
    parser.add_argument("--port", type=int, default=8080, help="TCP port exposed by the iPhone app.")
    parser.add_argument("--demo", action="store_true", help="Run without a phone using a synthetic room.")
    parser.add_argument(
        "--follow-mode",
        choices=("chase", "first_person"),
        default="chase",
        help="How the desktop camera follows the phone.",
    )
    parser.add_argument("--view-ahead", type=float, default=1.1, help="Meters ahead of the phone to look at.")
    parser.add_argument("--chase-distance", type=float, default=2.6, help="Meters behind the phone for chase mode.")
    parser.add_argument("--chase-height", type=float, default=1.0, help="Meters above the phone for chase mode.")
    parser.add_argument("--first-person-zoom", type=float, default=0.56, help="Open3D zoom in first-person mode.")
    parser.add_argument("--chase-zoom", type=float, default=0.40, help="Open3D zoom in chase mode.")
    parser.add_argument("--show-map", action="store_true", help="Show a live 2D top-down map in a separate OpenCV window.")
    parser.add_argument("--map-size", type=int, default=700, help="2D map image size in pixels.")
    parser.add_argument("--wireframe", action="store_true", help="Overlay wireframe edges on top of filled mesh surfaces.")
    parser.add_argument("--path-length", type=int, default=400, help="How many recent phone poses to keep in the 3D path trail.")
    parser.add_argument("--log-interval", type=float, default=2.0, help="Seconds between console stats updates.")
    parser.add_argument("--socket-timeout", type=float, default=0.01, help="Socket poll timeout in seconds.")
    return parser


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


def normalize(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm < 1e-6:
        return vector.copy()
    return vector / norm


def colors_for_classes(classes: np.ndarray) -> np.ndarray:
    colors = np.zeros((classes.shape[0], 3), dtype=np.float64)
    for class_id, color in COLOR_MAP.items():
        colors[classes == class_id] = color
    return colors


def pose_vectors(matrix: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    position = matrix[:3, 3].astype(np.float64)
    rotation = matrix[:3, :3].astype(np.float64)
    forward = normalize(-rotation[:, 2])
    up = normalize(rotation[:, 1])
    right = normalize(rotation[:, 0])
    return position, forward, up, right


def draw_2d_map(
    meshes: Dict[str, o3d.geometry.TriangleMesh],
    mesh_classes: Dict[str, np.ndarray],
    camera_state: CameraState,
    path_points: Deque[np.ndarray],
    img_size: int,
) -> np.ndarray:
    img = np.ones((img_size, img_size, 3), dtype=np.uint8) * 30

    all_vertices, all_classes = [], []
    for anchor_id, mesh in meshes.items():
        if anchor_id not in mesh_classes:
            continue
        verts = np.asarray(mesh.vertices)
        if verts.size == 0:
            continue
        all_vertices.append(verts)
        all_classes.append(mesh_classes[anchor_id])

    if not all_vertices:
        return img

    vertices = np.vstack(all_vertices)
    classes = np.concatenate(all_classes)
    x_coords, z_coords = vertices[:, 0], vertices[:, 2]

    x_min, x_max = np.min(x_coords), np.max(x_coords)
    z_min, z_max = np.min(z_coords), np.max(z_coords)
    span_x, span_z = max(x_max - x_min, 4.0), max(z_max - z_min, 4.0)
    pad_x, pad_z = span_x * 0.10, span_z * 0.10
    x_min_pad, x_max_pad = x_min - pad_x, x_max + pad_x
    z_min_pad, z_max_pad = z_min - pad_z, z_max + pad_z
    scale = img_size / max(x_max_pad - x_min_pad, z_max_pad - z_min_pad)

    px = ((x_coords - x_min_pad) * scale).astype(np.int32)
    py = ((z_coords - z_min_pad) * scale).astype(np.int32)
    bounds = (px >= 0) & (px < img_size) & (py >= 0) & (py < img_size)
    px, py, classes = px[bounds], py[bounds], classes[bounds]

    floor_mask = classes == 2
    img[py[floor_mask], px[floor_mask]] = [100, 200, 100]

    door_mask = classes == 7
    img[py[door_mask], px[door_mask]] = [0, 150, 255]

    obstacle_mask = np.isin(classes, [0, 1, 4, 5, 6])
    img[py[obstacle_mask], px[obstacle_mask]] = [50, 50, 200]

    if path_points:
        path_px = []
        for point in path_points:
            x, z = float(point[0]), float(point[2])
            px_path = int((x - x_min_pad) * scale)
            py_path = int((z - z_min_pad) * scale)
            if 0 <= px_path < img_size and 0 <= py_path < img_size:
                path_px.append((px_path, py_path))
        if len(path_px) > 1:
            cv2.polylines(img, [np.array(path_px, dtype=np.int32)], False, (180, 180, 255), 2, cv2.LINE_AA)

    if camera_state.valid:
        position, forward, _, _ = pose_vectors(camera_state.matrix)
        px_cam = int((position[0] - x_min_pad) * scale)
        py_cam = int((position[2] - z_min_pad) * scale)
        if 0 <= px_cam < img_size and 0 <= py_cam < img_size:
            cv2.circle(img, (px_cam, py_cam), 8, (255, 100, 100), -1)
            cv2.circle(img, (px_cam, py_cam), 9, (255, 255, 255), 1)
            arrow_tip = (
                int((position[0] + forward[0] * 0.45 - x_min_pad) * scale),
                int((position[2] + forward[2] * 0.45 - z_min_pad) * scale),
            )
            cv2.arrowedLine(img, (px_cam, py_cam), arrow_tip, (255, 255, 255), 2, cv2.LINE_AA, tipLength=0.35)

    return img


def build_demo_mesh_payload() -> np.ndarray:
    triangles: list[tuple[float, float, float, int]] = []

    def add_quad(a, b, c, d, class_id: int) -> None:
        triangles.extend(
            [
                (*a, class_id), (*b, class_id), (*c, class_id),
                (*a, class_id), (*c, class_id), (*d, class_id),
            ]
        )

    floor_x = np.linspace(-2.5, 2.5, 28)
    floor_z = np.linspace(-5.0, 0.2, 34)
    for ix in range(len(floor_x) - 1):
        for iz in range(len(floor_z) - 1):
            x0, x1 = floor_x[ix], floor_x[ix + 1]
            z0, z1 = floor_z[iz], floor_z[iz + 1]
            add_quad((x0, 0.0, z0), (x1, 0.0, z0), (x1, 0.0, z1), (x0, 0.0, z1), 2)

    wall_y = np.linspace(0.0, 2.7, 18)
    wall_x = np.linspace(-2.5, 2.5, 22)
    wall_z = np.linspace(-5.0, 0.2, 24)

    for iy in range(len(wall_y) - 1):
        for ix in range(len(wall_x) - 1):
            y0, y1 = wall_y[iy], wall_y[iy + 1]
            x0, x1 = wall_x[ix], wall_x[ix + 1]
            add_quad((x0, y0, -5.0), (x1, y0, -5.0), (x1, y1, -5.0), (x0, y1, -5.0), 1)

    for iy in range(len(wall_y) - 1):
        for iz in range(len(wall_z) - 1):
            y0, y1 = wall_y[iy], wall_y[iy + 1]
            z0, z1 = wall_z[iz], wall_z[iz + 1]
            add_quad((-2.5, y0, z0), (-2.5, y0, z1), (-2.5, y1, z1), (-2.5, y1, z0), 1)
            add_quad((2.5, y0, z0), (2.5, y1, z0), (2.5, y1, z1), (2.5, y0, z1), 1)

    door_y = np.linspace(0.0, 2.2, 14)
    door_z = np.linspace(-2.55, -2.25, 6)
    for iy in range(len(door_y) - 1):
        for iz in range(len(door_z) - 1):
            y0, y1 = door_y[iy], door_y[iy + 1]
            z0, z1 = door_z[iz], door_z[iz + 1]
            add_quad((-0.02, y0, z0), (0.02, y0, z0), (0.02, y1, z1), (-0.02, y1, z1), 7)

    return np.asarray(triangles, dtype=np.float32).reshape((-1,))


def build_demo_camera_matrix(now: float) -> np.ndarray:
    position = np.array(
        [
            np.sin(now * 0.35) * 1.4,
            1.55 + np.sin(now * 0.5) * 0.08,
            -2.4 + np.cos(now * 0.35) * 1.4,
        ],
        dtype=np.float32,
    )
    target = np.array([0.0, 1.35, -2.4], dtype=np.float32)

    forward = normalize(target - position)
    up_guess = np.array([0.0, 1.0, 0.0], dtype=np.float32)
    right = normalize(np.cross(forward, up_guess))
    if np.linalg.norm(right) < 1e-6:
        right = np.array([1.0, 0.0, 0.0], dtype=np.float32)
    up = normalize(np.cross(right, forward))

    matrix = np.eye(4, dtype=np.float32)
    matrix[:3, 0] = right
    matrix[:3, 1] = up
    matrix[:3, 2] = -forward
    matrix[:3, 3] = position
    return matrix


def apply_mesh_packet(
    vis: o3d.visualization.Visualizer,
    meshes: Dict[str, o3d.geometry.TriangleMesh],
    mesh_classes: Dict[str, np.ndarray],
    packet: MeshPacket,
) -> None:
    if packet.floats.size == 0:
        return

    mesh_data = packet.floats.reshape((-1, 4))
    usable_vertices = mesh_data.shape[0] - (mesh_data.shape[0] % 3)
    mesh_data = mesh_data[:usable_vertices]
    if mesh_data.size == 0:
        return

    vertices = mesh_data[:, :3].astype(np.float64, copy=False)
    classes = mesh_data[:, 3].astype(np.int32, copy=False)
    colors = colors_for_classes(classes)
    triangles = np.arange(vertices.shape[0], dtype=np.int32).reshape((-1, 3))

    if packet.anchor_id not in meshes:
        mesh = o3d.geometry.TriangleMesh()
        vis.add_geometry(mesh)
        meshes[packet.anchor_id] = mesh

    target_mesh = meshes[packet.anchor_id]
    target_mesh.vertices = o3d.utility.Vector3dVector(vertices)
    target_mesh.triangles = o3d.utility.Vector3iVector(triangles)
    target_mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    target_mesh.compute_vertex_normals()
    mesh_classes[packet.anchor_id] = classes
    vis.update_geometry(target_mesh)


def update_phone_axes(
    line_set: o3d.geometry.LineSet,
    pose: CameraState,
) -> None:
    if not pose.valid:
        line_set.points = o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
        return

    position, forward, up, right = pose_vectors(pose.matrix)
    points = np.vstack(
        [
            position,
            position + forward * 0.55,
            position + right * 0.30,
            position + up * 0.30,
        ]
    ).astype(np.float64)
    line_set.points = o3d.utility.Vector3dVector(points)


def update_path_geometry(
    line_set: o3d.geometry.LineSet,
    path_points: Deque[np.ndarray],
) -> None:
    if len(path_points) < 2:
        line_set.points = o3d.utility.Vector3dVector(np.empty((0, 3), dtype=np.float64))
        line_set.lines = o3d.utility.Vector2iVector(np.empty((0, 2), dtype=np.int32))
        return

    points = np.vstack(path_points).astype(np.float64)
    lines = np.column_stack(
        [
            np.arange(points.shape[0] - 1, dtype=np.int32),
            np.arange(1, points.shape[0], dtype=np.int32),
        ]
    )
    colors = np.tile(np.array([[0.85, 0.90, 1.00]], dtype=np.float64), (lines.shape[0], 1))
    line_set.points = o3d.utility.Vector3dVector(points)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)


def append_path_point(path_points: Deque[np.ndarray], pose: CameraState) -> None:
    if not pose.valid:
        return
    position = pose.matrix[:3, 3].astype(np.float64)
    if path_points and np.linalg.norm(position - path_points[-1]) < 0.03:
        return
    path_points.append(position)


def apply_follow_view(
    vis: o3d.visualization.Visualizer,
    pose: CameraState,
    args: argparse.Namespace,
) -> None:
    if not pose.valid:
        return

    position, forward, up, _ = pose_vectors(pose.matrix)
    lookat = position + forward * float(args.view_ahead)
    control = vis.get_view_control()

    if args.follow_mode == "first_person":
        front = -forward
        zoom = float(args.first_person_zoom)
    else:
        chase_eye = position - forward * float(args.chase_distance) + up * float(args.chase_height)
        front = normalize(chase_eye - lookat)
        zoom = float(args.chase_zoom)

    control.set_lookat(lookat)
    control.set_front(front)
    control.set_up(up)
    control.set_zoom(zoom)


def connect_socket(host: str, port: int) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((host, port))
    sock.setblocking(False)
    return sock


def main() -> int:
    args = build_arg_parser().parse_args()
    if not args.demo and not args.host:
        raise SystemExit("--host is required unless --demo is used.")

    vis = o3d.visualization.Visualizer()
    vis.create_window(window_name="HAVEN Live Semantic Surfaces", width=1280, height=900)
    render = vis.get_render_option()
    render.background_color = np.asarray([0.0, 0.0, 0.0])
    render.mesh_show_back_face = True
    render.mesh_show_wireframe = bool(args.wireframe)

    meshes: Dict[str, o3d.geometry.TriangleMesh] = {}
    mesh_classes: Dict[str, np.ndarray] = {}
    parser = StreamPacketParser()
    camera_state = CameraState()
    path_points: Deque[np.ndarray] = deque(maxlen=max(2, int(args.path_length)))
    mesh_packet_count = 0
    last_log_at = 0.0
    camera_initialized = False

    phone_axes = o3d.geometry.LineSet()
    phone_axes.lines = o3d.utility.Vector2iVector(np.array([[0, 1], [0, 2], [0, 3]], dtype=np.int32))
    phone_axes.colors = o3d.utility.Vector3dVector(
        np.array(
            [
                [0.35, 0.75, 1.00],
                [1.00, 0.35, 0.75],
                [0.85, 1.00, 1.00],
            ],
            dtype=np.float64,
        )
    )
    vis.add_geometry(phone_axes)

    path_lines = o3d.geometry.LineSet()
    vis.add_geometry(path_lines)

    world_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.5)
    vis.add_geometry(world_frame)

    sock: socket.socket | None = None
    if args.demo:
        apply_mesh_packet(vis, meshes, mesh_classes, MeshPacket(anchor_id="demo-room", floats=build_demo_mesh_payload()))
        camera_initialized = True
        vis.reset_view_point(True)
    else:
        print(f"Connecting to {args.host}:{args.port}...")
        sock = connect_socket(args.host, args.port)

    try:
        while True:
            if args.demo:
                camera_state.matrix = build_demo_camera_matrix(time.time())
                camera_state.valid = True
                camera_state.packet_count += 1
                append_path_point(path_points, camera_state)
            else:
                assert sock is not None
                ready_to_read, _, _ = select.select([sock], [], [], float(args.socket_timeout))
                if ready_to_read:
                    try:
                        data = sock.recv(131_072)
                        if not data:
                            break
                        for packet in parser.feed(data):
                            if isinstance(packet, MeshPacket):
                                apply_mesh_packet(vis, meshes, mesh_classes, packet)
                                mesh_packet_count += 1
                                if not camera_initialized and meshes:
                                    vis.reset_view_point(True)
                                    camera_initialized = True
                            else:
                                camera_state.matrix = packet.matrix
                                camera_state.valid = True
                                camera_state.packet_count += 1
                                append_path_point(path_points, camera_state)
                    except BlockingIOError:
                        pass
                    except Exception as exc:
                        print(f"Parsing error: {exc}")
                        parser.buffer.clear()

            update_phone_axes(phone_axes, camera_state)
            vis.update_geometry(phone_axes)

            update_path_geometry(path_lines, path_points)
            vis.update_geometry(path_lines)

            apply_follow_view(vis, camera_state, args)

            if args.show_map:
                map_img = draw_2d_map(meshes, mesh_classes, camera_state, path_points, int(args.map_size))
                cv2.imshow("HAVEN 2D Dynamic Map", map_img)
                cv2.waitKey(1)

            now = time.time()
            if now - last_log_at >= float(args.log_interval):
                last_log_at = now
                position, forward, up, _ = pose_vectors(camera_state.matrix) if camera_state.valid else (
                    np.zeros(3), np.zeros(3), np.zeros(3), np.zeros(3)
                )
                total_triangles = sum(len(np.asarray(mesh.triangles)) for mesh in meshes.values())
                print(
                    f"[viewer] anchors={len(meshes)} mesh_packets={mesh_packet_count} "
                    f"pose_packets={camera_state.packet_count} triangles={total_triangles} "
                    f"pos=({position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}) "
                    f"forward=({forward[0]:.2f}, {forward[1]:.2f}, {forward[2]:.2f}) "
                    f"up=({up[0]:.2f}, {up[1]:.2f}, {up[2]:.2f})"
                )

            if not vis.poll_events():
                break
            vis.update_renderer()

    except KeyboardInterrupt:
        pass
    finally:
        if sock is not None:
            sock.close()
        vis.destroy_window()
        cv2.destroyAllWindows()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
