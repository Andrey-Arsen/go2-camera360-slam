#!/usr/bin/env python3
"""Карта препятствий для Nav2 (nav_msgs/OccupancyGrid /map) из точек карты SLAM.

Слушает /slam/landmarks (scripts/slam_bridge.py: кадр map, метры, пол на z = 0; x, y, z — точка,
kx, ky, kz — камера ключевого кадра, откуда её видели) и раз в period секунд строит сетку заново —
точки после оптимизации карты двигаются:
  луч камера -> точка (в плоскости) — свободно: точку оттуда видели, значит между ними пусто;
  точка на высоте obstacle_min_z..obstacle_max_z — занято (робот с камерой — около 0.75 м);
  точки пола (ниже obstacle_min_z) дают только свободный луч; точки выше obstacle_max_z (потолок,
  верх стеллажей) не учитываются: луч к ним в плоскости прошёл бы поверх препятствий;
  полоса вдоль пути робота (/slam/pose, радиус footprint_radius) — свободно: робот там прошёл.
Берутся точки, которые видели не меньше min_observations раз и не дальше max_range от камеры.
Клетка занята, если в ней есть точка препятствия и лучей через неё не больше чем в free_per_point
раз больше, чем таких точек (иначе точка — выброс посреди прохода); свободна — если через неё
прошло не меньше free_min_rays лучей; иначе неизвестна.

Публикует /map (transient local, как map_server). При выходе (и раз в 10 с) пишет save_dir/map.pgm
+ map.yaml — формат map_server Nav2 — и landmarks.npz (точки и путь: пересобрать карту без прогона). С load:=.../map.yaml ничего не строит, а публикует сохранённую
карту — для навигации по готовой карте.

Запуск: python3 scripts/landmark_grid.py --ros-args -p use_sim_time:=true -p save_dir:=out/slam/<прогон>
"""

import array
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

FREE, OCC, UNKNOWN = 0, 100, -1


def read_cloud(msg):
    col = {f.name: f.offset // 4 for f in msg.fields}
    pts = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)
    get = lambda n: pts[:, col[n]].astype(np.float64) if n in col else None  # noqa: E731
    return {n: get(n) for n in ("x", "y", "z", "observations", "kx", "ky", "kz")}


def build_grid(c, res, zmin, zmax, min_obs, max_range, free_min_rays, free_per_point, path=None, footprint_radius=0.3, margin=1.0):
    """Точки -> (сетка int8 [строки=y, столбцы=x], начало (x0, y0)). Счёт лучей и точек по клеткам."""
    ok = np.ones(len(c["x"]), bool)
    if c["observations"] is not None:
        ok &= c["observations"] >= min_obs
    have_kf = c["kx"] is not None
    P = np.c_[c["x"], c["y"], c["z"]][ok]
    K = np.c_[c["kx"], c["ky"], c["kz"]][ok] if have_kf else None
    if have_kf:
        near = np.linalg.norm(P[:, :2] - K[:, :2], axis=1) <= max_range
        P, K = P[near], K[near]
    if len(P) == 0:
        return None, None
    extent = [P[:, :2]] + ([K[:, :2]] if have_kf else []) + ([path] if path is not None and len(path) else [])
    lo = np.floor((np.vstack(extent).min(0) - margin) / res) * res
    hi = np.ceil((np.vstack(extent).max(0) + margin) / res) * res
    w, h = int(round((hi[0] - lo[0]) / res)), int(round((hi[1] - lo[1]) / res))
    occ = np.zeros((h, w), np.int32)
    rays = np.zeros((h, w), np.int32)

    def cell(xy):
        ij = np.floor((xy - lo) / res).astype(np.int64)
        return np.clip(ij[:, 1], 0, h - 1), np.clip(ij[:, 0], 0, w - 1)

    band = (P[:, 2] >= zmin) & (P[:, 2] <= zmax)
    r, q = cell(P[band, :2])
    np.add.at(occ, (r, q), 1)
    if have_kf:
        use = P[:, 2] <= zmax  # лучи к точкам пола и препятствий
        A, B = K[use, :2], P[use, :2]
        L = np.linalg.norm(B - A, axis=1)
        n = np.maximum(1, np.ceil(L / (0.5 * res)).astype(np.int64))
        idx = np.repeat(np.arange(len(A)), n)
        k = np.arange(n.sum()) - np.repeat(np.cumsum(n) - n, n)
        t = k / n[idx]  # 0 .. <1: сама точка (конец луча) не считается свободной
        S = A[idx] + (B[idx] - A[idx]) * t[:, None]
        r, q = cell(S)
        # одна клетка — один голос на луч (иначе длинные лучи «перекрашивают» клетки многократно)
        key = np.unique(idx * (h * w) + r * w + q)
        np.add.at(rays.ravel(), key % (h * w), 1)
    walked = np.zeros((h, w), bool)
    if path is not None and len(path):  # робот прошёл — свободно
        k = int(np.ceil(footprint_radius / res))
        dy, dx = np.mgrid[-k:k + 1, -k:k + 1]
        disk = (dx ** 2 + dy ** 2) * res ** 2 <= footprint_radius ** 2
        r, q = cell(path)
        rr = (r[:, None] + dy[disk][None]).ravel()
        qq = (q[:, None] + dx[disk][None]).ravel()
        inside = (rr >= 0) & (rr < h) & (qq >= 0) & (qq < w)
        walked[rr[inside], qq[inside]] = True
    grid = np.full((h, w), UNKNOWN, np.int8)
    grid[(rays >= free_min_rays) | walked] = FREE
    grid[(occ > 0) & (rays <= free_per_point * occ) & ~walked] = OCC
    return grid, lo


class LandmarkGrid(Node):
    def __init__(self):
        super().__init__("landmark_grid")
        p = self.declare_parameter
        self.res = p("resolution", 0.05).value
        self.zmin = p("obstacle_min_z", 0.10).value
        self.zmax = p("obstacle_max_z", 0.90).value
        self.min_obs = p("min_observations", 3).value
        self.max_range = p("max_range", 12.0).value
        self.footprint_radius = p("footprint_radius", 0.3).value
        self.free_min_rays = p("free_min_rays", 2).value
        self.free_per_point = p("free_per_point", 20).value
        self.period = p("period", 2.0).value
        self.frame = p("map_frame", "map").value
        self.save_dir = p("save_dir", "").value
        load = p("load", "").value
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.pub = self.create_publisher(OccupancyGrid, "map", qos)
        self.cloud = None
        self.path = []  # путь робота по SLAM (x, y)
        self.last_points = None
        self.grid = self.origin = None
        self.last_build = self.last_save = 0.0
        if load:
            self._load(load)
            self.create_timer(5.0, self._publish)  # пока нет подписчиков transient local — повтор не мешает
        else:
            self.create_subscription(PointCloud2, "slam/landmarks", self.on_cloud, 1)
            self.create_subscription(Odometry, "slam/pose", self.on_pose, 20)
            self.create_timer(0.5, self._tick)
            self.get_logger().info(f"карта препятствий из /slam/landmarks: клетка {self.res} м, препятствия на высоте "
                                   f"{self.zmin:.2f}–{self.zmax:.2f} м")

    def on_cloud(self, msg):
        self.cloud = msg

    def on_pose(self, msg):
        p = msg.pose.pose.position
        if not self.path or math.hypot(p.x - self.path[-1][0], p.y - self.path[-1][1]) > 0.05:
            self.path.append((p.x, p.y))

    def _tick(self):
        if self.cloud is None or time.time() - self.last_build < self.period:
            return
        self.last_build = time.time()
        self.last_points = read_cloud(self.cloud)
        grid, origin = build_grid(self.last_points, self.res, self.zmin, self.zmax, self.min_obs, self.max_range,
                                  self.free_min_rays, self.free_per_point, np.array(self.path).reshape(-1, 2), self.footprint_radius)
        if grid is None:
            return
        self.grid, self.origin = grid, origin
        self._publish()
        if self.save_dir and time.time() - self.last_save > 10.0:
            self.save()

    def _publish(self):
        if self.grid is None:
            return
        m = OccupancyGrid()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.info.resolution = float(self.res)
        m.info.height, m.info.width = self.grid.shape
        m.info.origin.position.x, m.info.origin.position.y = float(self.origin[0]), float(self.origin[1])
        m.info.origin.orientation.w = 1.0
        data = array.array("b")
        data.frombytes(self.grid.tobytes())
        m.data = data
        self.pub.publish(m)

    def save(self):
        if self.grid is None or not self.save_dir:
            return
        self.last_save = time.time()
        out = Path(self.save_dir)
        out.mkdir(parents=True, exist_ok=True)
        img = np.full(self.grid.shape, 205, np.uint8)  # формат map_saver: неизвестно 205, свободно 254, занято 0
        img[self.grid == FREE] = 254
        img[self.grid == OCC] = 0
        img = img[::-1]  # в картинке верх — большие y
        with open(out / "map.pgm", "wb") as f:
            f.write(f"P5\n{img.shape[1]} {img.shape[0]}\n255\n".encode())
            f.write(img.tobytes())
        (out / "map.yaml").write_text(
            f"image: map.pgm\nmode: trinary\nresolution: {self.res}\n"
            f"origin: [{self.origin[0]:.4f}, {self.origin[1]:.4f}, 0.0]\nnegate: 0\n"
            f"occupied_thresh: 0.65\nfree_thresh: 0.19\n", encoding="utf-8")
        if self.last_points is not None:  # исходные данные — пересобрать карту без нового прогона
            np.savez_compressed(out / "landmarks.npz", path=np.array(self.path).reshape(-1, 2),
                                **{k: v for k, v in self.last_points.items() if v is not None})
        n_occ, n_free = int((self.grid == OCC).sum()), int((self.grid == FREE).sum())
        self.get_logger().info(f"карта препятствий: {self.grid.shape[1]}x{self.grid.shape[0]} клеток, занято {n_occ}, "
                               f"свободно {n_free} -> {out / 'map.yaml'}")

    def _load(self, path):
        y = {}
        for line in Path(path).read_text(encoding="utf-8").splitlines():
            if ":" in line:
                k, v = line.split(":", 1)
                y[k.strip()] = v.strip()
        self.res = float(y["resolution"])
        self.origin = np.array([float(v) for v in y["origin"].strip("[]").split(",")[:2]])
        raw = (Path(path).parent / y["image"]).read_bytes()
        parts = raw.split(b"\n", 3)
        w, h = (int(v) for v in parts[1].split())
        img = np.frombuffer(parts[3], np.uint8)[: w * h].reshape(h, w)[::-1]
        grid = np.full((h, w), UNKNOWN, np.int8)
        grid[img >= 250] = FREE
        grid[img <= 50] = OCC
        self.grid = grid
        self._publish()
        self.get_logger().info(f"карта препятствий из {path}: {w}x{h} клеток по {self.res} м")


def main():
    rclpy.init()
    node = LandmarkGrid()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node.save_dir and node.cloud is not None:
            node.save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
