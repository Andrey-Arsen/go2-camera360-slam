#!/usr/bin/env python3
"""Мост SLAM -> метрическая карта: масштаб и горизонт карты stella по одометрии ног, TF map -> odom.

stella_vslam по одной камере строит карту в своих единицах и со своими осями (кадр slam_map — оси
камеры в начале карты). Мост переводит её в метрический кадр map, z вверх, пол на z = 0:
  горизонт — по крену и тангажу из TF odom -> base_link (там они от IMU): «верх» камеры в
    slam_map, усреднённый по всем позам;
  масштаб, курс и сдвиг — подобием в плоскости (Umeyama 2D) путей камеры по SLAM и по одометрии
    (TF odom -> base_link), когда одометрия прошла min_motion м; уточняется до freeze_after м, потом
    замораживается — карта не должна «плыть» под навигацией;
  z — как у одометрии (odom стоит на полу, scripts/odom_tf.py), поэтому пол карты на z = 0.
Дальше на каждую позу SLAM: T_map_base — из позы камеры, map -> odom = T_map_base * T_odom_base^-1:
это поправка одометрии по камере. Масштаб получается таким же точным, как одометрия на первых
метрах (в симуляции её ошибки задаёт ODOM_NOISE).

Публикует: TF map -> odom (время кадра + transform_tolerance), /slam/pose (nav_msgs/Odometry,
map -> base_link), /slam/landmarks (точки карты stella в map, метры, раз в секунду; kx, ky, kz —
камера, откуда точку видели — по ним scripts/landmark_grid.py строит карту препятствий).
Преобразование slam_map -> map пишется в transform_out (JSON); с transform_in (локализация по
готовой карте, stella --disable-mapping) ничего не оценивается — map -> odom идёт с первой позы.
Если stella начала карту заново (ключевых кадров стало меньше вдвое), оценка начинается заново.
Страховка: после заморозки путь камеры по SLAM сверяется с одометрией в скользящем окне
(check_window м); если они расходятся больше чем на check_tolerance (масштаб карты stella уплыл —
монокуляр), map -> odom не обновляется — держится последняя хорошая поправка, робот идёт по
одометрии, пока SLAM снова не сойдётся с ней. И фильтр скачков: поправка map -> odom меняется
медленно (одометрия уплывает на сантиметры за метр), поэтому скачок больше jump_max_m / jump_max_deg
(плюс jump_per_m на метр пути с прошлой принятой) не принимается, пока не продержится jump_confirm_s —
так короткие сбои SLAM на разворотах не дёргают робота, а настоящая поправка (замыкание петли) проходит.

Запуск (вместе со stella с параметром map_frame:=slam_map):
  python3 scripts/slam_bridge.py --ros-args -p use_sim_time:=true -p transform_out:=out/slam/<прогон>/map_transform.json
"""

import array
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseArray, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quat_to_matrix(x, y, z, w):
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(R):
    """(x, y, z, w) поворота R — устойчиво для любых углов."""
    tr = np.trace(R)
    if tr > 0:
        k = 2.0 * math.sqrt(tr + 1.0)
        return np.array([(R[2, 1] - R[1, 2]) / k, (R[0, 2] - R[2, 0]) / k, (R[1, 0] - R[0, 1]) / k, 0.25 * k])
    i = int(np.argmax(np.diag(R)))
    j, m = (i + 1) % 3, (i + 2) % 3
    k = 2.0 * math.sqrt(1.0 + R[i, i] - R[j, j] - R[m, m])
    q = np.zeros(4)
    q[i] = 0.25 * k
    q[j] = (R[j, i] + R[i, j]) / k
    q[m] = (R[m, i] + R[i, m]) / k
    q[3] = (R[m, j] - R[j, m]) / k
    return q


def pose_matrix(position, orientation):
    T = np.eye(4)
    T[:3, :3] = quat_to_matrix(orientation.x, orientation.y, orientation.z, orientation.w)
    T[:3, 3] = [position.x, position.y, position.z]
    return T


def transform_matrix(tr):
    return pose_matrix(tr.translation, tr.rotation)


def rotation_between(a, b):
    """Наименьший поворот, переводящий единичный a в единичный b."""
    v = np.cross(a, b)
    c = float(np.dot(a, b))
    if np.linalg.norm(v) < 1e-12:
        return np.eye(3) if c > 0 else np.diag([1.0, -1.0, -1.0])
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * (1.0 / (1.0 + c))


def umeyama2d(src, dst):
    """Подобие в плоскости dst ≈ s*R@src + t по точкам (N,2). Возвращает s, R(2x2), t(2)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    U, D, Vt = np.linalg.svd(xd.T @ xs / len(src))
    S = np.eye(2)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[1, 1] = -1
    R = U @ S @ Vt
    var_s = (xs ** 2).sum() / len(src)
    s = float(np.trace(np.diag(D) @ S) / var_s)
    return s, R, mu_d - s * R @ mu_s


class Bridge(Node):
    def __init__(self):
        super().__init__("slam_bridge")
        p = self.declare_parameter
        self.min_motion = p("min_motion", 1.0).value
        self.freeze_after = p("freeze_after", 5.0).value
        self.tolerance = p("transform_tolerance", 0.2).value
        self.check_window = p("check_window", 2.0).value
        self.check_tolerance = p("check_tolerance", 0.2).value
        self.jump_max_m = p("jump_max_m", 0.10).value
        self.jump_max_deg = p("jump_max_deg", 3.0).value
        self.jump_per_m = p("jump_per_m", 0.05).value
        self.jump_confirm_s = p("jump_confirm_s", 1.0).value
        self.transform_in = p("transform_in", "").value
        self.transform_out = p("transform_out", "").value
        self.map_frame = p("map_frame", "map").value
        self.odom_frame = p("odom_frame", "odom").value
        self.base_frame = p("base_frame", "base_link").value
        self.camera_frame = p("camera_frame", "insta360_link").value
        slam_ns = p("slam_node", "run_slam").value

        self.tf = Buffer(cache_time=Duration(seconds=60))
        # TF принимается в своей (реентерабельной) группе — в многопоточном исполнителе он идёт,
        # пока обработчик позы ждёт TF; сами позы/точки обрабатываются по одной
        self._tf_listener = TransformListener(self.tf, self)
        self.tf_pub = TransformBroadcaster(self)
        self.pose_pub = self.create_publisher(Odometry, "slam/pose", 10)
        self.lm_pub = self.create_publisher(PointCloud2, "slam/landmarks", 1)
        self.create_subscription(Odometry, f"/{slam_ns}/camera_pose", self.on_pose, 50)
        self.create_subscription(PointCloud2, f"/{slam_ns}/landmarks", self.on_landmarks, 2)
        self.create_subscription(PoseArray, f"/{slam_ns}/keyframes", self.on_keyframes, 2)

        self.T_base_cam = None
        self.sim3 = None  # (s, R 3x3, t 3): p_map = s*R@p_slam + t
        self.frozen = False
        self.loaded = False
        self.max_keyframes = 0
        self.recent = []  # (путь одометрии, позиция камеры по SLAM в map, по одометрии) — для страховки
        self.holding = False
        self.last_good_map_odom = None  # поправка, которую держим при расхождении
        self.accepted = None  # (поправка map -> odom, путь одометрии) последней принятой
        self.jump_since = None  # время начала непринятого скачка
        self.jumps = 0
        self._reset_estimation()
        if self.transform_in:
            self._load(self.transform_in)
        else:
            self.get_logger().info(f"масштаб и горизонт карты — по одометрии: оценка после {self.min_motion:.1f} м пути, "
                                   f"заморозка после {self.freeze_after:.1f} м")

    def _reset_estimation(self):
        self.samples = []  # (p_slam, p_odom_cam, up_slam)
        self.odom_path = 0.0
        self.last_cam_xy = None
        self.last_estimate_n = 0

    # --- оценка slam_map -> map -------------------------------------------------------------
    def _estimate(self):
        P = np.array([s[0] for s in self.samples])
        Q = np.array([s[1] for s in self.samples])
        U = np.array([s[2] for s in self.samples]).mean(0)
        R_level = rotation_between(U / np.linalg.norm(U), np.array([0.0, 0.0, 1.0]))
        L = P @ R_level.T
        s, R2, t2 = umeyama2d(L[:, :2], Q[:, :2])
        th = math.atan2(R2[1, 0], R2[0, 0])
        Rz = np.array([[math.cos(th), -math.sin(th), 0.0], [math.sin(th), math.cos(th), 0.0], [0.0, 0.0, 1.0]])
        R = Rz @ R_level
        tz = float(np.mean(Q[:, 2] - s * L[:, 2]))
        t = np.array([t2[0], t2[1], tz])
        res = np.linalg.norm((P @ R.T * s + t - Q)[:, :2], axis=1)
        self.sim3 = (s, R, t)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, R_level[2, 2]))))
        self.fit_info = dict(samples=len(P), odom_path_m=round(self.odom_path, 2), residual_rms_cm=round(100 * float(np.sqrt(np.mean(res ** 2))), 1))
        self.get_logger().info(f"slam_map -> map: масштаб {s:.4f} м/ед., горизонт карты stella наклонён на {tilt:.2f}°, "
                               f"курс {math.degrees(th):+.1f}°; по {len(P)} позам, путь {self.odom_path:.1f} м, "
                               f"расхождение с одометрией {self.fit_info['residual_rms_cm']:.1f} см")
        if self.odom_path >= self.freeze_after and not self.frozen:
            self.frozen = True
            self.get_logger().info("преобразование заморожено")
            self._save()

    def _save(self):
        if not self.transform_out or self.sim3 is None:
            return
        s, R, t = self.sim3
        out = Path(self.transform_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        data = {"what": "p_map = scale * R * p_slam_map + t (stella_vslam_ros map_frame:=slam_map -> метрический map)",
                "scale": s, "rotation_xyzw": matrix_to_quat(R).tolist(), "rotation_matrix": R.tolist(),
                "translation": t.tolist(), "frozen": self.frozen, **getattr(self, "fit_info", {}),
                "saved": time.strftime("%Y-%m-%d %H:%M:%S")}
        out.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        self.get_logger().info(f"преобразование карты записано: {out}")

    def _load(self, path):
        d = json.loads(Path(path).read_text(encoding="utf-8"))
        R = np.array(d["rotation_matrix"], float)
        self.sim3 = (float(d["scale"]), R, np.array(d["translation"], float))
        self.frozen = self.loaded = True
        self.get_logger().info(f"преобразование карты из {path}: масштаб {d['scale']:.4f} м/ед. — map -> odom с первой позы")

    # --- входы ----------------------------------------------------------------------------
    def on_keyframes(self, msg):
        n = len(msg.poses)
        if not self.loaded and self.max_keyframes >= 10 and n < self.max_keyframes / 2:
            self.get_logger().warning(f"stella начала карту заново ({self.max_keyframes} -> {n} ключевых кадров) — "
                                      "оценка масштаба и горизонта заново")
            self.sim3, self.frozen, self.max_keyframes = None, False, 0
            self._reset_estimation()
        self.max_keyframes = max(self.max_keyframes, n)

    def on_pose(self, msg):
        stamp = Time.from_msg(msg.header.stamp)
        try:
            if self.T_base_cam is None:
                self.T_base_cam = transform_matrix(self.tf.lookup_transform(self.base_frame, self.camera_frame, Time()).transform)
            T_odom_base = transform_matrix(self.tf.lookup_transform(self.odom_frame, self.base_frame, stamp,
                                                                    timeout=Duration(seconds=0.3)).transform)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"нет TF для позы SLAM: {e}", throttle_duration_sec=5.0)
            return
        T_slam_cam = pose_matrix(msg.pose.pose.position, msg.pose.pose.orientation)
        T_odom_cam = T_odom_base @ self.T_base_cam
        if not self.frozen:
            up_cam = T_odom_cam[:3, :3].T @ np.array([0.0, 0.0, 1.0])  # «верх» в кадре камеры (крен/тангаж от IMU)
            self.samples.append((T_slam_cam[:3, 3].copy(), T_odom_cam[:3, 3].copy(), T_slam_cam[:3, :3] @ up_cam))
            xy = T_odom_cam[:2, 3]
            if self.last_cam_xy is not None:
                self.odom_path += float(np.linalg.norm(xy - self.last_cam_xy))
            self.last_cam_xy = xy.copy()
            if self.odom_path >= self.min_motion and len(self.samples) - self.last_estimate_n >= 10:
                self.last_estimate_n = len(self.samples)
                self._estimate()
        if self.sim3 is None:
            return
        s, R, t = self.sim3
        T_map_cam = np.eye(4)
        T_map_cam[:3, :3] = R @ T_slam_cam[:3, :3]
        T_map_cam[:3, 3] = s * R @ T_slam_cam[:3, 3] + t
        T_map_base = T_map_cam @ np.linalg.inv(self.T_base_cam)
        T_map_odom = T_map_base @ np.linalg.inv(T_odom_base)
        if self.frozen:
            T_map_odom = self._check(T_map_cam[:3, 3], T_odom_cam[:3, 3], T_map_odom)
            T_map_odom = self._filter_jump(T_map_odom, stamp.nanoseconds * 1e-9)
            T_map_base = T_map_odom @ T_odom_base

        tf_msg = TransformStamped()
        tf_msg.header.stamp = (stamp + Duration(seconds=self.tolerance)).to_msg()
        tf_msg.header.frame_id, tf_msg.child_frame_id = self.map_frame, self.odom_frame
        self._fill(tf_msg.transform.translation, tf_msg.transform.rotation, T_map_odom)
        self.tf_pub.sendTransform(tf_msg)
        pose = Odometry()
        pose.header.stamp = msg.header.stamp
        pose.header.frame_id, pose.child_frame_id = self.map_frame, self.base_frame
        self._fill(pose.pose.pose.position, pose.pose.pose.orientation, T_map_base)
        self.pose_pub.publish(pose)

    def _check(self, p_map_cam, p_odom_cam, T_map_odom):
        """Сверка пути камеры по SLAM и по одометрии за последние check_window м. Расхождение видно
        только когда окно наберёт путь, поэтому при срабатывании берётся поправка из начала окна —
        до того, как SLAM поехал, — и держится, пока SLAM снова не сойдётся с одометрией."""
        dist = self.recent[-1][0] + float(np.linalg.norm(p_odom_cam[:2] - self.recent[-1][2][:2])) if self.recent else 0.0
        self.recent.append((dist, p_map_cam.copy(), p_odom_cam.copy(), T_map_odom))
        while len(self.recent) > 2 and dist - self.recent[1][0] >= self.check_window:
            self.recent.pop(0)
        span = dist - self.recent[0][0]
        d_odom = float(np.linalg.norm(p_odom_cam[:2] - self.recent[0][2][:2]))
        # решать только по участку, где робот ехал, а не крутился: смещение за окно не меньше половины окна
        if span >= self.check_window and d_odom >= 0.5 * self.check_window:
            d_slam = float(np.linalg.norm(p_map_cam[:2] - self.recent[0][1][:2]))
            bad = abs(d_slam / d_odom - 1.0) > self.check_tolerance
            if bad and not self.holding:
                self.last_good_map_odom = self.recent[0][3]
                self.get_logger().warning(f"SLAM расходится с одометрией: за {span:.1f} м смещение {d_slam:.2f} м против "
                                          f"{d_odom:.2f} м — держу поправку map -> odom {span:.1f} м назад")
            elif not bad and self.holding:
                self.get_logger().info(f"SLAM снова сходится с одометрией ({d_slam:.2f} м против {d_odom:.2f} м) — поправка идёт")
            self.holding = bad
        return self.last_good_map_odom if self.holding else T_map_odom

    def _filter_jump(self, T_map_odom, t):
        dist = self.recent[-1][0] if self.recent else 0.0  # путь одометрии (камера), м
        if self.accepted is None:
            self.accepted = (T_map_odom, dist)
            return T_map_odom
        T_prev, d_prev = self.accepted
        D = np.linalg.inv(T_prev) @ T_map_odom
        moved = float(np.linalg.norm(D[:2, 3]))
        turned = math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(D[:3, :3]) - 1.0) / 2.0))))
        allow = self.jump_per_m * max(0.0, dist - d_prev)
        if moved <= self.jump_max_m + allow and turned <= self.jump_max_deg + 20.0 * allow:
            self.accepted, self.jump_since = (T_map_odom, dist), None
            return T_map_odom
        if self.jump_since is None:
            self.jump_since = t
            self.jumps += 1
        if t - self.jump_since >= self.jump_confirm_s:  # держится — значит настоящая поправка
            self.get_logger().info(f"поправка map -> odom сместилась на {100 * moved:.0f} см / {turned:.1f}° и держится "
                                   f"{t - self.jump_since:.1f} с — принимаю")
            self.accepted, self.jump_since = (T_map_odom, dist), None
            return T_map_odom
        self.get_logger().warning(f"скачок SLAM {100 * moved:.0f} см / {turned:.1f}° — пока не принимаю", throttle_duration_sec=2.0)
        return T_prev

    @staticmethod
    def _fill(position, orientation, T):
        position.x, position.y, position.z = (float(v) for v in T[:3, 3])
        q = matrix_to_quat(T[:3, :3])
        orientation.x, orientation.y, orientation.z, orientation.w = (float(v) for v in q)

    def on_landmarks(self, msg):
        if self.sim3 is None or msg.width * msg.height == 0:
            return
        s, R, t = self.sim3
        pts = np.frombuffer(msg.data, np.float32).reshape(-1, msg.point_step // 4)
        out = pts.copy()
        col = {f.name: f.offset // 4 for f in msg.fields}
        for names in (("x", "y", "z"), ("kx", "ky", "kz")):  # точка и камера, откуда её видели
            if all(n in col for n in names):
                idx = [col[n] for n in names]
                out[:, idx] = (pts[:, idx].astype(np.float64) @ R.T * s + t).astype(np.float32)
        m = PointCloud2()
        m.header.stamp = msg.header.stamp
        m.header.frame_id = self.map_frame
        m.height, m.width = 1, len(out)
        m.fields = [PointField(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count) for f in msg.fields]
        m.is_bigendian, m.point_step, m.is_dense = msg.is_bigendian, msg.point_step, True
        m.row_step = m.point_step * m.width
        data = array.array("B")
        data.frombytes(out.tobytes())
        m.data = data
        self.lm_pub.publish(m)


def main():
    rclpy.init()
    node = Bridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if not node.loaded:
            node._save()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
