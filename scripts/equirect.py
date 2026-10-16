#!/usr/bin/env python3
"""Склейка двух fisheye Insta360 в 360-панораму (equirectangular) для stella_vslam.

Слушает /insta360/front/image_raw и /insta360/back/image_raw (кадры с одним временем), берёт
положение объективов из TF (insta360_link -> insta360_front_optical / insta360_back_optical) и
публикует /insta360/equirect/image_raw (rgb8, width x height, кадр insta360_optical), а сам кадр
insta360_optical — в /tf_static: это insta360_link с осями камеры (Z вперёд, X вправо, Y вниз).

Панорама — как её ждёт stella_vslam (модель Equirectangular): столбец u -> долгота
lon = (u/width - 0.5)*360°, строка v -> широта lat = (0.5 - v/height)*180°; середина кадра —
вперёд робота, верх — вверх. Объектив — равноугольный fisheye (fov, по умолчанию 200°), круг
изображения вписан в квадратный кадр. Каждый пиксель панорамы берётся из объектива, к оси
которого он ближе; в полосе шва (90° ± blend_deg/2 от оси) — смесь обоих. Объективы разнесены
на ~4 см, поэтому склейка точна для предметов на расстоянии stitch_distance, а у ближних и
дальних у шва остаётся сдвиг до долей градуса — как у настоящей камеры.

lenses:=front — только передний объектив (задняя половина панорамы чёрная): тот же SLAM
«на одном fisheye» для сравнения.

Маска для stella (параметр mask_out, PNG): 0 — не искать там точки: тело робота
(config/robot_mask_front.png, robot_mask_back.png — делает scripts/make_robot_mask.sh), места
без изображения и, при lenses:=front, задняя половина. Пишется один раз при старте узла.

cv_bridge не используется (в системном Python numpy 2, а cv_bridge Humble собран под numpy 1).

Запуск: source scripts/slam_env.sh && python3 scripts/equirect.py --ros-args -p use_sim_time:=true
        [-p width:=1920 -p height:=960 -p lenses:=both -p mask_out:=out/slam/mask.png]
"""

import array
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, StaticTransformBroadcaster, TransformListener

ROOT = Path(__file__).resolve().parents[1]
LINK = "insta360_link"
OPTICAL = "insta360_optical"
LENS_FRAMES = {"front": "insta360_front_optical", "back": "insta360_back_optical"}
# insta360_link (X вперёд, Y влево, Z вверх) <- оптические оси панорамы (X вправо, Y вниз, Z вперёд)
R_LINK_OPT = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])


def quat_to_matrix(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def matrix_to_quat(R):
    """Кватернион (x, y, z, w) поворота R."""
    w = np.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
    x = np.sqrt(max(0.0, 1.0 + R[0, 0] - R[1, 1] - R[2, 2])) / 2.0
    y = np.sqrt(max(0.0, 1.0 - R[0, 0] + R[1, 1] - R[2, 2])) / 2.0
    z = np.sqrt(max(0.0, 1.0 - R[0, 0] - R[1, 1] + R[2, 2])) / 2.0
    x = np.copysign(x, R[2, 1] - R[1, 2])
    y = np.copysign(y, R[0, 2] - R[2, 0])
    z = np.copysign(z, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def equirect_bearings(width, height):
    """Направления пикселей панорамы в кадре insta360_link (как convert_point_to_bearing у stella)."""
    lon = (np.arange(width) / width - 0.5) * 2.0 * np.pi
    lat = -(np.arange(height) / height - 0.5) * np.pi
    lon, lat = np.meshgrid(lon, lat)
    b_opt = np.stack([np.cos(lat) * np.sin(lon), -np.sin(lat), np.cos(lat) * np.cos(lon)], axis=-1)
    return b_opt @ R_LINK_OPT.T


def lens_maps(bearings, R, t, size, fov_deg, stitch_distance):
    """Для объектива с осями R (столбцы — его оптические X, Y, Z в insta360_link) и центром t:
    координаты в его кадре (map_x, map_y) и угол от оси (град) для каждого пикселя панорамы."""
    P = bearings * stitch_distance - t  # точка на расстоянии склейки, от центра объектива
    d = P @ R  # в оптическом кадре объектива
    d /= np.linalg.norm(d, axis=-1, keepdims=True)
    theta = np.arccos(np.clip(d[..., 2], -1.0, 1.0))
    phi = np.arctan2(d[..., 1], d[..., 0])
    half = size / 2.0
    r = theta / np.radians(fov_deg / 2.0) * half
    c = (size - 1) / 2.0
    map_x = (c + r * np.cos(phi)).astype(np.float32)
    map_y = (c + r * np.sin(phi)).astype(np.float32)
    return map_x, map_y, np.degrees(theta)


def image_to_array(msg):
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"кодировка {msg.encoding} не поддерживается (нужна rgb8)")
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    a = a.reshape(msg.height, msg.width, 3)
    return a if msg.encoding == "rgb8" else a[..., ::-1]


class Equirect(Node):
    def __init__(self):
        super().__init__("equirect")
        p = self.declare_parameter
        self.width = p("width", 1920).value
        self.height = p("height", 960).value
        self.fov = p("fov_deg", 200.0).value
        self.stitch_distance = p("stitch_distance", 4.0).value
        self.blend = p("blend_deg", 4.0).value
        self.lenses = p("lenses", "both").value
        self.mask_out = p("mask_out", "").value
        self.mask_dilate = p("mask_dilate_px", 8).value
        self.robot_mask_dir = p("robot_mask_dir", str(ROOT / "config")).value
        out_topic = p("output", "insta360/equirect/image_raw").value
        if self.lenses not in ("both", "front", "back"):
            raise SystemExit(f"lenses: both, front или back, а не {self.lenses}")

        self.tf = Buffer()
        self._tf_listener = TransformListener(self.tf, self)
        self.static_tf = StaticTransformBroadcaster(self)
        self.pub = self.create_publisher(Image, out_topic, QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=2))
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=3)
        if self.lenses == "both":
            subs = [Subscriber(self, Image, f"insta360/{n}/image_raw", qos_profile=qos) for n in LENS_FRAMES]
            self.sync = ApproximateTimeSynchronizer(subs, queue_size=6, slop=0.01)
            self.sync.registerCallback(self.on_pair)
        else:  # один объектив: второй не нужен (и может не публиковаться)
            one = self.lenses
            self.create_subscription(Image, f"insta360/{one}/image_raw",
                                     lambda m: self.on_pair(m, None) if one == "front" else self.on_pair(None, m), qos)
        self.maps = None  # строятся по первому кадру: нужен размер кадра объектива и TF
        self.n, self.t_proc, self.t_log = 0, 0.0, time.time()
        self._publish_optical_frame()
        self.get_logger().info(f"жду кадры обоих объективов и TF {LINK} -> {', '.join(LENS_FRAMES.values())}")

    def _publish_optical_frame(self):
        m = TransformStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = LINK
        m.child_frame_id = OPTICAL
        q = matrix_to_quat(R_LINK_OPT)
        m.transform.rotation.x, m.transform.rotation.y, m.transform.rotation.z, m.transform.rotation.w = q
        self.static_tf.sendTransform(m)

    def _lens_pose(self, name):
        tr = self.tf.lookup_transform(LINK, LENS_FRAMES[name], Time()).transform
        q = tr.rotation
        R = quat_to_matrix(q.x, q.y, q.z, q.w)
        t = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
        return R, t

    def _build(self, size):
        """Таблицы выборки: один remap из [перед | зад] для «своего» объектива каждого пикселя
        и маленький — для второго объектива в полосе шва."""
        b = equirect_bearings(self.width, self.height)
        mx, my, th = {}, {}, {}
        for i, n in enumerate(LENS_FRAMES):
            R, t = self._lens_pose(n)
            mx[n], my[n], th[n] = lens_maps(b, R, t, size, self.fov, self.stitch_distance)
            mx[n] += i * size  # кадры объективов лежат рядом: [перед | зад]
            if self.lenses not in ("both", n):
                th[n] = np.full_like(th[n], 999.0)  # объектив выключен
            self.get_logger().info(f"{n}: центр объектива {np.round(t * 1000, 1)} мм от {LINK}")
        lim = self.fov / 2.0
        w = {}
        for n in LENS_FRAMES:  # вес объектива: 1 до шва, 0 за ним, линейно в полосе blend_deg
            if self.blend > 0:
                w[n] = np.clip((90.0 + self.blend / 2.0 - th[n]) / self.blend, 0.0, 1.0)
            else:
                w[n] = (th[n] < 90.0).astype(np.float64)
            w[n][th[n] > lim] = 0.0
        tot = w["front"] + w["back"]
        covered = tot > 0
        wf = np.where(covered, w["front"] / np.maximum(tot, 1e-9), 0.0)
        main_front = wf >= 0.5
        main_x = np.where(main_front, mx["front"], mx["back"]).astype(np.float32)
        main_y = np.where(main_front, my["front"], my["back"]).astype(np.float32)
        main_x[~covered], main_y[~covered] = -1.0, -1.0  # чёрное
        band = covered & (wf > 0.0) & (wf < 1.0)
        bi = np.nonzero(band)
        # полоса шва у полюсов расширяется (до сотни тысяч пикселей), а remap берёт не больше
        # 32767 по стороне — раскладываем её строками по 1024
        k, cols = len(bi[0]), 1024
        rows = max(1, -(-k // cols))

        def as_grid(v):
            return np.pad(v.astype(np.float32), (0, rows * cols - k), constant_values=-1.0).reshape(rows, cols)

        other_x = as_grid(np.where(main_front, mx["back"], mx["front"])[bi])
        other_y = as_grid(np.where(main_front, my["back"], my["front"])[bi])
        w_main = np.where(main_front, wf, 1.0 - wf)[bi].astype(np.float32)[:, None]
        self.maps = dict(size=size, main=cv2.convertMaps(main_x, main_y, cv2.CV_16SC2), band=bi,
                         other_x=other_x, other_y=other_y, w_main=w_main, covered=covered,
                         lens_x={n: mx[n] for n in LENS_FRAMES}, lens_y={n: my[n] for n in LENS_FRAMES},
                         main_front=main_front)
        self.get_logger().info(
            f"панорама {self.width}x{self.height} из объективов {size} px ({self.lenses}); "
            f"полоса шва {len(bi[0])} пикс., без изображения {np.count_nonzero(~covered)} пикс.")
        if self.mask_out:
            self._write_mask(size)

    def _write_mask(self, size):
        m = self.maps
        ok = m["covered"].copy()
        robot = np.zeros_like(ok)
        found = []
        for n in LENS_FRAMES:
            f = Path(self.robot_mask_dir) / f"robot_mask_{n}.png"
            if not f.exists():
                continue
            rm = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
            rm = cv2.resize(rm, (size, size), interpolation=cv2.INTER_NEAREST)
            x = m["lens_x"][n] - (list(LENS_FRAMES).index(n) * size)
            hit = cv2.remap(rm, x, m["lens_y"][n], cv2.INTER_NEAREST, borderValue=0) > 127
            own = m["main_front"] if n == "front" else ~m["main_front"]
            robot |= hit & own
            found.append(n)
        if not found:
            self.get_logger().warning(f"нет масок тела робота в {self.robot_mask_dir} (scripts/make_robot_mask.sh) — "
                                      "SLAM будет видеть свой корпус")
        if self.mask_dilate > 0 and robot.any():
            k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * self.mask_dilate + 1,) * 2)
            robot = cv2.dilate(robot.astype(np.uint8), k).astype(bool)
        ok &= ~robot
        out = Path(self.mask_out)
        out.parent.mkdir(parents=True, exist_ok=True)
        tmp = out.with_name(out.stem + ".part.png")
        cv2.imwrite(str(tmp), np.where(ok, 255, 0).astype(np.uint8))
        tmp.replace(out)
        self.get_logger().info(f"маска для stella: {out} (тело робота — {100.0 * robot.mean():.1f}% панорамы, "
                               f"по маскам: {', '.join(found) or 'нет'})")

    def on_pair(self, front, back):
        t0 = time.time()
        a = image_to_array(front) if front is not None else None
        b = image_to_array(back) if back is not None else None
        if a is None:
            a = np.zeros_like(b)
        if b is None:
            b = np.zeros_like(a)
        head = front if front is not None else back
        if a.shape != b.shape or a.shape[0] != a.shape[1]:
            self.get_logger().error(f"кадры объективов должны быть одинаковые и квадратные: {a.shape}, {b.shape}")
            return
        if self.maps is None or self.maps["size"] != a.shape[0]:
            try:
                self._build(a.shape[0])
            except Exception as e:  # noqa: BLE001 — TF ещё не пришёл
                self.get_logger().warning(f"пока не строю панораму: {e}", throttle_duration_sec=5.0)
                self.maps = None
                return
        m = self.maps
        src = np.hstack([a, b])
        out = cv2.remap(src, m["main"][0], m["main"][1], cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
        if len(m["band"][0]):
            other = cv2.remap(src, m["other_x"], m["other_y"], cv2.INTER_LINEAR,
                              borderMode=cv2.BORDER_CONSTANT).reshape(-1, 3)[:len(m["band"][0])]
            main = out[m["band"]].astype(np.float32)
            out[m["band"]] = (main * m["w_main"] + other.astype(np.float32) * (1.0 - m["w_main"])).astype(np.uint8)
        msg = Image()
        msg.header.stamp = head.header.stamp
        msg.header.frame_id = OPTICAL
        msg.height, msg.width = out.shape[:2]
        msg.encoding = "rgb8"
        msg.step = msg.width * 3
        data = array.array("B")  # не bytes: их rclpy проверяет поштучно в Python (~0.2 с на кадр)
        data.frombytes(out.tobytes())
        msg.data = data
        self.pub.publish(msg)
        self.n += 1
        self.t_proc += time.time() - t0
        if time.time() - self.t_log > 30.0:
            self.get_logger().info(f"панорам {self.n}, склейка {1000.0 * self.t_proc / max(self.n, 1):.1f} мс на кадр")
            self.t_log = time.time()


def main():
    rclpy.init()
    node = Equirect()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
