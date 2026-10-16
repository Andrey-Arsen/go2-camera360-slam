#!/usr/bin/env python3
"""Проверка 360-камеры в ROS 2: идут ли кадры обоих fisheye, с какой частотой, есть ли TF.

Слушает --seconds секунд (по стенным часам), затем печатает по каждому объективу: размер,
кодировку, frame_id, частоту в секундах симуляции и стенных, круг изображения (должен быть
вписан в кадр — так задан объектив 200°), где объектив относительно base_link по TF, и
сохраняет последний кадр каждого в out/check/<объектив>.png. Код выхода 0 — всё в порядке.

cv_bridge не используется: в системном Python numpy 2, а cv_bridge Humble собран под numpy 1.

Запуск (при работающем scripts/go2_sim.sh или run_go2_isaac.py):
  source /opt/ros/humble/setup.bash && python3 scripts/check_cameras.py [--seconds 20]
"""

import argparse
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

ROOT = Path(__file__).resolve().parents[1]
LENSES = ("front", "back")


def image_array(msg):
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"кодировка {msg.encoding} не ожидалась (rgb8)")
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    return a if msg.encoding == "rgb8" else a[..., ::-1]


def image_circle(rgb):
    """Радиус круга изображения в долях полуширины: по самому дальнему от центра ненулевому пикселю
    вдоль 4 сторон и 4 диагоналей (до края кадра). Вписанный круг — по сторонам ~1.0 и по диагоналям
    ~1.0; если бы объектив не масштабировался под размер кадра, по диагоналям вышло бы ~1.41."""
    h, w, _ = rgb.shape
    lit = rgb.max(axis=2) > 3
    cy, cx = (h - 1) / 2, (w - 1) / 2
    out = []
    for dy, dx in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        uy, ux = dy / np.hypot(dx, dy), dx / np.hypot(dx, dy)
        lim = min(cx / abs(ux) if ux else np.inf, cy / abs(uy) if uy else np.inf)
        r = np.arange(0, lim, 0.5)
        y = np.round(cy + r * uy).astype(int)
        x = np.round(cx + r * ux).astype(int)
        hit = np.nonzero(lit[y, x])[0]
        out.append(r[hit.max()] / (w / 2) if len(hit) else 0.0)
    return np.array(out)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=20.0, help="сколько слушать, стенных секунд")
    parser.add_argument("--out", default=str(ROOT / "out" / "check"))
    args = parser.parse_args()
    rclpy.init()
    node = rclpy.create_node("check_cameras", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, node)
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=2)
    got = {n: [] for n in LENSES}  # (стенное время, время сообщения, сообщение)

    for n in LENSES:
        node.create_subscription(Image, f"/insta360/{n}/image_raw",
                                 lambda m, n=n: got[n].append((time.time(), Time.from_msg(m.header.stamp).nanoseconds * 1e-9, m)), qos)
    t0 = time.time()
    while time.time() - t0 < args.seconds:
        rclpy.spin_once(node, timeout_sec=0.1)

    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    ok = True
    for n in LENSES:
        frames = got[n]
        if not frames:
            print(f"[check] {n}: кадров нет за {args.seconds:.0f} с — Isaac Sim запущен? ROS_DOMAIN_ID тот же?")
            ok = False
            continue
        _, _, m = frames[-1]
        rgb = image_array(m)
        cv2.imwrite(str(out / f"{n}.png"), rgb[..., ::-1])
        rate = ""
        if len(frames) > 2:
            wall = (frames[-1][0] - frames[0][0]) / (len(frames) - 1)
            sim = (frames[-1][1] - frames[0][1]) / (len(frames) - 1)
            rate = f", {1 / sim:.1f} Гц по времени симуляции, {1 / wall:.1f} Гц по стенным часам" if sim > 0 and wall > 0 else ""
        circle = image_circle(rgb)
        print(f"[check] {n}: {len(frames)} кадров, {m.width}x{m.height} {m.encoding}, frame_id {m.header.frame_id}{rate}")
        print(f"[check] {n}: круг изображения по сторонам {np.round(circle[:4], 3)}, по диагоналям {np.round(circle[4:], 3)} "
              f"(вписанный круг — около 1.0 и 1.0)")
        if not (np.all(np.abs(circle[:4] - 1.0) < 0.03) and abs(np.median(circle[4:]) - 1.0) < 0.03):
            print(f"[check] {n}: !! круг изображения не вписан в кадр — объектив масштабируется не так, как ожидалось")
            ok = False
        elif circle[4:].max() > 1.05:
            print(f"[check] {n}: за кругом изображения в углу есть светлые пиксели — артефакт рендера вне поля "
                  f"объектива, маска кадра его закроет")
        try:
            T = tf.lookup_transform("base_link", m.header.frame_id, Time())
            p = T.transform.translation
            print(f"[check] {n}: {m.header.frame_id} относительно base_link: x {p.x * 1000:+.1f} y {p.y * 1000:+.1f} z {p.z * 1000:+.1f} мм")
        except Exception as e:  # noqa: BLE001
            print(f"[check] {n}: !! нет TF base_link -> {m.header.frame_id}: {e}")
            ok = False
    try:
        T = tf.lookup_transform("world", "base_link", Time())
        p = T.transform.translation
        print(f"[check] base_link в world: x {p.x:+.2f} y {p.y:+.2f} z {p.z:+.3f} м")
    except Exception as e:  # noqa: BLE001
        print(f"[check] !! нет TF world -> base_link: {e}")
        ok = False
    print(f"[check] кадры: {out}/front.png, back.png" if any(got.values()) else "")
    print("[check] всё в порядке" if ok else "[check] есть проблемы — см. строки с !!")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
