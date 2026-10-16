#!/usr/bin/env python3
"""Сверка SLAM с эталоном симуляции: метрическая поза, одометрия ног и геометрия stella.

Эталон — TF world -> base_link_gt (Isaac Sim, run_go2_isaac.py --gt-frame base_link_gt). В моменты
поз SLAM сравниваются с ним:
  SLAM (метры)  — /slam/pose (map -> base_link от scripts/slam_bridge.py: масштаб — по одометрии);
                  совмещение поворотом и сдвигом, БЕЗ подгонки масштаба: так видна и ошибка масштаба;
  одометрия     — TF odom -> base_link (scripts/odom_tf.py, с ошибками ODOM_NOISE): что было бы без камеры;
  stella        — /run_slam/camera_pose против эталонной позы камеры, совмещение подобием (Umeyama
                  Sim(3)): точность самой геометрии карты, без вопроса масштаба.
Пока идёт — публикует для RViz (кадр map): /slam/path (SLAM), /slam/gt_path (эталон, совмещённый с
картой), /slam/odom_path (одометрия). По Ctrl+C (или --seconds) пишет в --out: report.txt,
trajectory.png (вид сверху), poses.npz, map_world.json (связь кадра map с эталоном — для nav_goals.py).

Запуск: source scripts/slam_env.sh && python3 scripts/eval_slam.py --out out/slam/<прогон>
"""

import argparse
import json
import signal
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as PathMsg
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

WORLD, GT, MAP, ODOM, BASE, CAM = "world", "base_link_gt", "map", "odom", "base_link", "insta360_link"


def umeyama(src, dst, with_scale=True):
    """dst ≈ s*R@src + t по точкам (N,3). Возвращает s, R, t (s = 1 без масштаба)."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    xs, xd = src - mu_s, dst - mu_d
    U, D, Vt = np.linalg.svd(xd.T @ xs / len(src))
    S = np.eye(3)
    if np.linalg.det(U) * np.linalg.det(Vt) < 0:
        S[2, 2] = -1
    R = U @ S @ Vt
    var_s = (xs ** 2).sum() / len(src)
    s = float(np.trace(np.diag(D) @ S) / var_s) if with_scale and var_s > 0 else 1.0
    return s, R, mu_d - s * R @ mu_s


def ate(src, dst, with_scale):
    s, R, t = umeyama(src, dst, with_scale)
    A = src @ R.T * s + t
    return np.linalg.norm(A - dst, axis=1), A, s, (s, R, t)


def stats(e):
    return f"RMSE {100 * np.sqrt(np.mean(e ** 2)):.1f} см, медиана {100 * np.median(e):.1f} см, максимум {100 * e.max():.1f} см"


def pos(tr):
    return np.array([tr.translation.x, tr.translation.y, tr.translation.z])


def rot(q):
    return np.array([[1 - 2 * (q.y ** 2 + q.z ** 2), 2 * (q.x * q.y - q.z * q.w), 2 * (q.x * q.z + q.y * q.w)],
                     [2 * (q.x * q.y + q.z * q.w), 1 - 2 * (q.x ** 2 + q.z ** 2), 2 * (q.y * q.z - q.x * q.w)],
                     [2 * (q.x * q.z - q.y * q.w), 2 * (q.y * q.z + q.x * q.w), 1 - 2 * (q.x ** 2 + q.y ** 2)]])


class Eval:
    def __init__(self, node, fps):
        self.node, self.fps = node, fps
        self.tf = Buffer(cache_time=Duration(seconds=7200))
        self._listener = TransformListener(self.tf, node)
        self.pending_slam, self.pending_raw = [], []
        self.t, self.slam, self.gt, self.odom = [], [], [], []  # метрическая поза base_link
        self.raw_t, self.raw, self.raw_gt = [], [], []  # камера: stella (свои ед.) и эталон
        node.create_subscription(Odometry, "/slam/pose", lambda m: self.pending_slam.append(m), 50)
        node.create_subscription(Odometry, "/run_slam/camera_pose", lambda m: self.pending_raw.append(m), 50)
        self.pubs = {k: node.create_publisher(PathMsg, f"/slam/{k}", 1) for k in ("path", "gt_path", "odom_path")}
        node.create_timer(0.5, self.resolve)
        node.create_timer(2.0, self.publish_paths)
        self.T_base_cam = None

    def _ready(self, t, *pairs):
        return all(self.tf.can_transform(a, b, t) for a, b in pairs)

    def resolve(self):
        now = self.node.get_clock().now()
        keep = []
        for m in self.pending_slam:
            t = Time.from_msg(m.header.stamp)
            if self._ready(t, (WORLD, GT), (ODOM, BASE)):
                p = m.pose.pose.position
                self.t.append(t.nanoseconds * 1e-9)
                self.slam.append(np.array([p.x, p.y, p.z]))
                self.gt.append(pos(self.tf.lookup_transform(WORLD, GT, t).transform))
                self.odom.append(pos(self.tf.lookup_transform(ODOM, BASE, t).transform))
            elif (now - t).nanoseconds * 1e-9 < 5.0:
                keep.append(m)
        self.pending_slam = keep
        keep = []
        for m in self.pending_raw:
            t = Time.from_msg(m.header.stamp)
            if self._ready(t, (WORLD, GT), (BASE, CAM)):
                if self.T_base_cam is None:
                    self.T_base_cam = pos(self.tf.lookup_transform(BASE, CAM, Time()).transform)
                g = self.tf.lookup_transform(WORLD, GT, t).transform
                p = m.pose.pose.position
                self.raw_t.append(t.nanoseconds * 1e-9)
                self.raw.append(np.array([p.x, p.y, p.z]))
                self.raw_gt.append(pos(g) + rot(g.rotation) @ self.T_base_cam)
            elif (now - t).nanoseconds * 1e-9 < 5.0:
                keep.append(m)
        self.pending_raw = keep

    def publish_paths(self):
        if len(self.t) < 10:
            return
        S, G, O = np.array(self.slam), np.array(self.gt), np.array(self.odom)
        _, _, _, (_, R, t) = ate(G, S, with_scale=False)  # эталон -> кадр карты
        stamp = self.node.get_clock().now().to_msg()
        for key, P in (("path", S), ("gt_path", G @ R.T + t), ("odom_path", O)):
            m = PathMsg()
            m.header.stamp, m.header.frame_id = stamp, MAP
            for p in P[:: max(1, len(P) // 2000)]:
                ps = PoseStamped()
                ps.header.frame_id = MAP
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = map(float, p)
                ps.pose.orientation.w = 1.0
                m.poses.append(ps)
            self.pubs[key].publish(m)

    def report(self, out):
        self.resolve()
        out.mkdir(parents=True, exist_ok=True)
        lines = []
        if len(self.t) < 10:
            lines.append(f"метрических поз SLAM с эталоном: {len(self.t)} — мало для сверки "
                         "(SLAM не инициализировался или одометрия ещё не прошла min_motion для масштаба)")
        else:
            T, S, G, O = np.array(self.t), np.array(self.slam), np.array(self.gt), np.array(self.odom)
            path = np.sum(np.linalg.norm(np.diff(G[:, :2], axis=0), axis=1))
            e_s, A_s, _, _ = ate(S, G, with_scale=False)
            e_o, A_o, _, _ = ate(O, G, with_scale=False)
            _, _, k_s, _ = ate(S, G, with_scale=True)
            _, _, k_o, _ = ate(O, G, with_scale=True)
            gaps = np.diff(T)
            lost = gaps[gaps > 3.0 / self.fps]
            lines += [
                f"путь по эталону {path:.2f} м за {T[-1] - T[0]:.1f} с симуляции; метрических поз SLAM {len(T)}",
                f"SLAM (метры, масштаб по одометрии): {stats(e_s)}; масштаб карты {100 * (1 / k_s - 1):+.1f}%",
                f"одометрия ног без камеры:           {stats(e_o)}; её масштаб {100 * (1 / k_o - 1):+.1f}%",
                f"трекинг: разрывов дольше {3.0 / self.fps:.1f} с — {len(lost)} (сумма {lost.sum():.1f} с)",
            ]
            self.plot(G, A_s, A_o, out / "trajectory.png")
            # world <- map (только в симуляции): оба кадра выровнены по гравитации, поэтому только курс и
            # сдвиг — полный 3D-поворот по плоскому пути определяет наклон плохо
            mu_s, mu_g = S[:, :2].mean(0), G[:, :2].mean(0)
            U, _, Vt = np.linalg.svd((G[:, :2] - mu_g).T @ (S[:, :2] - mu_s))
            R2 = U @ np.diag([1.0, np.sign(np.linalg.det(U @ Vt))]) @ Vt
            Rw = np.eye(3)
            Rw[:2, :2] = R2
            tw = np.r_[mu_g - R2 @ mu_s, np.mean(G[:, 2] - S[:, 2])]
            (out / "map_world.json").write_text(json.dumps(
                {"what": "world = R @ map + t — связь кадра карты с эталоном симуляции (scripts/nav_goals.py)",
                 "R": Rw.tolist(), "t": tw.tolist()}, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
            np.savez_compressed(out / "poses.npz", t=T, slam=S, gt=G, odom=O)
        if len(self.raw_t) >= 10:
            e_r, _, k_r, _ = ate(np.array(self.raw), np.array(self.raw_gt), with_scale=True)
            lines.append(f"геометрия stella (подобие, {len(self.raw_t)} поз камеры): {stats(e_r)}; 1 ед. карты stella = {k_r:.3f} м")
        text = "\n".join(lines)
        (out / "report.txt").write_text(text + "\n", encoding="utf-8")
        print(text, flush=True)
        print(f"[eval] {out}/report.txt", flush=True)

    @staticmethod
    def plot(G, A_s, A_o, path, size=900, pad=60):
        allp = np.vstack([G, A_s, A_o])[:, :2]
        lo, hi = allp.min(0), allp.max(0)
        span = max(hi - lo) if max(hi - lo) > 0 else 1.0
        k = (size - 2 * pad) / span

        def px(P):  # x вправо, y вверх
            q = (P[:, :2] - lo) * k + pad
            return np.c_[q[:, 0], size - q[:, 1]].astype(np.int32)

        img = np.full((size, size, 3), 255, np.uint8)
        cv2.polylines(img, [px(A_o)], False, (200, 140, 60), 2, cv2.LINE_AA)
        cv2.polylines(img, [px(G)], False, (60, 160, 60), 3, cv2.LINE_AA)
        cv2.polylines(img, [px(A_s)], False, (30, 110, 230), 2, cv2.LINE_AA)
        cv2.circle(img, tuple(px(G[:1])[0]), 8, (0, 0, 0), 2, cv2.LINE_AA)
        bar = 10 ** np.floor(np.log10(span / 2))
        cv2.line(img, (pad, size - 25), (pad + int(bar * k), size - 25), (0, 0, 0), 2)
        cv2.putText(img, f"{bar:g} m", (pad, size - 32), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.putText(img, "green: ground truth  orange: SLAM (metric)  blue: leg odometry  circle: start", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1, cv2.LINE_AA)
        cv2.imwrite(str(path), img)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--out", required=True, help="папка отчёта")
    parser.add_argument("--fps", type=float, default=10.0, help="частота кадров панорамы, Гц (время симуляции)")
    parser.add_argument("--seconds", type=float, default=0.0, help="выйти через столько стенных секунд")
    args = parser.parse_args()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node("eval_slam", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    ev = Eval(node, args.fps)
    stop = []
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    t0 = time.time()
    while rclpy.ok() and not stop and (args.seconds <= 0 or time.time() - t0 < args.seconds):
        rclpy.spin_once(node, timeout_sec=0.1)
    ev.report(Path(args.out))
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
