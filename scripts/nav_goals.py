#!/usr/bin/env python3
"""Цели для Nav2 по очереди (действие NavigateToPose) и отчёт: дошёл ли робот и куда на самом деле.

Цели — в кадре map: «x,y» или «x,y,курс_град» через пробел, или --from-run ПАПКА: точки, где робот
был при картировании (poses.npz прогона — заведомо проходимые места): самая дальняя от старта,
дальняя от неё, ещё одна — и обратно к старту.
По каждой цели: результат Nav2, где робот остановился по SLAM (/slam/pose) и — в симуляции, если
в папке карты есть map_world.json (его пишет scripts/eval_slam.py) — по эталону (TF world ->
base_link_gt), то есть настоящая ошибка прихода.

Запуск (при работающем scripts/slam_sim.sh с NAV=1):
  source /opt/ros/humble/setup.bash && python3 scripts/nav_goals.py 2.0,1.0 0,0,180
  python3 scripts/nav_goals.py --from-run out/slam/<прогон> [--out out/slam/<прогон-навигации>]
"""

import argparse
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.parameter import Parameter
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener

STATUS = {GoalStatus.STATUS_SUCCEEDED: "дошёл", GoalStatus.STATUS_ABORTED: "не смог (прервано)",
          GoalStatus.STATUS_CANCELED: "отменено"}


def load_grid(yaml_path):
    """map.yaml + map.pgm (формат map_server) -> (сетка: True — занято, начало (x0, y0), клетка м)."""
    y = dict(line.split(":", 1) for line in Path(yaml_path).read_text(encoding="utf-8").splitlines() if ":" in line)
    res = float(y["resolution"])
    origin = np.array([float(v) for v in y["origin"].strip(" []").split(",")[:2]])
    raw = (Path(yaml_path).parent / y["image"].strip()).read_bytes().split(b"\n", 3)
    w, h = (int(v) for v in raw[1].split())
    img = np.frombuffer(raw[3], np.uint8)[: w * h].reshape(h, w)[::-1]
    return img <= 50, origin, res


def goals_from_run(run, n=3, clearance=0.9):
    """Точки пути картирования, не ближе clearance м к препятствиям карты: дальняя от старта, дальняя
    от неё, ещё одна — и обратно к старту. Курс — как ехал робот в этой точке."""
    d = np.load(Path(run) / "poses.npz")
    P = d["slam"][:, :2]
    ok = np.ones(len(P), bool)
    if (Path(run) / "map.yaml").exists():
        occ, origin, res = load_grid(Path(run) / "map.yaml")
        free = (~occ).astype(np.uint8)
        dist = cv2.distanceTransform(free, cv2.DIST_L2, 5) * res  # до ближайшей занятой клетки, м
        ij = np.floor((P - origin) / res).astype(int)
        inside = (ij[:, 0] >= 0) & (ij[:, 0] < occ.shape[1]) & (ij[:, 1] >= 0) & (ij[:, 1] < occ.shape[0])
        ok &= inside
        ok[inside] &= dist[ij[inside, 1], ij[inside, 0]] >= clearance
    C = P[ok] if ok.any() else P
    start = P[0]
    picked = [start]
    for _ in range(n):  # дальняя точка от уже выбранных
        far = np.min(np.linalg.norm(C[:, None, :] - np.array(picked)[None], axis=2), axis=1)
        picked.append(C[int(np.argmax(far))])
    goals = []
    for p in picked[1:] + [start]:
        i = int(np.argmin(np.linalg.norm(P - p, axis=1)))
        a, b = P[max(i - 5, 0)], P[min(i + 5, len(P) - 1)]
        yaw = math.degrees(math.atan2(b[1] - a[1], b[0] - a[0])) if np.linalg.norm(b - a) > 1e-3 else 0.0
        goals.append((float(p[0]), float(p[1]), yaw))
    return goals


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("goals", nargs="*", help="x,y или x,y,курс_град в кадре map")
    parser.add_argument("--from-run", help="взять цели из прогона картирования (poses.npz) и его map_world.json")
    parser.add_argument("--map-world", help="map_world.json для ошибки по эталону (по умолчанию — из --from-run)")
    parser.add_argument("--timeout", type=float, default=180.0, help="на одну цель, с симуляции")
    parser.add_argument("--out", help="папка для nav_report.txt")
    args = parser.parse_args()
    goals = [tuple(float(v) for v in g.split(",")) for g in args.goals]
    goals = [(g[0], g[1], g[2] if len(g) > 2 else 0.0) for g in goals]
    if args.from_run:
        goals += goals_from_run(args.from_run)
    mw_path = args.map_world or (str(Path(args.from_run) / "map_world.json") if args.from_run else None)
    map_world = None
    if mw_path and Path(mw_path).exists():
        d = json.loads(Path(mw_path).read_text(encoding="utf-8"))
        map_world = (np.array(d["R"]), np.array(d["t"]))
    if not goals:
        raise SystemExit("нет целей: дайте x,y[,курс] или --from-run")

    rclpy.init()
    node = rclpy.create_node("nav_goals", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    tf = Buffer()
    TransformListener(tf, node)
    last = {}
    node.create_subscription(Odometry, "/slam/pose", lambda m: last.__setitem__("pose", m), 10)
    client = ActionClient(node, NavigateToPose, "navigate_to_pose")
    log = []

    def say(text):
        print(text, flush=True)
        log.append(text)

    say(f"целей: {len(goals)}; жду, пока Nav2 станет активным...")
    # сервер действия появляется раньше, чем bt_navigator активен, — до этого цели отклоняются
    state = node.create_client(GetState, "bt_navigator/get_state")
    t_wait = time.time()
    while True:
        if state.wait_for_service(timeout_sec=1.0):
            fut = state.call_async(GetState.Request())
            rclpy.spin_until_future_complete(node, fut, timeout_sec=2.0)
            if fut.result() is not None and fut.result().current_state.label == "active":
                break
        if time.time() - t_wait > 600:
            raise SystemExit("Nav2 не стал активным за 10 минут — см. out/go2_logs/nav2.log")
        rclpy.spin_once(node, timeout_sec=1.0)
    client.wait_for_server()
    say(f"Nav2 активен (ждал {time.time() - t_wait:.0f} с)")
    errors = []
    for k, (x, y, yaw) in enumerate(goals, 1):
        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = "map"
        ps.header.stamp = node.get_clock().now().to_msg()
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.z, ps.pose.orientation.w = math.sin(math.radians(yaw) / 2), math.cos(math.radians(yaw) / 2)
        goal.pose = ps
        for _ in range(10):  # отклонённую цель — ещё раз (навигатор мог быть занят отменой прошлой)
            t_start = node.get_clock().now()
            fut = client.send_goal_async(goal)
            rclpy.spin_until_future_complete(node, fut)
            handle = fut.result()
            if handle.accepted:
                break
            time.sleep(2.0)
        if not handle.accepted:
            say(f"цель {k} ({x:+.2f}, {y:+.2f}): Nav2 не принял")
            continue
        res = handle.get_result_async()
        while not res.done():
            rclpy.spin_once(node, timeout_sec=0.1)
            if (node.get_clock().now() - t_start).nanoseconds * 1e-9 > args.timeout:
                handle.cancel_goal_async()
                break
        for _ in range(20):  # дать дойти последней позе
            rclpy.spin_once(node, timeout_sec=0.05)
        status = res.result().status if res.done() else -1
        dur = (node.get_clock().now() - t_start).nanoseconds * 1e-9
        line = f"цель {k} ({x:+.2f}, {y:+.2f}, {yaw:+.0f}°): {STATUS.get(status, 'время вышло')} за {dur:.0f} с"
        if "pose" in last:
            p = last["pose"].pose.pose.position
            line += f"; по SLAM в {100 * math.hypot(p.x - x, p.y - y):.0f} см от цели"
        if map_world is not None:
            try:
                g = tf.lookup_transform("world", "base_link_gt", Time()).transform.translation
                R, t = map_world
                gm = R.T @ (np.array([g.x, g.y, g.z]) - t)  # эталон -> кадр карты
                e = math.hypot(gm[0] - x, gm[1] - y)
                errors.append(e)
                line += f", по эталону — в {100 * e:.0f} см"
            except Exception as ex:  # noqa: BLE001
                line += f" (эталона нет: {ex})"
        say(line)
    if errors:
        say(f"по эталону: средняя ошибка прихода {100 * np.mean(errors):.0f} см, максимальная {100 * max(errors):.0f} см")
    if args.out:
        Path(args.out).mkdir(parents=True, exist_ok=True)
        (Path(args.out) / "nav_report.txt").write_text("\n".join(log) + "\n", encoding="utf-8")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
