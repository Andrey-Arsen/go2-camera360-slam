#!/usr/bin/env python3
"""Езда Go2 по программе: /cmd_vel по шагам во времени симуляции — без человека, повторяемо.

Программы:
  mask   стоять, вперёд, повороты, вбок, назад — ноги проходят все положения (scripts/make_robot_mask.sh)
  slam   стоять 4 с (инициализация SLAM), 3 м вперёд, разворот на 180°, обратно, разворот —
         и ещё круг: «туда-обратно» с разворотами на месте, конец рядом с началом
  long   стоять 4 с, три раза «5 м туда — разворот — 5 м обратно» с разворотом 0.6 рад/с (как при
         навигации) — около 15 м: на таком пути ошибки одометрии ног успевают накопиться
Шаг программы: (секунд симуляции, вперёд м/с, вбок м/с, поворот рад/с). Публикует 20 Гц по
времени симуляции (/clock), в конце — стоп. Ctrl+C — стоп и выход.

Запуск (при работающей симуляции и контроллере ходьбы):
  source /opt/ros/humble/setup.bash && python3 scripts/drive_pattern.py slam
"""

import argparse
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.parameter import Parameter

TURN = 1.0  # рад/с — политика училась до ±1
PATTERNS = {
    "mask": [(4, 0, 0, 0), (4, 0.6, 0, 0), (3, 0, 0, TURN), (3, 0, 0.3, 0), (3, -0.4, 0, 0),
             (3, 0, 0, -TURN), (3, 0, -0.3, 0), (3, 0.8, 0, 0.5), (2, 0, 0, 0)],
    "slam": [(4, 0, 0, 0)] + 2 * [(6, 0.5, 0, 0), (1, 0, 0, 0), (math.pi / TURN, 0, 0, TURN), (1, 0, 0, 0)] + [(2, 0, 0, 0)],
    "long": [(4, 0, 0, 0)] + 6 * [(10, 0.5, 0, 0), (1, 0, 0, 0), (math.pi / 0.6, 0, 0, 0.6), (1, 0, 0, 0)] + [(2, 0, 0, 0)],
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pattern", choices=sorted(PATTERNS))
    parser.add_argument("--topic", default="cmd_vel")
    args = parser.parse_args()
    steps = PATTERNS[args.pattern]
    rclpy.init()
    node = rclpy.create_node("drive_pattern", parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)])
    pub = node.create_publisher(Twist, args.topic, 10)
    total = sum(s[0] for s in steps)
    state = {"t0": None, "done": False, "step": -1}

    def tick():
        now = node.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return  # /clock ещё не пришёл
        if state["t0"] is None:
            state["t0"] = now
            node.get_logger().info(f"программа {args.pattern}: {len(steps)} шагов, {total:.0f} с симуляции")
        t = now - state["t0"]
        acc = 0.0
        cmd = (0.0, 0.0, 0.0)
        for i, (dur, vx, vy, wz) in enumerate(steps):
            if t < acc + dur:
                cmd = (vx, vy, wz)
                if i != state["step"]:
                    state["step"] = i
                    node.get_logger().info(f"шаг {i + 1}/{len(steps)}: вперёд {vx:+.2f} вбок {vy:+.2f} поворот {wz:+.2f}")
                break
            acc += dur
        else:
            state["done"] = True
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(cmd[0]), float(cmd[1]), float(cmd[2])
        pub.publish(msg)

    node.create_timer(0.05, tick)
    try:
        while rclpy.ok() and not state["done"]:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        pub.publish(Twist())  # стоп: у контроллера нет таймаута cmd_vel
        node.get_logger().info("программа закончена — стоп")
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
