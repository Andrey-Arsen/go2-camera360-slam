#!/usr/bin/env python3
"""Включение узлов Nav2 (configure -> activate) с проверкой состояния — вместо nav2_lifecycle_manager.

Зачем: FastDDS в Humble отдаёт ответ сервиса, только если за 100 мс увидел читателя ответа у клиента;
под нагрузкой (окна Isaac, RViz) не успевает — «failed to send response to /planner_server/change_state
(timeout)», и стандартный менеджер ждёт этот ответ вечно, Nav2 не включается. Здесь после каждого
перехода состояние узла проверяется запросом get_state (с повторами), а непрошедший переход
повторяется. Активация карт стоимости ждёт TF map -> odom — при картировании это после первого
метра пути, поэтому ждём сколько нужно.

Запуск (после config/nav2.launch.py): python3 scripts/nav2_lifecycle.py
"""

import sys
import time

import rclpy
from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState, GetState

NODES = ["controller_server", "smoother_server", "planner_server", "behavior_server",
         "bt_navigator", "waypoint_follower", "velocity_smoother"]


class Bringup:
    def __init__(self, node):
        self.node = node
        self.get = {n: node.create_client(GetState, f"/{n}/get_state") for n in NODES}
        self.change = {n: node.create_client(ChangeState, f"/{n}/change_state") for n in NODES}

    def _call(self, client, request, timeout):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def state(self, name):
        r = self._call(self.get[name], GetState.Request(), 2.0)
        return r.current_state.label if r is not None else None

    def bring(self, name, transition_id, target, wait_s):
        """Перевести узел в target: запрос перехода, потом опрос состояния; повтор, если не сменилось."""
        t0 = time.time()
        while time.time() - t0 < wait_s:
            s = self.state(name)
            if s == target:
                return True
            if s in ("unconfigured", "inactive"):  # не в переходе — просим ещё раз
                req = ChangeState.Request()
                req.transition.id = transition_id
                self._call(self.change[name], req, 5.0)
            time.sleep(1.0)
        return False


def main():
    rclpy.init()
    node = rclpy.create_node("nav2_lifecycle")
    b = Bringup(node)
    print("[nav2] жду узлы Nav2...", flush=True)
    for name in NODES:
        while b.state(name) is None:
            time.sleep(1.0)
    for name in NODES:
        if not b.bring(name, Transition.TRANSITION_CONFIGURE, "inactive", 120):
            print(f"[nav2] {name} не настроился — см. out/go2_logs/nav2.log", flush=True)
            sys.exit(1)
    print("[nav2] узлы настроены; включаю (карты стоимости ждут TF map -> odom)", flush=True)
    for name in NODES:
        if not b.bring(name, Transition.TRANSITION_ACTIVATE, "active", 3600):
            print(f"[nav2] {name} не включился за час — нет TF map -> odom?", flush=True)
            sys.exit(1)
        print(f"[nav2] {name} активен", flush=True)
    print("[nav2] Nav2 активен — цели принимаются", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
