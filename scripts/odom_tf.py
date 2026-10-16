#!/usr/bin/env python3
"""Одометрия ног -> TF odom -> base_link, как её даст настоящий Go2 (по желанию — с его ошибками).

Слушает /odom (nav_msgs/Odometry: в симуляции — IsaacLegs, по x/y это почти эталон; на роботе —
оценка состояния Unitree) и /imu (крен и тангаж — по гравитации, они у IMU точные) и публикует
TF odom -> base_link с временем сообщения одометрии:
  x, y, курс — шагами из /odom; с ошибками настоящей одометрии ног: каждый шаг длиннее на
    scale_error (0.03 = +3%), курс уходит на yaw_drift_deg_per_m градусов на метр пути;
  крен, тангаж — из ориентации IMU (у /odom IsaacLegs тангаж сдвинут на ~3.5°, а z неверный);
  z — base_height (высота корпуса на ходу), чтобы odom стоял на полу, как у колёсных роботов.
Ошибки по умолчанию 0; SLAM-лаунчер задаёт их переменной ODOM_NOISE="scale_error,yaw_drift".

Запуск: source /opt/ros/humble/setup.bash && python3 scripts/odom_tf.py --ros-args -p use_sim_time:=true
        [-p scale_error:=0.03 -p yaw_drift_deg_per_m:=0.5]
"""

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def roll_pitch_of(q):
    roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
    return roll, pitch


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)


class OdomTf(Node):
    def __init__(self):
        super().__init__("odom_tf")
        self.scale_error = self.declare_parameter("scale_error", 0.0).value
        self.yaw_drift = math.radians(self.declare_parameter("yaw_drift_deg_per_m", 0.0).value)
        self.base_height = self.declare_parameter("base_height", 0.30).value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.tf = TransformBroadcaster(self)
        self.prev = None  # (x, y, yaw) чистые
        self.x = self.y = self.yaw = 0.0  # с ошибками
        self.path = 0.0
        self.roll = self.pitch = 0.0
        self.create_subscription(Imu, "imu", self.on_imu, 50)
        self.create_subscription(Odometry, "odom", self.on_odom, 50)
        self.get_logger().info(f"odom -> base_link из /odom; ошибки: масштаб {100 * self.scale_error:+.1f}%, "
                               f"курс {math.degrees(self.yaw_drift):.2f}°/м")

    def on_imu(self, m):
        q = m.orientation
        if abs(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w - 1.0) < 1e-3:
            self.roll, self.pitch = roll_pitch_of(q)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        x, y, yaw = p.x, p.y, yaw_of(q)
        if self.prev is None:
            self.x, self.y, self.yaw = x, y, yaw
        else:
            px, py, pyaw = self.prev
            # шаг в кадре корпуса на прошлом шаге — так ошибки копятся, как у настоящей одометрии
            c, s = math.cos(pyaw), math.sin(pyaw)
            fx, fy = c * (x - px) + s * (y - py), -s * (x - px) + c * (y - py)
            step = math.hypot(fx, fy)
            dyaw = math.atan2(math.sin(yaw - pyaw), math.cos(yaw - pyaw))
            k = 1.0 + self.scale_error
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            self.x += k * (c * fx - s * fy)
            self.y += k * (s * fx + c * fy)
            self.yaw += dyaw + self.yaw_drift * step
            self.path += step
        self.prev = (x, y, yaw)
        t = TransformStamped()
        t.header.stamp = m.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = self.x, self.y, self.base_height
        qx, qy, qz, qw = quat_from_rpy(self.roll, self.pitch, self.yaw)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
        self.tf.sendTransform(t)


def main():
    rclpy.init()
    node = OdomTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
