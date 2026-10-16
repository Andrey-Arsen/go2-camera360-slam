"""Узлы Nav2 для Go2 с 360-камерой — те же, что nav2_bringup navigation_launch.py (Humble), но без
nav2_lifecycle_manager: включает их scripts/nav2_lifecycle.py.

Зачем: FastDDS в Humble под нагрузкой теряет ответ сервиса смены состояния («failed to send response
to /planner_server/change_state (timeout)») — стандартный менеджер ждёт его вечно, и Nav2 не
включается (было и с задержкой старта менеджера на 8 с). nav2_lifecycle.py после каждого перехода
проверяет состояние узла и повторяет непрошедший. Параметры — config/nav2_params.yaml; cmd_vel:
контроллер -> cmd_vel_nav -> velocity_smoother -> /cmd_vel (как в nav2_bringup).

Запуск: ros2 launch config/nav2.launch.py [params_file:=...]; затем python3 scripts/nav2_lifecycle.py
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml

NODES = [  # (пакет, исполняемый файл, имя, свои переназначения); порядок — как включает nav2_lifecycle.py
    ("nav2_controller", "controller_server", "controller_server", [("cmd_vel", "cmd_vel_nav")]),
    ("nav2_smoother", "smoother_server", "smoother_server", []),
    ("nav2_planner", "planner_server", "planner_server", []),
    ("nav2_behaviors", "behavior_server", "behavior_server", []),
    ("nav2_bt_navigator", "bt_navigator", "bt_navigator", []),
    ("nav2_waypoint_follower", "waypoint_follower", "waypoint_follower", []),
    ("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother",
     [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),
]


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = ParameterFile(RewrittenYaml(source_file=params_file, root_key="",
                                         param_rewrites={"use_sim_time": use_sim_time},
                                         convert_types=True), allow_substs=True)
    remap = [("/tf", "tf"), ("/tf_static", "tf_static")]
    nodes = [Node(package=pkg, executable=exe, name=name, output="screen", parameters=[params],
                  remappings=remap + extra) for pkg, exe, name, extra in NODES]
    return LaunchDescription([
        SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
        DeclareLaunchArgument("params_file", default_value=str(Path(__file__).resolve().parent / "nav2_params.yaml")),
        DeclareLaunchArgument("use_sim_time", default_value="True"),
        *nodes,
    ])
