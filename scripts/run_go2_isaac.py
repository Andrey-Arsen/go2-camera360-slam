#!/usr/bin/env python3
"""Isaac Sim: Go2 (IsaacLegs) с надстройкой и 360-камерой (два fisheye) — в окружении Isaac.

Открывает robot/go2_camera360/go2_camera360.usda (scripts/build_go2_camera360.py), добавляет
окружение с сервера ассетов, ставит робота в --spawn, включает мост ROS 2 и публикует
(scripts/ros2_cameras.py): /insta360/front|back/image_raw, /tf (world -> base_link),
/tf_static (base_link -> кадры камеры), /clock.
Графы IsaacLegs работают как есть: /joint_states, /joint_command, /imu, /odom — по ним
ходит контроллер политики (ros2 launch fullbody_controller go2.launch.py). Головной лидар и
камера IsaacLegs (/point_cloud, /scan, /rgb) по умолчанию выключены — в этом проекте только
360-камера, а рендер лишних датчиков тянет кадры вниз (--isaaclegs-sensors — оставить).

Обычно запускается из scripts/go2_sim.sh вместе с контроллером, телеуправлением и RViz.
Запуск отдельно: ~/isaacsim/python.sh scripts/run_go2_isaac.py [--env full_warehouse]
        [--spawn X Y YAW_DEG] [--image-size 960] [--image-every 6] [--headless] [--no-play]
"""

import argparse
import signal
import sys
import traceback
from pathlib import Path

from isaacsim import SimulationApp

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser()
parser.add_argument("--scene", default=str(ROOT / "robot" / "go2_camera360" / "go2_camera360.usda"))
parser.add_argument("--env", default="full_warehouse", help="окружение (см. go2_scene.ENVIRONMENTS) или путь к .usd")
parser.add_argument("--spawn", type=float, nargs=3, default=None, metavar=("X", "Y", "YAW"),
                    help="где поставить робота, м и градусы (по умолчанию — где стоит в сцене, в начале координат)")
parser.add_argument("--image-size", type=int, default=960, help="сторона кадра fisheye для ROS, пикселей; 0 — без камер")
parser.add_argument("--image-every", type=int, default=6, help="публиковать каждый N-й кадр рендера (6 -> 10 Гц)")
parser.add_argument("--lenses", choices=("both", "front", "back"), default="both", help="какие объективы публиковать")
parser.add_argument("--isaaclegs-sensors", action="store_true", help="оставить головной лидар и камеру IsaacLegs")
parser.add_argument("--gt-frame", default="", metavar="ИМЯ",
                    help="эталонную позу робота публиковать кадром world -> ИМЯ (например base_link_gt), "
                         "а не world -> base_link: для SLAM, где base_link — от одометрии (odom -> base_link)")
parser.add_argument("--robot-mask", metavar="ПАПКА",
                    help="копить маску тела робота на кадрах объективов (сегментация, всё под /World/go2) и "
                         "записать ПАПКА/robot_mask_<объектив>.png (255 — робот); см. scripts/make_robot_mask.sh")
parser.add_argument("--headless", action="store_true")
parser.add_argument("--no-play", action="store_true", help="не запускать симуляцию сразу (Play в окне)")
parser.add_argument("--seconds", type=float, default=0.0, help="выйти через столько секунд симуляции (для проверки)")
args, _ = parser.parse_known_args()

app = SimulationApp({"headless": args.headless, "width": 1280, "height": 720, "sync_loads": True,
                     "renderer": "RaytracedLighting"})

# Остановка по Ctrl+C / SIGTERM — флагом, который цикл проверяет между кадрами: исключение
# из обработчика могло бы прилететь внутрь чужого колбэка Kit и потеряться. Фоновым процессам
# оболочка выставляет SIGINT «игнорировать» — возвращаем свой.
STOP = []
signal.signal(signal.SIGINT, lambda *_: STOP.append(1))
signal.signal(signal.SIGTERM, lambda *_: STOP.append(1))

import time  # noqa: E402

import numpy as np  # noqa: E402
import omni.timeline  # noqa: E402
from isaacsim.core.utils.extensions import enable_extension  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
enable_extension("isaacsim.ros2.bridge")
app.update()
import go2_scene  # noqa: E402
import ros2_cameras  # noqa: E402


class RobotMask:
    """Объединение пикселей тела робота (всё под /World/go2: корпус, ноги, надстройка, корпус X5)
    на кадрах объективов за всю поездку — маска, где SLAM не должен искать точки."""

    def __init__(self, products, names, out_dir, every_n_frames):
        import omni.replicator.core as rep
        self.out = Path(out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.annot = {}
        for path, rp in products.items():
            a = rep.AnnotatorRegistry.get_annotator("instance_id_segmentation", init_params={"colorize": False})
            a.attach([rp])
            self.annot[names[path]] = a
        self.every, self.k = every_n_frames, 0
        self.union, self.frames, self.last_save = {}, 0, time.time()
        print(f"[go2] маска робота: копится на {', '.join(self.annot)} -> {self.out}", flush=True)

    def sample(self):
        self.k += 1
        if self.k % self.every:
            return
        for name, a in self.annot.items():
            s = a.get_data()
            if not s or "data" not in s or np.asarray(s["data"]).size == 0:
                return
            ids = [int(i) for i, p in s["info"]["idToLabels"].items() if str(p).startswith(go2_scene.ROBOT)]
            robot = np.isin(np.asarray(s["data"]), ids)
            self.union[name] = robot if name not in self.union else (self.union[name] | robot)
        self.frames += 1
        if time.time() - self.last_save > 10.0:
            self.save()

    def save(self):
        from PIL import Image
        self.last_save = time.time()
        for name, m in self.union.items():
            Image.fromarray(np.where(m, 255, 0).astype(np.uint8)).save(self.out / f"robot_mask_{name}.png")
        if self.union:
            share = ", ".join(f"{n} {100.0 * m.mean():.1f}%" for n, m in self.union.items())
            print(f"[go2] маска робота: кадров {self.frames}, тело занимает {share} кадра", flush=True)


def main():
    stage, _ = go2_scene.open_scene(app, args.env, args.scene)
    go2_scene.set_spawn(stage, args.spawn)
    app.update()
    if not args.isaaclegs_sensors:
        go2_scene.disable_graphs(["/Graphs/ROS_LidarRTX", "/Graphs/ROS_Camera"])

    names = ("front", "back") if args.lenses == "both" else (args.lenses,)
    lenses = {go2_scene.LENSES[n]: (f"insta360/{n}/image_raw", f"insta360_{n}_optical") for n in names}
    if args.image_size <= 0:
        lenses = {}  # камеры в сцене есть, но не рендерятся и не публикуются
    camera_frames = {f"{go2_scene.FRAMES}/{f}": f for f in go2_scene.CAMERA_FRAMES}
    products = ros2_cameras.start(stage, lenses, go2_scene.BASE, camera_frames,
                                  image_size=args.image_size, image_every_n_frames=args.image_every,
                                  gt_frame=args.gt_frame or None)
    mask = None
    if args.robot_mask:
        if not products:
            raise SystemExit("--robot-mask без камер (--image-size 0) не получится")
        names = {go2_scene.LENSES[n]: n for n in go2_scene.LENSES}
        mask = RobotMask(products, names, args.robot_mask, args.image_every)

    timeline = omni.timeline.get_timeline_interface()
    if not args.no_play:
        timeline.play()
    print("[go2] готово: симуляция идёт" if not args.no_play else "[go2] готово: нажмите Play", flush=True)
    print("[go2] ходить: ros2 launch fullbody_controller go2.launch.py; руль: scripts/teleop_wasd.py", flush=True)
    t_end = args.seconds if args.seconds > 0 else None
    try:
        while app.is_running() and not STOP:
            app.update()
            if mask is not None and timeline.is_playing():
                mask.sample()
            if t_end is not None and timeline.get_current_time() >= t_end:
                break
    finally:
        if mask is not None:
            mask.save()
    timeline.stop()


try:
    main()
except SystemExit as e:
    print(f"[go2] {e}", flush=True)
except KeyboardInterrupt:
    print("[go2] остановлено", flush=True)
except BaseException:
    traceback.print_exc()
    sys.stderr.flush()
finally:
    ros2_cameras.stop()
    app.close()
