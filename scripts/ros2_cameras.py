"""Публикация 360-камеры Go2 в ROS 2: кадры обоих fisheye, TF и /clock.

Топики (имена — как будут у драйвера настоящей Insta360, чтобы потом подменить симуляцию железом):
  /insta360/front/image_raw    sensor_msgs/Image rgb8, кадр insta360_front_optical
  /insta360/back/image_raw     sensor_msgs/Image rgb8, кадр insta360_back_optical
  /tf                          world -> base_link: эталонная поза робота из симуляции; с gt_frame —
                               world -> gt_frame (base_link тогда свободен для odom -> base_link)
  /tf_static                   base_link -> insta360_link, insta360_front, insta360_back и их
                               *_optical (оптические ROS: Z из объектива, X вправо, Y вниз) — крепление;
                               повторяется раз в секунду симуляции (transient local, как у tf2)
  /clock                       время симуляции (все узлы — с use_sim_time)

Объектив — равноугольный fisheye 200°: пиксель на расстоянии r от центра кадра смотрит под углом
theta = r / (size/2) * 100° от оси объектива; круг изображения вписан в квадратный кадр.
Оси картинки совпадают с оптическим кадром: u — вправо (+X), v — вниз (+Y).

Домен ROS берётся из ROS_DOMAIN_ID окружения, в котором запущен Isaac Sim.
Из своего скрипта: ros2_cameras.start(stage, ...); остановить — stop().
"""

import omni.graph.core as og
import omni.replicator.core as rep
import omni.syntheticdata
import omni.syntheticdata._syntheticdata as sd
import omni.usd
from isaacsim.core.utils.extensions import enable_extension
from pxr import Sdf, Usd, UsdGeom

GRAPH = "/ROS2CamerasGraph"

_writers = []
_render_products = []


def _set_frame_names(stage, frame_names):
    # Сессионный слой: имена кадров нужны только ROS, в файл сцены не попадут.
    with Usd.EditContext(stage, stage.GetSessionLayer()):
        for path, name in frame_names.items():
            prim = stage.GetPrimAtPath(path)
            if not prim.IsValid():
                raise RuntimeError(f"[ros2_cameras] нет прима {path} для кадра {name}")
            prim.CreateAttribute("isaac:nameOverride", Sdf.ValueTypeNames.String).Set(name)


def _build_tf_clock_graph(world, base, gt_target, camera_frames, static_every_n_frames=60):
    """/clock и поза робота — каждый кадр; кадры камеры относительно корпуса — в /tf_static раз в
    static_every_n_frames кадров. Не «один раз»: узел TF при первом вызове только создаёт издателя."""
    keys = og.Controller.Keys
    og.Controller.edit(
        {"graph_path": GRAPH, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("tick", "omni.graph.action.OnPlaybackTick"),
                ("static_gate", "isaacsim.core.nodes.IsaacSimulationGate"),
                ("sim_time", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("context", "isaacsim.ros2.bridge.ROS2Context"),
                ("clock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("tf_robot", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
                ("tf_camera", "isaacsim.ros2.bridge.ROS2PublishTransformTree"),
            ],
            keys.CONNECT: [
                ("tick.outputs:tick", "clock.inputs:execIn"),
                ("tick.outputs:tick", "tf_robot.inputs:execIn"),
                ("tick.outputs:tick", "static_gate.inputs:execIn"),
                ("static_gate.outputs:execOut", "tf_camera.inputs:execIn"),
                ("sim_time.outputs:simulationTime", "clock.inputs:timeStamp"),
                ("sim_time.outputs:simulationTime", "tf_robot.inputs:timeStamp"),
                ("sim_time.outputs:simulationTime", "tf_camera.inputs:timeStamp"),
                ("context.outputs:context", "clock.inputs:context"),
                ("context.outputs:context", "tf_robot.inputs:context"),
                ("context.outputs:context", "tf_camera.inputs:context"),
            ],
            keys.SET_VALUES: [
                ("static_gate.inputs:step", static_every_n_frames),
                ("tf_robot.inputs:parentPrim", [Sdf.Path(world)]),
                ("tf_robot.inputs:targetPrims", [Sdf.Path(gt_target)]),
                ("tf_camera.inputs:parentPrim", [Sdf.Path(base)]),
                ("tf_camera.inputs:targetPrims", [Sdf.Path(p) for p in camera_frames]),
                ("tf_camera.inputs:staticPublisher", True),  # меняет только QoS (как StaticTransformBroadcaster)
                ("tf_camera.inputs:topicName", "tf_static"),  # тему — самим
            ],
        },
    )


def start(stage, lenses, base, camera_frames, world="/World", image_size=960, image_every_n_frames=6, gt_frame=None):
    """Включает публикацию.

    lenses — {путь камеры USD: (топик, frame_id картинки)}; base — прим корпуса (кадр base_link);
    camera_frames — {путь прима: имя кадра} для /tf_static от base_link. Кадры камер: image_size px,
    каждый N-й кадр рендера (6 -> 10 Гц при 60). gt_frame — публиковать эталонную позу корпуса
    кадром с этим именем (метка под корпусом), а не base_link. Возвращает {путь камеры: render product}."""
    stop()
    enable_extension("isaacsim.ros2.bridge")
    names = {world: "world", base: "base_link", **camera_frames}
    gt_target = base
    if gt_frame:  # пустой прим под корпусом: его поза — поза корпуса, а имя кадра своё
        gt_target = f"{base}/gt_marker"
        with Usd.EditContext(stage, stage.GetSessionLayer()):
            UsdGeom.Xform.Define(stage, gt_target)
        names[gt_target] = gt_frame
    _set_frame_names(stage, names)
    _build_tf_clock_graph(world, base, gt_target, list(camera_frames))

    rv = omni.syntheticdata.SyntheticData.convert_sensor_type_to_rendervar(sd.SensorType.Rgb.name)
    products = {}
    for path, (topic, frame) in lenses.items():
        rp = rep.create.render_product(path, (image_size, image_size), name=f"{topic.split('/')[1]}_ROS2")
        writer = rep.writers.get(f"{rv}ROS2PublishImage")
        writer.initialize(topicName=topic, frameId=frame, queueSize=1)
        writer.attach([rp])
        gate = omni.syntheticdata.SyntheticData._get_node_path(rv + "IsaacSimulationGate", rp.path)
        og.Controller.attribute(gate + ".inputs:step").set(image_every_n_frames)
        _writers.append(writer)
        _render_products.append(rp)
        products[path] = rp
    print(f"[ros2_cameras] публикую {', '.join('/' + t for t, _ in lenses.values())} "
          f"({image_size} px, каждый {image_every_n_frames}-й кадр), /tf, /tf_static, /clock", flush=True)
    return products


def stop():
    # Сначала отсоединить всё, потом уничтожать render product.
    while _writers:
        _writers.pop().detach()
    while _render_products:
        _render_products.pop().destroy()
    stage = omni.usd.get_context().get_stage()
    if stage and stage.GetPrimAtPath(GRAPH).IsValid():
        stage.RemovePrim(GRAPH)
