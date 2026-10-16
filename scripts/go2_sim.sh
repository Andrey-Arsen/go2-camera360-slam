#!/usr/bin/env bash
# Go2 с 360-камерой (два fisheye на надстройке) в Isaac Sim: ходьба политикой IsaacLegs,
# кадры камеры и TF в ROS 2, RViz; рулить WASD в этом терминале.
#   scripts/go2_sim.sh                                  # Full Warehouse
#   scripts/go2_sim.sh digital_twin_warehouse           # другое окружение (см. run_go2_isaac.py --help)
#   scripts/go2_sim.sh full_warehouse --spawn 2 -3 90   # где поставить робота: X Y угол
#   scripts/go2_sim.sh full_warehouse --image-size 480  # камеры мельче — симуляция быстрее
# SLAM (stella_vslam по 360-панораме) — scripts/slam_sim.sh (то же, что SLAM=1 здесь):
#   склейка двух fisheye (scripts/equirect.py) -> stella_vslam (окно карты) -> мост в метры и TF
#   map -> odom (scripts/slam_bridge.py, масштаб — по одометрии ног scripts/odom_tf.py) -> сверка с
#   эталоном (scripts/eval_slam.py); результаты — out/slam/<прогон>/ (report.txt, trajectory.png,
#   map.msg + map_transform.json — карта для локализации: MAP=out/slam/<прогон> scripts/slam_sim.sh).
# Переменные: WITH_RVIZ=0 — без RViz; DRIVE=slam|mask — ехать по программе (scripts/drive_pattern.py)
#   вместо WASD и выйти в конце; для SLAM: LENSES=front|back — один объектив, VIEWER=none — без окна
#   карты, SLAM_RUN=имя — папка прогона, MAP=папка — локализация по готовой карте (без картирования),
#   ODOM_NOISE="масштаб,курс" — ошибки одометрии ног (по умолчанию 0.03,0.5: +3% и 0.5°/м; 0,0 — без);
#   NAV=1 — ещё и Nav2 (config/nav2_params.yaml; цели — «2D Goal Pose» в RViz), GOALS="x,y[,курс] ..." или
#   GOALS=auto (точки из прогона MAP) — проехать цели (scripts/nav_goals.py) и выйти.
# Нужно один раз: robot/go2_camera360 (scripts/build_go2_camera360.py), собранный IsaacLegs
# (third_party/IsaacLegs) и torch для контроллера (third_party/pydeps); для SLAM — stella_vslam
# (scripts/install_stella.sh) и маска робота (scripts/make_robot_mask.sh). См. README.
# Журналы — out/go2_logs/. Выход: Ctrl+C здесь (закроет всё) или закрыть окно Isaac Sim.
# Аргументы после окружения уходят в run_go2_isaac.py.
set -e
set -m  # каждый фоновый процесс — своя группа: Ctrl+C здесь их не задевает, гасим группой целиком
cd "$(dirname "$0")/.."
ROOT=$PWD
ENV_NAME=${1:-full_warehouse}
[ $# -gt 0 ] && shift
SLAM=${SLAM:-0}
for need in robot/go2_camera360/go2_camera360.usda third_party/IsaacLegs/install/setup.bash third_party/pydeps/torch; do
    [ -e "$need" ] || { echo "нет $need — см. README, раздел «Установка»"; exit 1; }
done
source scripts/ros_env.sh
# остатки прошлого запуска (уборку прервали вторым Ctrl+C, закрыли терминал) мешают: два контроллера
# ходьбы валят робота, две одометрии и две склейки сбивают SLAM
LEFT=$(scripts/stop_all.sh --list)
if [ -n "$LEFT" ]; then
    echo "в домене ROS ${ROS_DOMAIN_ID:-0} ещё работают процессы прошлого запуска:"; echo "$LEFT"
    echo "закройте их: scripts/stop_all.sh — и запустите снова"; exit 1
fi
source third_party/IsaacLegs/install/setup.bash
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$ROOT/third_party/pydeps"  # torch для контроллера, в конец — системные пакеты главнее
LOG=out/go2_logs
mkdir -p "$LOG"
if [ "$SLAM" != 0 ]; then
    source scripts/slam_env.sh
    [ -f "$STELLA_VOCAB" ] && ros2 pkg prefix stella_vslam_ros > /dev/null 2>&1 \
        || { echo "нет stella_vslam — сначала scripts/install_stella.sh"; exit 1; }
    [ -f config/robot_mask_front.png ] || echo "!! нет маски тела робота (scripts/make_robot_mask.sh) — SLAM будет видеть свой корпус"
    RUN=out/slam/${SLAM_RUN:-$(date +%Y%m%d_%H%M%S)}
    mkdir -p "$RUN"
    case " $* " in *" --headless "*) VIEWER=${VIEWER:-none} ;; esac
    VIEWER=${VIEWER:-pangolin_viewer}
    ODOM_NOISE=${ODOM_NOISE:-0.03,0.5}
    if [ -n "${MAP:-}" ]; then
        [ -f "$MAP/map.msg" ] && [ -f "$MAP/map_transform.json" ] \
            || { echo "в $MAP нет map.msg и map_transform.json — это должна быть папка прогона с картированием"; exit 1; }
    fi
    set -- "$@" --gt-frame base_link_gt  # эталон — отдельным кадром: base_link — от одометрии (odom -> base_link)
fi

# python.sh Isaac запускает python дочерним процессом — поэтому гасим всю группу задания.
# disown — чтобы bash не печатал «Terminated» про свои фоновые задания при выходе.
~/isaacsim/python.sh scripts/run_go2_isaac.py --env "$ENV_NAME" "$@" > "$LOG/isaac.log" 2>&1 &
ISAAC_PID=$!
disown
CTRL_PID="" RVIZ_PID="" EQ_PID="" ODOM_PID="" STELLA_PID="" BRIDGE_PID="" EVAL_PID="" GRID_PID="" NAV_PID="" NAVUP_PID=""
stop_group() {  # PID [сигнал] [секунд ждать]
    [ -n "$1" ] || return 0
    kill -"${2:-TERM}" -- "-$1" 2>/dev/null || return 0
    for _ in $(seq 1 "${3:-10}"); do kill -0 -- "-$1" 2>/dev/null || return 0; sleep 1; done
}
cleanup() {
    set +e  # уже закрытый процесс — не повод бросить уборку на полпути
    trap '' INT TERM HUP  # и повторный Ctrl+C тоже: иначе процессы остаются жить (см. scripts/stop_all.sh)
    echo; echo "закрываю всё (до минуты: stella пишет карту, сверка — отчёт), Ctrl+C больше не нужен..."
    # Isaac — первым и мягко: SIGTERM всей группе, ждём, пока из группы не выйдут все.
    stop_group "$ISAAC_PID" TERM 60
    # stella по Ctrl+C сохраняет карту, сверка — пишет отчёт: даём им время
    stop_group "$STELLA_PID" INT 60
    stop_group "$NAV_PID" INT 20
    stop_group "$BRIDGE_PID" INT 20
    stop_group "$GRID_PID" INT 20
    stop_group "$EVAL_PID" INT 120
    for pid in $NAVUP_PID $EQ_PID $ODOM_PID $CTRL_PID $RVIZ_PID; do kill -- "-$pid" 2>/dev/null; done
    sleep 1
    for pid in $NAVUP_PID $EQ_PID $ODOM_PID $STELLA_PID $BRIDGE_PID $GRID_PID $NAV_PID $EVAL_PID $CTRL_PID $RVIZ_PID $ISAAC_PID; do kill -9 -- "-$pid" 2>/dev/null; done
    if [ "$SLAM" != 0 ] && [ -f "$RUN/report.txt" ]; then
        echo; echo "SLAM, прогон $RUN:"; cat "$RUN/report.txt"
        [ -f "$RUN/map.msg" ] && echo "карта: $RUN/map.msg (+ map_transform.json, map.yaml); навигация по ней: MAP=$RUN NAV=1 scripts/slam_sim.sh"
        [ -f "$RUN/nav_report.txt" ] && { echo "навигация:"; cat "$RUN/nav_report.txt"; }
    fi
}
trap cleanup EXIT
echo "Isaac Sim грузится (ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}); окружение в первый раз скачивается несколько минут..."
until grep -q "\[go2\] готово" "$LOG/isaac.log"; do
    if ! kill -0 $ISAAC_PID 2>/dev/null || grep -q "^\[go2\] нет\|^\[go2\] не\|Traceback" "$LOG/isaac.log"; then
        echo "Isaac Sim не запустился:"; grep -E "^\[go2\]|Traceback|Error:" "$LOG/isaac.log" | tail -5
        echo "весь журнал: $LOG/isaac.log"; exit 1
    fi
    sleep 2
done
grep "^\[go2\]\|^\[ros2_cameras\]" "$LOG/isaac.log"

# один поток torch: под нагрузкой (окна, SLAM, Nav2) шаг политики со всеми потоками рос с 0.04 до
# 18–33 мс, команды суставам опаздывали, очередь joint_states (KEEP_ALL) росла — робот падал
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 ros2 launch fullbody_controller go2.launch.py > "$LOG/controller.log" 2>&1 &
CTRL_PID=$!
disown

if [ "$SLAM" != 0 ]; then
    python3 scripts/equirect.py --ros-args -p use_sim_time:=true -p lenses:="${LENSES:-both}" \
        -p mask_out:="$RUN/mask.png" > "$LOG/equirect.log" 2>&1 &
    EQ_PID=$!
    disown
    echo "Склейка панорамы ждёт кадры и TF..."
    for _ in $(seq 1 120); do
        [ -f "$RUN/mask.png" ] && break
        kill -0 $EQ_PID 2>/dev/null || { echo "склейка упала:"; tail -5 "$LOG/equirect.log"; exit 1; }
        sleep 1
    done
    [ -f "$RUN/mask.png" ] || { echo "склейка не дождалась кадров — см. $LOG/equirect.log"; exit 1; }
    grep -a -o "панорама .*\|маска для stella.*" "$LOG/equirect.log" | head -2
    python3 scripts/odom_tf.py --ros-args -p use_sim_time:=true -p scale_error:="${ODOM_NOISE%,*}" \
        -p yaw_drift_deg_per_m:="${ODOM_NOISE#*,}" > "$LOG/odom_tf.log" 2>&1 &
    ODOM_PID=$!
    disown
    if [ -n "${MAP:-}" ]; then  # локализация по готовой карте: карта не меняется, преобразование — её
        STELLA_MAP_ARGS=(--disable-mapping -i "$MAP/map.msg")
        BRIDGE_ARGS=(-p transform_in:="$MAP/map_transform.json")
    else
        STELLA_MAP_ARGS=(-o "$RUN/map.msg")
        BRIDGE_ARGS=(-p transform_out:="$RUN/map_transform.json")
    fi
    ros2 run stella_vslam_ros run_slam -v "$STELLA_VOCAB" -c config/stella_equirect.yaml --mask "$RUN/mask.png" \
        "${STELLA_MAP_ARGS[@]}" --eval-log-dir "$RUN" --viewer "$VIEWER" \
        --ros-args -p use_sim_time:=true -p publish_tf:=false -p map_frame:=slam_map -p landmarks_period:=1.0 \
        -r camera/image_raw:=/insta360/equirect/image_raw > "$LOG/stella.log" 2>&1 &
    STELLA_PID=$!
    disown
    python3 scripts/slam_bridge.py --ros-args -p use_sim_time:=true "${BRIDGE_ARGS[@]}" > "$LOG/slam_bridge.log" 2>&1 &
    BRIDGE_PID=$!
    disown
    if [ -n "${MAP:-}" ]; then GRID_ARGS=(-p load:="$MAP/map.yaml"); else GRID_ARGS=(-p save_dir:="$RUN"); fi
    python3 scripts/landmark_grid.py --ros-args -p use_sim_time:=true "${GRID_ARGS[@]}" > "$LOG/landmark_grid.log" 2>&1 &
    GRID_PID=$!
    disown
    python3 scripts/eval_slam.py --out "$RUN" > "$LOG/eval.log" 2>&1 &
    EVAL_PID=$!
    disown
    if [ "${NAV:-0}" != 0 ]; then
        # карты стоимости Nav2 при включении сами ждут TF map -> odom (при картировании он появляется
        # после первого метра пути — масштаб по одометрии), цели принимаются, когда Nav2 активен
        # config/nav2.launch.py — узлы nav2_bringup; включает их scripts/nav2_lifecycle.py с проверкой
        # состояния (стандартный менеджер под нагрузкой зависал: FastDDS терял ответ сервиса)
        ros2 launch "$ROOT/config/nav2.launch.py" use_sim_time:=True \
            params_file:="$ROOT/config/nav2_params.yaml" > "$LOG/nav2.log" 2>&1 &
        NAV_PID=$!
        disown
        python3 scripts/nav2_lifecycle.py 2>&1 | tee "$LOG/nav2_lifecycle.log" \
            | grep --line-buffered -E "Nav2 активен|не настроился|не включился" &  # итог — в этот терминал
        NAVUP_PID=$(jobs -p %+)  # группа конвейера = PID его первого процесса, а $! — последнего
        disown
        echo "Nav2 запущен (config/nav2_params.yaml), включится, когда SLAM даст позу в метрах; цели — «2D Goal Pose» в RViz"
    fi
    echo "stella_vslam (${MAP:+локализация по $MAP, }окно: $VIEWER), мост в метры и сверка с эталоном запущены; прогон — $RUN"
fi

if [ "${WITH_RVIZ:-1}" != 0 ]; then  # WITH_RVIZ=0 — без RViz
    RVIZ_CFG=rviz/go2_camera360.rviz
    [ "$SLAM" != 0 ] && RVIZ_CFG=rviz/go2_slam.rviz
    rviz2 -d "$RVIZ_CFG" --ros-args -p use_sim_time:=true > "$LOG/rviz.log" 2>&1 &
    RVIZ_PID=$!
    disown
fi
if [ -n "${GOALS:-}" ]; then
    [ "${NAV:-0}" != 0 ] || { echo "GOALS — только вместе с NAV=1"; exit 1; }
    if [ "$GOALS" = auto ]; then GOAL_ARGS=(--from-run "$MAP"); else read -r -a GOAL_ARGS <<< "$GOALS"; fi
    [ -n "${MAP:-}" ] && [ -f "$MAP/map_world.json" ] && GOAL_ARGS+=(--map-world "$MAP/map_world.json")
    echo "Цели для Nav2: ${GOALS} (scripts/nav_goals.py), потом выход."
    python3 scripts/nav_goals.py "${GOAL_ARGS[@]}" --out "$RUN" 2>&1 | grep --line-buffered -v "^\[" || true
    exit 0
fi
if [ -n "${DRIVE:-}" ]; then
    echo "Езда по программе «$DRIVE» (scripts/drive_pattern.py), потом выход."
    python3 scripts/drive_pattern.py "$DRIVE" 2>&1 | grep --line-buffered -E "программа|шаг" || true
    exit 0
fi
echo "Контроллер ходьбы${RVIZ_PID:+ и RViz} запущен${RVIZ_PID:+ы}. Управление — здесь (окно терминала должно быть активным):"
python3 scripts/teleop_wasd.py --watch-pid $ISAAC_PID
