#!/usr/bin/env bash
# Маска тела робота для SLAM — без участия человека: Isaac без окна (ровный пол), робот ходит по
# программе (scripts/drive_pattern.py mask: вперёд, повороты, вбок, назад), сегментация копит
# пиксели тела на обоих объективах -> config/robot_mask_front.png, config/robot_mask_back.png
# (255 — робот). Из них scripts/equirect.py делает маску панорамы для stella_vslam.
# Делать заново после изменения надстройки (scripts/build_go2_camera360.py). Около 2 минут.
#   scripts/make_robot_mask.sh [--image-size 960]
set -e
set -m
cd "$(dirname "$0")/.."
ROOT=$PWD
for need in robot/go2_camera360/go2_camera360.usda third_party/IsaacLegs/install/setup.bash third_party/pydeps/torch; do
    [ -e "$need" ] || { echo "нет $need — см. README, раздел «Установка»"; exit 1; }
done
export ROS_DOMAIN_ID=${MASK_DOMAIN:-77}  # свой домен: не мешать уже запущенной симуляции
source scripts/ros_env.sh
source third_party/IsaacLegs/install/setup.bash
export PYTHONPATH="${PYTHONPATH:+$PYTHONPATH:}$ROOT/third_party/pydeps"
LOG=out/mask_logs
TMP=out/robot_mask
mkdir -p "$LOG"
rm -rf "$TMP"

~/isaacsim/python.sh scripts/run_go2_isaac.py --env flat_grid --headless --robot-mask "$TMP" "$@" > "$LOG/isaac.log" 2>&1 &
ISAAC_PID=$!
disown
CTRL_PID=""
cleanup() {
    set +e
    if kill -0 -- "-$ISAAC_PID" 2>/dev/null; then
        kill -TERM -- "-$ISAAC_PID" 2>/dev/null
        for _ in $(seq 1 60); do kill -0 -- "-$ISAAC_PID" 2>/dev/null || break; sleep 1; done
    fi
    [ -n "$CTRL_PID" ] && kill -- "-$CTRL_PID" 2>/dev/null
    sleep 1
    for pid in $CTRL_PID $ISAAC_PID; do kill -9 -- "-$pid" 2>/dev/null; done
}
trap cleanup EXIT
echo "Isaac Sim грузится без окна (домен $ROS_DOMAIN_ID)..."
until grep -q "\[go2\] готово" "$LOG/isaac.log"; do
    if ! kill -0 $ISAAC_PID 2>/dev/null || grep -q "^\[go2\] нет\|^\[go2\] не\|Traceback" "$LOG/isaac.log"; then
        echo "Isaac Sim не запустился:"; grep -E "^\[go2\]|Traceback|Error:" "$LOG/isaac.log" | tail -5; exit 1
    fi
    sleep 2
done
# один поток torch: под нагрузкой (окна, SLAM, Nav2) шаг политики со всеми потоками рос с 0.04 до
# 18–33 мс, команды суставам опаздывали, очередь joint_states (KEEP_ALL) росла — робот падал
OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 ros2 launch fullbody_controller go2.launch.py > "$LOG/controller.log" 2>&1 &
CTRL_PID=$!
disown
python3 scripts/drive_pattern.py mask 2>&1 | grep --line-buffered -E "программа|шаг" || true
cleanup  # Isaac пишет маски при выходе
trap - EXIT
grep "^\[go2\] маска робота" "$LOG/isaac.log" | tail -1
for n in front back; do
    [ -s "$TMP/robot_mask_$n.png" ] || { echo "!! нет маски $n — см. $LOG/isaac.log"; exit 1; }
done
mkdir -p config
cp "$TMP/robot_mask_front.png" "$TMP/robot_mask_back.png" config/
echo "Готово: config/robot_mask_front.png, config/robot_mask_back.png"
