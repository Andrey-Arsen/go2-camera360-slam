# Окружение для SLAM (source, не запускать): ROS 2 Humble + stella_vslam из third_party/stella
# (scripts/install_stella.sh). Даёт `ros2 run stella_vslam_ros run_slam` и переменную STELLA_VOCAB.
# Если ROS уже подключён (например, лаунчером вместе с IsaacLegs) — только добавляет stella.
_c360="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
[ -n "$ROS_DISTRO" ] || source "$_c360/scripts/ros_env.sh"
export LD_LIBRARY_PATH="$_c360/third_party/stella/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
if [ -f "$_c360/third_party/stella/ros_ws/install/local_setup.bash" ]; then
    source "$_c360/third_party/stella/ros_ws/install/local_setup.bash"
fi
export STELLA_VOCAB="$_c360/third_party/stella/orb_vocab.fbow"
unset _c360
