#!/usr/bin/env bash
# Найти и закрыть процессы этого проекта, оставшиеся от прошлых запусков: Isaac Sim, контроллер
# ходьбы, склейка, SLAM, мост, Nav2, RViz... Нужно, если уборку при выходе прервали (второй Ctrl+C,
# закрытый терминал): иначе в новом запуске два контроллера ходьбы валят робота, а две одометрии и
# две склейки сбивают SLAM. Берутся только процессы, запущенные из папки проекта и в том же домене
# ROS (ROS_DOMAIN_ID), что и этот терминал, — другие домены (например, проверки без окна) не трогаются.
#   scripts/stop_all.sh          # закрыть
#   scripts/stop_all.sh --list   # только показать
cd "$(dirname "$0")/.."
ROOT=$PWD
DOMAIN=${ROS_DOMAIN_ID:-0}

# себя и всех предков (лаунчер, который спрашивает) не трогаем
skip=" $$ "
p=$PPID
while [ -n "$p" ] && [ "$p" -gt 1 ] 2>/dev/null; do
    skip+="$p "
    p=$(awk '{print $4}' "/proc/$p/stat" 2>/dev/null)
done

found=()
for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    case "$skip" in *" $pid "*) continue ;; esac
    cwd=$(readlink "$d/cwd" 2>/dev/null) || continue
    case "$cwd" in "$ROOT" | "$ROOT"/*) ;; *) continue ;; esac
    cmd=$(tr '\0' ' ' < "$d/cmdline" 2>/dev/null)
    case "$cmd" in
        *run_go2_isaac.py* | *policy_controller* | *"ros2 launch"* | *"ros2 run"* | *run_slam* | \
        */nav2_* | *rviz2* | *"scripts/"*.py* | *go2_sim.sh* | *slam_sim.sh* | *make_robot_mask.sh*) ;;
        *) continue ;;
    esac
    dom=$(tr '\0' '\n' < "$d/environ" 2>/dev/null | sed -n 's/^ROS_DOMAIN_ID=//p')
    [ "${dom:-0}" = "$DOMAIN" ] || continue
    found+=("$pid")
    [ "${1:-}" = "--list" ] && echo "  $pid  ${cmd:0:110}"
done
[ "${1:-}" = "--list" ] && exit 0
if [ ${#found[@]} -eq 0 ]; then
    echo "процессов проекта в домене $DOMAIN нет"
    exit 0
fi
echo "закрываю ${#found[@]} процесс(ов) проекта в домене $DOMAIN..."
kill -TERM "${found[@]}" 2>/dev/null
for _ in $(seq 1 15); do
    alive=()
    for pid in "${found[@]}"; do kill -0 "$pid" 2>/dev/null && alive+=("$pid"); done
    [ ${#alive[@]} -eq 0 ] && break
    sleep 1
done
[ ${#alive[@]} -gt 0 ] && kill -9 "${alive[@]}" 2>/dev/null
echo "готово"
