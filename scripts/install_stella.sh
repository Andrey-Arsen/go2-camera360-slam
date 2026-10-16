#!/usr/bin/env bash
# Сборка stella_vslam + окно PangolinViewer + ROS 2-обёртка stella_vslam_ros — в third_party/stella,
# без sudo (ничего не ставится в систему). Версии закреплены — те, с которыми проект проверен.
#
# Нужны apt-пакеты (на рабочей машине уже стоят): ros-humble-libg2o libglew-dev libgl-dev
#   libeigen3-dev libopencv-dev libyaml-cpp-dev libsqlite3-dev libsuitesparse-dev
#
#   scripts/install_stella.sh          # первый раз — несколько минут; повторно — только изменения
#   JOBS=8 scripts/install_stella.sh   # меньше потоков сборки (по умолчанию половина ядер)
#
# Результат: third_party/stella/install (библиотеки), third_party/stella/ros_ws (run_slam),
# third_party/stella/orb_vocab.fbow (словарь ORB). Окружение — scripts/slam_env.sh.
set -e
cd "$(dirname "$0")/.."
S=$PWD/third_party/stella
PREFIX=$S/install
JOBS=${JOBS:-$(( $(nproc) / 2 ))}
mkdir -p "$S/src" "$S/ros_ws/src" "$PREFIX"
source scripts/ros_env.sh
export CMAKE_PREFIX_PATH="$PREFIX${CMAKE_PREFIX_PATH:+:$CMAKE_PREFIX_PATH}"

clone() {  # url каталог коммит [ветка]
    local url=$1 dir=$2 commit=$3 branch=${4:-}
    if [ ! -d "$dir/.git" ]; then
        git clone -q ${branch:+-b "$branch"} "$url" "$dir"
    fi
    (
        cd "$dir" || exit 1
        git checkout -q "$commit" 2>/dev/null || { git fetch -q origin && git checkout -q "$commit"; }
        git submodule update -q --init --recursive
    )
    echo "== $(basename "$dir") $(cd "$dir" && git rev-parse --short=12 HEAD)"
}

build() {  # каталог [аргументы cmake...]
    local dir=$1; shift
    cmake -S "$S/src/$dir" -B "$S/build/$dir" -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PREFIX" \
          -DCMAKE_INSTALL_RPATH="$PREFIX/lib" "$@" > "$S/build_$dir.log" 2>&1 \
        || { echo "!! cmake $dir — см. $S/build_$dir.log"; tail -20 "$S/build_$dir.log"; exit 1; }
    cmake --build "$S/build/$dir" -j"$JOBS" >> "$S/build_$dir.log" 2>&1 \
        || { echo "!! сборка $dir — см. $S/build_$dir.log"; grep -m5 -E "error|Error" "$S/build_$dir.log"; exit 1; }
    cmake --install "$S/build/$dir" >> "$S/build_$dir.log" 2>&1
}

# Pangolin — версия и правка из инструкции stella_vslam для ROS 2 (realpath ломает сборку)
clone https://github.com/stevenlovegrove/Pangolin.git "$S/src/Pangolin" ad8b5f832222
if grep -q "realpath(path.c_str(), abs_path)" "$S/src/Pangolin/src/utils/file_utils.cpp"; then
    sed -i -e "193,198d" "$S/src/Pangolin/src/utils/file_utils.cpp"
fi
build Pangolin -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF -DBUILD_PANGOLIN_DEPTHSENSE=OFF -DBUILD_PANGOLIN_FFMPEG=OFF \
    -DBUILD_PANGOLIN_LIBDC1394=OFF -DBUILD_PANGOLIN_LIBJPEG=OFF -DBUILD_PANGOLIN_LIBOPENEXR=OFF \
    -DBUILD_PANGOLIN_LIBPNG=OFF -DBUILD_PANGOLIN_LIBREALSENSE=OFF -DBUILD_PANGOLIN_LIBREALSENSE2=OFF \
    -DBUILD_PANGOLIN_LIBTIFF=OFF -DBUILD_PANGOLIN_LIBUVC=OFF -DBUILD_PANGOLIN_LZ4=OFF -DBUILD_PANGOLIN_OPENNI=OFF \
    -DBUILD_PANGOLIN_OPENNI2=OFF -DBUILD_PANGOLIN_PLEORA=OFF -DBUILD_PANGOLIN_PYTHON=OFF -DBUILD_PANGOLIN_TELICAM=OFF \
    -DBUILD_PANGOLIN_TOON=OFF -DBUILD_PANGOLIN_UVC_MEDIAFOUNDATION=OFF -DBUILD_PANGOLIN_V4L=OFF \
    -DBUILD_PANGOLIN_VIDEO=OFF -DBUILD_PANGOLIN_ZSTD=OFF -DBUILD_PYPANGOLIN_MODULE=OFF
echo "   Pangolin собран"

clone https://github.com/stella-cv/stella_vslam.git "$S/src/stella_vslam" e445b5452535
build stella_vslam -DBUILD_TESTS=OFF -DUSE_CCACHE=OFF
echo "   stella_vslam собран"

clone https://github.com/stella-cv/pangolin_viewer.git "$S/src/pangolin_viewer" 1e0233534055
build pangolin_viewer
echo "   pangolin_viewer собран"

if [ ! -s "$S/orb_vocab.fbow" ]; then
    curl -sSfL https://github.com/stella-cv/FBoW_orb_vocab/raw/main/orb_vocab.fbow -o "$S/orb_vocab.fbow.part"
    mv "$S/orb_vocab.fbow.part" "$S/orb_vocab.fbow"
fi
echo "== словарь ORB $(du -h "$S/orb_vocab.fbow" | cut -f1)"

clone https://github.com/stella-cv/stella_vslam_ros.git "$S/ros_ws/src/stella_vslam_ros" 186f623dd1c2 ros2
# Наши правки обёртки (patches/stella_vslam_ros.patch):
#  * QoS кадров: в обёртке sensor_data (BEST_EFFORT, глубина 1) — панорама 1920x960 весит 5.5 МБ, и
#    по DDS терялось больше половины (из 287 кадров stella получила 123, трекинг срывался на
#    поворотах). RELIABLE с той же глубиной 1 — кадры доходят, а если stella не успевает, берёт свежий;
#  * ~/landmarks — точки карты (PointCloud2, единицы карты stella) раз в landmarks_period секунд.
PATCH=$PWD/patches/stella_vslam_ros.patch
(
    cd "$S/ros_ws/src/stella_vslam_ros"
    if git apply --check "$PATCH" 2>/dev/null; then
        git apply "$PATCH"
    elif ! git apply --reverse --check "$PATCH" 2>/dev/null; then
        echo "!! patches/stella_vslam_ros.patch не накладывается — исходник изменился?"; exit 1
    fi
)
(cd "$S/ros_ws" && colcon build --symlink-install --event-handlers console_direct- \
    --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_RPATH="$PREFIX/lib" -DCMAKE_PREFIX_PATH="$CMAKE_PREFIX_PATH" \
    > "$S/build_stella_vslam_ros.log" 2>&1) \
    || { echo "!! сборка stella_vslam_ros — см. $S/build_stella_vslam_ros.log"; tail -30 "$S/build_stella_vslam_ros.log"; exit 1; }
grep -q "pangolin_viewer" "$S/build_stella_vslam_ros.log" || true
echo "   stella_vslam_ros собран"

source scripts/slam_env.sh
ros2 run stella_vslam_ros run_slam -h 2>&1 | grep -q -- "--viewer" \
    && echo "Готово: run_slam запускается. Окружение: source scripts/slam_env.sh" \
    || { echo "!! run_slam не запускается:"; ros2 run stella_vslam_ros run_slam -h 2>&1 | tail -5; exit 1; }
