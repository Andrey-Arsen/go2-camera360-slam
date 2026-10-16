#!/usr/bin/env bash
# Go2 с 360-камерой в Isaac Sim + SLAM: склейка двух fisheye в панораму, stella_vslam (окно
# карты), сверка с эталоном симуляции; рулить WASD. Всё как у scripts/go2_sim.sh (см. его шапку).
#   scripts/slam_sim.sh                               # Full Warehouse, WASD
#   DRIVE=slam scripts/slam_sim.sh                    # проехать программу без человека и выйти
#   LENSES=front scripts/slam_sim.sh                  # SLAM только по переднему объективу (сравнение)
# Результат — out/slam/<прогон>/: report.txt (точность), trajectory.png, map.msg (карта stella).
SLAM=1 exec "$(dirname "$0")/go2_sim.sh" "$@"
