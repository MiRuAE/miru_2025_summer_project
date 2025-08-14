# miru_2025_summer_project
F1TENTH Path Following + MCL Localization

f1tenth_pp(래플린 녹화·경로 퍼블리시·Pure Pursuit)와 particle_filter_cpp(MCL 로컬라이제이션)를 함께 사용해 SLAM/맵 기반 위치추정 + 레이싱 라인 추종 주행을 구성하는 방법을 정리했습니다.

구성 개요

Localization: particle_filter_cpp가 map → odom TF를 퍼블리시 (LiDAR + 맵 정합)

Odometry: 시뮬/실차 오도메트리(odom → base_link 또는 ego_racecar/base_link)

Path Following: f1tenth_pp

lap_recorder: 한 바퀴 주행 좌표를 CSV로 기록

path_follow_pp: CSV를 nav_msgs/Path로 퍼블리시하고 Pure Pursuit로 /drive 출력

준비물

ROS 2 Humble

토픽

/scan : sensor_msgs/LaserScan

/ego_racecar/odom (또는 /odom) : nav_msgs/Odometry

프레임

기본: map → odom → ego_racecar/base_link (또는 base_link)

맵 파일

예: /home/shchon11/sim_ws/maps/my_map.yaml (+ pgm)

설치
1) particle_filter_cpp (MCL)
