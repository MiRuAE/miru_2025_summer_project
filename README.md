# 🚗 F1TENTH Pure Pursuit & Particle Filter Integration

이 프로젝트는 **F1TENTH 플랫폼**에서  
- **Pure Pursuit 기반 경로 추종(f1tenth_pp)**  
- **C++ 기반 Particle Filter Localization(particle_filter_cpp)**  

를 통합하여 자율주행을 구현하는 예제입니다.

---

## 📦 패키지 구성

### f1tenth_pp
- CSV 기반 경로 추종 (Pure Pursuit)
- 기록된 주행 경로를 불러와 차량을 제어

### particle_filter_cpp
- C++로 작성된 Particle Filter Localization
- 라이다 데이터를 이용하여 차량 위치를 추정
- `map -> base_link` 변환(TF) 발행

---

## 1️⃣ 설치 방법

### 1. ROS 2 워크스페이스 생성
```bash
mkdir -p ~/f1tenth_ws/src
cd ~/f1tenth_ws/src
```
2. 레포지토리 클론

# Pure Pursuit 패키지
```bash
git clone <f1tenth_pp_repo_url>
```
# Particle Filter Localization 패키지
```bash
git clone https://github.com/2025-AILAB-Internship-F1TheBeast/particle_filter_cpp.git
```
3. 빌드
```bash
cd ~/f1tenth_ws
colcon build --symlink-install
source install/setup.bash
```
2️⃣ f1tenth_pp 사용법
(1) 주행 경로 기록
```bash
ros2 run f1tenth_pp lap_recorder --ros-args \
  -p odom_topic:=/ego_racecar/odom \
  -p csv_path:=/home/<사용자>/maps/raceline_raw.csv
```
    한 바퀴 주행 후 Ctrl + C → 지정한 경로에 CSV 저장됨

(2) 기록된 경로 기반 주행
```bash
ros2 run f1tenth_pp path_follow_pp --ros-args \
  -p csv_path:=/home/<사용자>/maps/raceline_raw.csv
```
주요 파라미터
파라미터 이름	기본값	설명
csv_path	없음	주행 경로 CSV 파일 경로
Ld	1.5	Pure Pursuit Lookahead Distance (m)
v_max	5.0	최대 속도 (m/s)
frame_map	"map"	전역 좌표계 이름
frame_base	"base_link"	차량 기준 좌표계 이름
3️⃣ particle_filter_cpp 사용법
실행
```bash
ros2 run particle_filter_cpp particle_filter --ros-args \
  -p map_file:=/home/<사용자>/maps/map.yaml
```
주요 기능

    /scan 토픽 구독 (라이다 데이터)

    맵 파일 기반 위치 추정

    /pf_pose 토픽에 추정된 차량 위치 발행

    map -> base_link TF 브로드캐스팅

주요 파라미터
파라미터 이름	설명
map_file	사용될 맵 파일 경로
num_particles	파티클 개수
sensor_noise	센서 노이즈 표준편차
motion_noise	이동 노이즈 표준편차
4️⃣ 통합 실행 순서

    Localization 실행
```bash
ros2 run particle_filter_cpp particle_filter --ros-args \
  -p map_file:=/home/<사용자>/maps/map.yaml
```
    Pure Pursuit 실행
```bash
ros2 run f1tenth_pp path_follow_pp --ros-args \
  -p csv_path:=/home/<사용자>/maps/raceline_raw.csv \
  -p frame_map:=map \
  -p frame_base:=base_link \
  -p v_max:=2.0
```

📜 라이선스

MIT License


---

원하면 여기에 **토픽 구조도**와 **실행 흐름 다이어그램**까지 넣어서  
실행 구조가 한눈에 보이도록 할 수도 있습니다.  
그렇게 하면 GitHub용 README가 더 완성도 있게 나와요.  

원하세요? 제가 그림까지 추가해드릴게요.
