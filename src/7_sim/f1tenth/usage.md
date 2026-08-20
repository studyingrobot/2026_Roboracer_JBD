# F1TENTH ROS2 Humble Docker 배포 및 자율주행 노드 구조 정리본

## 1. 현재 프로젝트 목표

이 프로젝트의 목표는 ROS2 Humble 기반 F1TENTH 시뮬레이터 환경에서 자율주행 stack을 만들고, 이후 실차 주행으로 이식할 수 있는 구조를 만드는 것이다.

현재 목표는 다음과 같다.

```text
1단계: Docker 기반 시뮬레이터 환경 통일
2단계: 시뮬레이터에서 localization → planning → control 실행
3단계: Pure Pursuit 기반 경로 추종 확인
4단계: drive_mode:=sim / real 토글로 시뮬과 실차 출력 분리
5단계: 이후 safety, goal pose planner, vehicle interface, 실차 localization 등 확장
```

현재까지 만든 기본 구조는 다음과 같다.

```text
localization
    ↓
planning
    ↓
control
    ↓
sim / real output
```

---

# 2. GitHub와 Docker의 차이

## 2.1 GitHub

GitHub는 코드와 문서를 저장하는 곳이다.

현재 GitHub repo에는 다음이 들어 있다.

```text
Dockerfile
README.md
usage.md
f1tenth_gym_ros 시뮬 브리지 코드
algorithms/
  localization/
  planning/
  control/
  f1tenth_bringup/
```

즉 GitHub는 다음 역할이다.

```text
코드 저장소
문서 저장소
Dockerfile 저장소
팀원 배포용 repo
```

팀원이 `git clone`을 하면 Docker 이미지 자체를 받는 것이 아니라,
Docker 이미지를 만들기 위한 코드와 Dockerfile을 받는 것이다.

```bash
git clone https://github.com/SeEEEun/f1tenth_gym_ros_humble
```

---

## 2.2 Docker

Docker는 실행 환경을 통일하기 위한 도구다.

Dockerfile에는 다음과 같은 환경 설치 과정이 들어 있다.

```text
Ubuntu 22.04
ROS2 Humble
RViz2
f1tenth_gym
f1tenth_gym_ros
slam_toolbox
nav2 map server
ackermann_msgs
teleop_twist_keyboard
필요한 Python 패키지
```

팀원이 `docker build`를 하면 GitHub에서 받은 Dockerfile을 기반으로 Docker image가 만들어진다.

```bash
docker build -t f1tenth_gym_ros_humble -f Dockerfile .
```

이후 `rocker`로 image를 실행하면 컨테이너에 들어갈 수 있다.

```bash
rocker --x11 ... f1tenth_gym_ros_humble
```

---

## 2.3 한 줄 요약

```text
GitHub = 코드와 Dockerfile을 받는 곳
Docker = 그 Dockerfile로 만든 ROS2 실행 환경
```

즉 현재 배포 방식은 다음과 같다.

```text
GitHub clone
    ↓
Docker build
    ↓
rocker로 컨테이너 실행
    ↓
ROS2 F1TENTH 시뮬 실행
```

---

# 3. 팀원이 미리 설치해야 하는 것들

## 3.1 필수

팀원 PC에는 최소한 아래가 필요하다.

```text
Ubuntu 22.04 권장
Docker
rocker
Git
Python3 pip
```

---

## 3.2 Docker 설치

```bash
sudo apt update
sudo apt install -y ca-certificates curl gnupg lsb-release

sudo install -m 0755 -d /etc/apt/keyrings

curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
  https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

sudo apt update
sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

Docker 권한 설정:

```bash
sudo usermod -aG docker $USER
newgrp docker
```

확인:

```bash
docker ps
```

---

## 3.3 rocker 설치

```bash
sudo apt update
sudo apt install -y python3-pip python3-venv

python3 -m pip install --user -U rocker

echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc
```

확인:

```bash
which rocker
rocker --version
```

정상 예시:

```text
/home/<user>/.local/bin/rocker
rocker 0.3.0
```

---

## 3.4 NVIDIA GPU가 있는 경우

GPU가 있는 PC에서는 NVIDIA driver와 nvidia-container-toolkit이 필요할 수 있다.

설치 예시:

```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt update
sudo apt install -y nvidia-container-toolkit

sudo systemctl restart docker
```

확인:

```bash
nvidia-smi
```

GPU가 없는 PC에서는 `--nvidia` 옵션을 쓰면 안 된다.

---

# 4. GitHub에서 받아오는 명령어

팀원 PC에서 다음을 실행한다.

```bash
cd ~

git clone https://github.com/SeEEEun/f1tenth_gym_ros_humble

cd f1tenth_gym_ros_humble
```

정상 확인:

```bash
ls
```

예상 구조:

```text
Dockerfile
README.md
usage.md
algorithms
f1tenth_gym_ros
launch
config
maps
package.xml
setup.py
...
```

알고리즘 패키지 확인:

```bash
ls algorithms
```

정상:

```text
control
f1tenth_bringup
localization
planning
```

---

# 5. Docker 이미지 빌드

repo 폴더 안에서 실행한다.

```bash
cd ~/f1tenth_gym_ros_humble

docker build -t f1tenth_gym_ros_humble -f Dockerfile .
```

여기서 명령어 의미는 다음과 같다.

```text
docker build
  → Dockerfile을 기반으로 이미지 생성

-t f1tenth_gym_ros_humble
  → 이미지 이름을 f1tenth_gym_ros_humble로 지정

-f Dockerfile
  → 현재 폴더의 Dockerfile 사용

.
  → 현재 폴더 전체를 build context로 사용
```

빌드 완료 확인:

```bash
docker images | grep f1tenth
```

정상 예시:

```text
f1tenth_gym_ros_humble   latest   ...
```

---

# 6. Docker 이미지와 컨테이너의 차이

```text
Docker image
  → 설치가 끝난 실행 환경의 원본 상자

Container
  → 그 image를 실제로 실행한 작업 공간
```

즉:

```text
docker build
  → image 생성

rocker 또는 docker run
  → container 실행
```

---

# 7. 컨테이너 실행 명령어

## 7.1 GPU 없는 PC

```bash
cd ~/f1tenth_gym_ros_humble

rocker --x11 \
  --volume .:/sim_ws/src/f1tenth_gym_ros \
  --volume ./algorithms/localization:/sim_ws/src/localization \
  --volume ./algorithms/planning:/sim_ws/src/planning \
  --volume ./algorithms/control:/sim_ws/src/control \
  --volume ./algorithms/f1tenth_bringup:/sim_ws/src/f1tenth_bringup \
  -- f1tenth_gym_ros_humble
```

---

## 7.2 GPU 있는 PC

```bash
cd ~/f1tenth_gym_ros_humble

rocker --nvidia --x11 \
  --volume .:/sim_ws/src/f1tenth_gym_ros \
  --volume ./algorithms/localization:/sim_ws/src/localization \
  --volume ./algorithms/planning:/sim_ws/src/planning \
  --volume ./algorithms/control:/sim_ws/src/control \
  --volume ./algorithms/f1tenth_bringup:/sim_ws/src/f1tenth_bringup \
  -- f1tenth_gym_ros_humble
```

`--nvidia`에서 다음 에러가 나오면 GPU가 안 잡힌 PC다.

```text
rocker: error: --nvidia was specified, but no NVIDIA drivers or devices were detected on the host.
```

이 경우 `--nvidia`를 빼고 실행한다.

---

## 7.3 RViz OpenGL 문제가 나는 경우

GPU 없는 PC에서 RViz가 안 뜨면 소프트웨어 렌더링으로 실행한다.

```bash
cd ~/f1tenth_gym_ros_humble

rocker --x11 \
  --env LIBGL_ALWAYS_SOFTWARE=1 \
  --volume .:/sim_ws/src/f1tenth_gym_ros \
  --volume ./algorithms/localization:/sim_ws/src/localization \
  --volume ./algorithms/planning:/sim_ws/src/planning \
  --volume ./algorithms/control:/sim_ws/src/control \
  --volume ./algorithms/f1tenth_bringup:/sim_ws/src/f1tenth_bringup \
  -- f1tenth_gym_ros_humble
```

---

# 8. 컨테이너 안에서 ROS 환경 source

컨테이너에 들어가면 프롬프트가 다음처럼 바뀐다.

```text
root@<container_id>:/sim_ws#
```

항상 먼저 아래를 실행한다.

```bash
cd /sim_ws
source /opt/ros/humble/setup.bash
```

패키지를 빌드한 후에는 다음도 실행한다.

```bash
source install/local_setup.bash
```

`ros2: command not found`가 나오면 source가 안 된 것이다.

해결:

```bash
source /opt/ros/humble/setup.bash
source /sim_ws/install/local_setup.bash
which ros2
```

정상:

```text
/opt/ros/humble/bin/ros2
```

---

# 9. 컨테이너 안에서 패키지 빌드

```bash
cd /sim_ws
source /opt/ros/humble/setup.bash

colcon build --packages-select localization planning control f1tenth_bringup

source install/local_setup.bash
```

정상 예시:

```text
Finished <<< localization
Finished <<< planning
Finished <<< control
Finished <<< f1tenth_bringup
```

---

# 10. 실행 순서

## 10.1 터미널 1 — 시뮬레이터 / RViz 실행

컨테이너 안에서:

```bash
cd /sim_ws
source /opt/ros/humble/setup.bash
source install/local_setup.bash

ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

RViz가 뜨고 차량이 보이면 정상이다.

---

## 10.2 터미널 2 — 같은 컨테이너 접속

호스트 새 터미널에서 컨테이너 ID를 확인한다.

```bash
docker ps --format "table {{.ID}}\t{{.Image}}\t{{.Names}}\t{{.Status}}"
```

F1TENTH 컨테이너 ID를 확인한 뒤 접속한다.

```bash
docker exec -it <CONTAINER_ID> /bin/bash -lc "cd /sim_ws && source /opt/ros/humble/setup.bash && source install/local_setup.bash && bash"
```

예시:

```bash
docker exec -it 59e669bf809e /bin/bash -lc "cd /sim_ws && source /opt/ros/humble/setup.bash && source install/local_setup.bash && bash"
```

---

## 10.3 터미널 2 — 자율주행 통합 launch 실행

```bash
ros2 launch f1tenth_bringup autonomy.launch.py drive_mode:=sim
```

정상 로그 예시:

```text
localization_node started
waypoint_planner_node started
pure_pursuit_node started
drive_mode       : sim
sim_drive_topic  : /drive
```

---

## 10.4 터미널 3 — 확인용

새 터미널에서 같은 컨테이너에 접속한 뒤:

```bash
ros2 node list
```

정상 예시:

```text
/bridge
/localization_node
/waypoint_planner_node
/pure_pursuit_node
/rviz
```

토픽 확인:

```bash
ros2 topic info /drive
ros2 topic echo /drive --once
```

정상 예시:

```text
Type: ackermann_msgs/msg/AckermannDriveStamped
Publisher count: 1
Subscription count: 1
```

---

# 11. 현재 받아온 repo 설명

현재 GitHub repo를 clone하면 다음이 받아진다.

```text
f1tenth_gym_ros_humble/
├── Dockerfile
├── README.md
├── usage.md
├── algorithms/
│   ├── localization/
│   ├── planning/
│   ├── control/
│   └── f1tenth_bringup/
├── f1tenth_gym_ros/
├── launch/
├── config/
├── maps/
├── package.xml
└── setup.py
```

## 11.1 Dockerfile

Dockerfile은 Docker image를 만들기 위한 설치 스크립트다.

현재 Dockerfile은 다음 문제를 우회하도록 수정되어 있다.

```text
scipy / llvmlite pip hash mismatch 문제
apt 패키지 기반 scipy / numba 설치
casadi는 --no-deps로 설치
numpy 버전 충돌 방지
```

## 11.2 algorithms 폴더

`algorithms/` 안에는 우리가 만든 자율주행 패키지가 들어 있다.

```text
algorithms/localization
algorithms/planning
algorithms/control
algorithms/f1tenth_bringup
```

이 폴더들은 rocker 실행 시 `/sim_ws/src` 아래로 마운트된다.

```text
./algorithms/localization      → /sim_ws/src/localization
./algorithms/planning          → /sim_ws/src/planning
./algorithms/control           → /sim_ws/src/control
./algorithms/f1tenth_bringup   → /sim_ws/src/f1tenth_bringup
```

---

# 12. 현재 있는 노드 설명

## 12.1 `localization_node`

### 패키지

```text
localization
```

### 역할

시뮬레이터에서 제공하는 odometry를 받아서 공통 localization 토픽으로 다시 publish한다.

현재는 Particle Filter가 아니라 odom relay 역할이다.

### 입력

```text
/ego_racecar/odom
```

타입:

```text
nav_msgs/msg/Odometry
```

### 출력

```text
/localization/odom
/localization/pose
```

타입:

```text
/localization/odom  → nav_msgs/msg/Odometry
/localization/pose  → geometry_msgs/msg/PoseStamped
```

### 구조

```text
/ego_racecar/odom
        ↓
localization_node
        ↓
/localization/odom
/localization/pose
```

### 실차 적용 관점

실차에서는 `/ego_racecar/odom`이 없을 수 있다.
이 경우 input odom topic을 다음 중 하나로 바꿔야 한다.

```text
/odom
/vesc/odom
/pf/pose/odom
/amcl_pose
```

이 노드는 나중에 Particle Filter나 AMCL로 대체 가능하다.

---

## 12.2 `waypoint_planner_node`

### 패키지

```text
planning
```

### 역할

CSV waypoint 파일을 읽어서 전체 경로를 publish한다.

### 입력

```text
waypoints.csv
```

현재 형식:

```csv
x,y,speed
-3.8,0.2,1.0
-2.0,0.2,1.0
0.0,0.2,1.0
...
```

### 출력

```text
/planning/path
/planning/markers
```

타입:

```text
/planning/path     → nav_msgs/msg/Path
/planning/markers  → visualization_msgs/msg/MarkerArray
```

### 구조

```text
waypoints.csv
        ↓
waypoint_planner_node
        ↓
/planning/path
/planning/markers
```

### 현재 한계

현재 waypoint는 테스트용 사각형 waypoint다.
실제 트랙 한 바퀴 주행을 위해서는 트랙 중앙선 또는 raceline CSV가 필요하다.

---

## 12.3 `pure_pursuit_node`

### 패키지

```text
control
```

### 역할

현재 위치와 경로를 받아서 Pure Pursuit 기반 조향과 PID 기반 속도 명령을 생성한다.

### 입력

```text
/localization/odom
/planning/path
```

타입:

```text
/localization/odom → nav_msgs/msg/Odometry
/planning/path     → nav_msgs/msg/Path
```

---

## 12.4 Pure Pursuit와 PID 구분

Pure Pursuit는 조향 알고리즘이다.

```text
lookahead point 계산
차량 기준 target point 변환
곡률 계산
조향각 계산
```

PID는 속도 추종에 사용한다.

```text
목표 속도 - 현재 속도
        ↓
PID
        ↓
속도 명령
```

즉:

```text
조향 = Pure Pursuit
속도 = PID 기반 속도 추종
```

---

## 12.5 `drive_mode:=sim`

시뮬 모드에서는 `/drive`로 publish한다.

```bash
ros2 launch f1tenth_bringup autonomy.launch.py drive_mode:=sim
```

출력:

```text
/drive
```

타입:

```text
ackermann_msgs/msg/AckermannDriveStamped
```

구조:

```text
pure_pursuit_node
        ↓
/drive
        ↓
f1tenth_gym_ros simulator
```

확인:

```bash
ros2 topic info /drive
ros2 topic echo /drive --once
```

---

## 12.6 `drive_mode:=real`

실차 모드에서는 실차 VESC/servo 토픽으로 publish한다.

```bash
ros2 launch f1tenth_bringup autonomy.launch.py drive_mode:=real
```

출력:

```text
/commands/motor/speed
/commands/servo/position
```

타입:

```text
/commands/motor/speed     → std_msgs/msg/Float64
/commands/servo/position  → std_msgs/msg/Float64
```

구조:

```text
pure_pursuit_node
        ↓
/commands/motor/speed
/commands/servo/position
        ↓
VESC / servo driver
```

확인:

```bash
ros2 topic list | grep commands
ros2 topic echo /commands/motor/speed --once
ros2 topic echo /commands/servo/position --once
```

---

## 12.7 `f1tenth_bringup`

### 패키지

```text
f1tenth_bringup
```

### 역할

여러 노드를 한 번에 실행한다.

현재 통합 launch:

```bash
ros2 launch f1tenth_bringup autonomy.launch.py drive_mode:=sim
```

이 launch는 다음 노드를 실행한다.

```text
localization_node
waypoint_planner_node
pure_pursuit_node
```

현재 전체 구조:

```text
/ego_racecar/odom
        ↓
localization_node
        ↓
/localization/odom
        ↓
pure_pursuit_node
        ↑
/planning/path
        ↑
waypoint_planner_node
        ↑
waypoints.csv
```

최종 출력:

```text
drive_mode:=sim
    → /drive

drive_mode:=real
    → /commands/motor/speed
    → /commands/servo/position
```

---

# 13. 현재 구조의 한계

현재 구조는 자율주행 stack의 기본 골격이다.

가능한 것:

```text
시뮬 브리지 실행
localization relay
waypoint path publish
Pure Pursuit 제어
PID 속도 추종
sim / real 출력 토픽 전환
통합 launch 실행
```

부족한 것:

```text
LiDAR 기반 벽 회피 없음
emergency brake 없음
실제 트랙 waypoint 없음
goal pose를 path로 변환하는 노드 없음
실차 map-based localization 없음
servo / motor calibration 없음
lap time 측정 없음
데이터 로깅 없음
장애물 회피 없음
behavior selection 없음
```

따라서 현재 테스트용 waypoint로 주행하면 벽에 부딪힐 수 있다.

이유:

```text
Pure Pursuit는 경로 추종 알고리즘이지 벽 회피 알고리즘이 아님
현재 /scan을 control에서 사용하지 않음
현재 waypoint가 실제 트랙 중앙선이 아님
```

---

# 14. 추가하면 좋은 노드 정리

아래 노드들은 스터디용으로도 좋고, 실차 주행에도 적용 가능한 구조다.

---

# 14.1 1순위 — 반드시 추가 추천

## 1. `safety_brake_node`

### 목적

LiDAR를 이용해서 벽이나 장애물이 가까우면 차량을 멈춘다.

### 필요성

실차 주행에서 가장 먼저 필요한 safety layer다.
현재 control은 `/scan`을 보지 않기 때문에 벽이 있어도 알 수 없다.

### 입력

```text
/scan
/localization/odom
```

### 출력

```text
/safety/stop_required
/safety/min_front_distance
```

타입:

```text
/safety/stop_required       → std_msgs/msg/Bool
/safety/min_front_distance  → std_msgs/msg/Float32
```

### 기본 로직

```text
전방 ±20도 LiDAR 영역 확인
        ↓
min distance < 0.5m
        ↓
stop_required = true
        ↓
control에서 speed = 0
```

### 발전 방향

```text
TTC = distance / velocity

TTC < 0.7초
        ↓
emergency stop
```

### 스터디 포인트

```text
LaserScan 구조
angle_min
angle_increment
range indexing
front sector filtering
Time To Collision
emergency braking
```

### 추천도

```text
실차 중요도: 매우 높음
스터디 가치: 매우 높음
구현 난이도: 낮음
```

---

## 2. `goal_pose_planner_node`

### 목적

RViz의 2D Goal Pose를 받아서 `/planning/path`로 변환한다.

현재 RViz에서 Goal Pose를 찍으면 `/goal_pose`는 publish된다.
하지만 기존 planning/control은 `/goal_pose`를 사용하지 않는다.

### 입력

```text
/goal_pose
/localization/odom
```

### 출력

```text
/planning/path
```

### 구조

```text
RViz 2D Goal Pose
        ↓
/goal_pose
        ↓
goal_pose_planner_node
        ↓
/planning/path
        ↓
pure_pursuit_node
        ↓
/drive
```

### 첫 구현

처음에는 현재 위치에서 goal까지 직선 path를 생성한다.

```text
현재 위치 → goal 위치
```

### 주의

이 노드는 벽 회피 planner가 아니다.
직선 경로이므로 벽 너머에 goal을 찍으면 벽에 부딪힐 수 있다.

### 발전 방향

```text
직선 path
    ↓
A* path planning
    ↓
map 기반 global planning
```

### 추천도

```text
실차 중요도: 중간
스터디 가치: 높음
구현 난이도: 낮음
```

---

## 3. `vehicle_interface_node`

### 목적

control 노드는 공통 명령만 내고, vehicle interface가 시뮬/실차 토픽으로 변환한다.

현재는 `pure_pursuit_node` 안에 `drive_mode:=sim|real`이 들어가 있다.
동작은 하지만 실차 이식성을 생각하면 분리하는 것이 더 깔끔하다.

### 현재 구조

```text
pure_pursuit_node
    ├── drive_mode:=sim  → /drive
    └── drive_mode:=real → /commands/motor/speed
                           /commands/servo/position
```

### 추천 구조

```text
pure_pursuit_node
        ↓
/control/drive_cmd
        ↓
vehicle_interface_node
        ├── sim  → /drive
        └── real → /commands/motor/speed
                   /commands/servo/position
```

### 장점

```text
control 코드는 시뮬/실차를 몰라도 됨
실차 보정값을 vehicle_interface에 모을 수 있음
servo_center 관리 쉬움
speed_to_erpm_gain 관리 쉬움
실차 이식 구조가 깔끔해짐
```

### 실차 보정 파라미터

```yaml
servo_center: 0.5
servo_gain: 1.0
servo_min: 0.0
servo_max: 1.0

speed_to_erpm_gain: 3000.0
speed_to_erpm_offset: 0.0
```

### 추천도

```text
실차 중요도: 매우 높음
스터디 가치: 높음
구현 난이도: 중간
```

---

## 4. `debug_marker_node`

### 목적

RViz에서 자율주행 상태를 보기 쉽게 시각화한다.

### 시각화할 것

```text
global path
lookahead point
nearest waypoint
selected target
front obstacle sector
safety distance
vehicle heading
```

### 출력 토픽

```text
/debug/lookahead_marker
/debug/target_marker
/debug/safety_marker
/debug/nearest_waypoint_marker
```

### 장점

```text
팀원이 이해하기 쉬움
디버깅이 쉬움
발표 자료 만들기 좋음
실차 테스트 때 현재 상태를 빠르게 파악 가능
```

### 추천도

```text
실차 중요도: 중간
스터디 가치: 매우 높음
구현 난이도: 낮음~중간
```

---

# 14.2 2순위 — 한 바퀴 주행 성능 개선

## 5. `waypoint_manager_node`

### 목적

전체 waypoint 중 현재 차량 위치에서 가장 가까운 waypoint와 lookahead target을 계산한다.

### 입력

```text
/localization/odom
/planning/global_path
```

### 출력

```text
/planning/nearest_waypoint
/planning/target_point
/planning/lap_progress
```

### 구조

```text
/localization/odom
/planning/global_path
        ↓
waypoint_manager_node
        ↓
/planning/nearest_waypoint
/planning/target_point
/planning/lap_progress
```

### 장점

```text
Pure Pursuit 코드가 단순해짐
lap progress 계산 가능
lap timer와 연결 가능
경로 추종 상태를 RViz에서 보기 쉬움
```

### 추천도

```text
실차 중요도: 높음
스터디 가치: 높음
구현 난이도: 중간
```

---

## 6. `velocity_profile_node`

### 목적

경로의 곡률을 보고 구간별 목표 속도를 생성한다.

### 입력

```text
/planning/path
```

또는 waypoint CSV의 `speed` 값.

### 출력

```text
/planning/speed_profile
```

### 기본 개념

```text
직선 구간 → 빠르게
코너 구간 → 느리게
급커브 → 더 느리게
```

### 예시 CSV

```csv
x,y,speed
-3.8,0.2,0.6
-2.0,0.2,1.2
0.0,0.2,1.5
2.0,0.2,0.8
```

### 스터디 포인트

```text
curvature 계산
속도 계획
코너 감속
직선 가속
lap time 개선
```

### 추천도

```text
실차 중요도: 높음
스터디 가치: 높음
구현 난이도: 중간
```

---

## 7. `lap_timer_node`

### 목적

한 바퀴 주행 시간을 측정한다.

### 입력

```text
/localization/odom
/planning/lap_progress
```

### 출력

```text
/evaluation/lap_time
/evaluation/lap_count
```

### 활용 예시

```text
lookahead 0.8 → lap time 24.3s
lookahead 1.2 → lap time 22.1s
max_speed 1.5 → collision
```

### 추천도

```text
실차 중요도: 중간
스터디 가치: 높음
구현 난이도: 낮음
```

---

## 8. `data_logger_node`

### 목적

주행 데이터를 CSV로 저장한다.

### 저장할 데이터

```text
time
x, y, yaw
current_speed
target_speed
steering_angle
lookahead_point
nearest_waypoint
min_front_distance
stop_required
collision 여부
```

### 출력 예시

```text
logs/run_001.csv
logs/run_002.csv
```

### 활용

```text
속도 그래프
조향각 그래프
경로 추종 오차
충돌 시점 분석
lap time 비교
보고서 작성
```

### 추천도

```text
실차 중요도: 중간
스터디 가치: 매우 높음
구현 난이도: 낮음
```

---

# 14.3 3순위 — 실차와 대회용 확장

## 9. `scan_preprocessor_node`

### 목적

LiDAR 데이터를 바로 쓰지 않고 전처리한다.

### 입력

```text
/scan
```

### 출력

```text
/scan/filtered
/perception/front_min_distance
/perception/obstacle_sectors
```

### 하는 일

```text
NaN 제거
inf 제거
range clipping
moving average filter
전방 sector 추출
좌우 거리 계산
장애물 거리 계산
```

### 추천도

```text
실차 중요도: 높음
스터디 가치: 매우 높음
구현 난이도: 낮음~중간
```

---

## 10. `particle_filter_node` 또는 `map_localization_node`

### 목적

실차에서 맵 기준 차량 위치를 추정한다.

현재 localization은 시뮬 odom relay다.

```text
/ego_racecar/odom → /localization/odom
```

실차에서는 맵 기준 위치 추정이 필요하다.

### 입력

```text
/map
/scan
/initialpose
```

### 출력

```text
/localization/odom
/localization/pose
```

또는:

```text
/pf/pose/odom
/pf/viz/inferred_pose
```

### 후보 방식

```text
Particle Filter
AMCL
slam_toolbox localization mode
ICP / NDT localization
```

### 스터디 포인트

```text
Bayes Filter
motion model
sensor model
resampling
map-based localization
```

### 추천도

```text
실차 중요도: 매우 높음
스터디 가치: 매우 높음
구현 난이도: 높음
```

---

## 11. `gap_follow_node`

### 목적

LiDAR 기반 reactive obstacle avoidance를 수행한다.

Pure Pursuit는 경로 추종이고, 장애물 회피는 못 한다.
랜덤 장애물이나 벽 회피를 위해 gap follow가 필요하다.

### 입력

```text
/scan/filtered
```

### 출력

```text
/planning/reactive_target
```

또는:

```text
/control/reactive_cmd
```

### 알고리즘 개념

```text
1. LiDAR에서 가까운 장애물 찾기
2. 장애물 주변에 bubble 생성
3. 가장 큰 free gap 찾기
4. gap 중앙 또는 가장 먼 점 선택
5. 해당 방향으로 조향
```

### 스터디 포인트

```text
Follow-the-Gap
disparity extender
obstacle bubble
largest free gap
reactive planning
```

### 추천도

```text
실차 중요도: 중간~높음
스터디 가치: 매우 높음
구현 난이도: 중간
```

---

## 12. `behavior_selector_node`

### 목적

상황에 따라 주행 모드를 선택한다.

### 예시

```text
장애물 없음 → Pure Pursuit
앞쪽 장애물 있음 → Gap Follow
너무 가까움 → Emergency Brake
RViz goal 테스트 → Goal Planner
```

### 입력

```text
/safety/stop_required
/planning/global_target
/planning/reactive_target
/goal_pose
```

### 출력

```text
/planning/selected_target
```

또는:

```text
/control/selected_cmd
```

### 구조

```text
/safety/stop_required
/planning/global_target
/planning/reactive_target
        ↓
behavior_selector_node
        ↓
/planning/selected_target
```

### 추천도

```text
실차 중요도: 높음
스터디 가치: 매우 높음
구현 난이도: 중간~높음
```

---

# 14.4 4순위 — 실차 보정과 고급 제어

## 13. `calibration_node` 또는 `vehicle_calibration_tool`

### 목적

실차의 모터와 조향 값을 보정한다.

실차에서는 `0.5`가 항상 직진이 아닐 수 있다.
또한 ERPM과 실제 속도의 관계도 차량마다 다르다.

### 보정할 값

```yaml
servo_center: 0.5
servo_gain: 1.0
servo_min: 0.0
servo_max: 1.0

speed_to_erpm_gain: 3000.0
speed_to_erpm_offset: 0.0
```

### 실차 테스트 순서

```text
1. 바퀴를 띄운다
2. servo position 0.5를 보낸다
3. 직진인지 확인한다
4. 0.48, 0.50, 0.52 등을 테스트한다
5. 실제 직진값을 servo_center로 저장한다
6. 낮은 ERPM으로 모터 테스트한다
```

### 추천도

```text
실차 중요도: 매우 높음
스터디 가치: 중간
구현 난이도: 낮음
```

---

## 14. `watchdog_node`

### 목적

명령이 끊겼을 때 차량을 자동 정지시킨다.

실차에서는 마지막 속도 명령이 유지되는 상황이 위험할 수 있다.

### 입력

```text
/control/drive_cmd
```

또는 현재 구조에서는:

```text
/drive
```

### 출력

```text
/vehicle/drive_cmd_safe
```

또는 직접:

```text
/commands/motor/speed
/commands/servo/position
```

### 기본 로직

```text
마지막 command 수신 시간 확인
        ↓
0.3초 이상 새 명령 없음
        ↓
speed = 0
servo = center
```

### 추천도

```text
실차 중요도: 매우 높음
스터디 가치: 높음
구현 난이도: 낮음
```

---

## 15. `mpc_node`

### 목적

Pure Pursuit보다 더 정교한 경로 추종을 위해 Model Predictive Control을 적용한다.

### 입력

```text
/localization/odom
/planning/path
/planning/speed_profile
```

### 출력

```text
/control/drive_cmd
```

### 필요성

Pure Pursuit는 간단하고 빠르지만 고속 코너에서 불안정할 수 있다.
MPC는 차량 모델을 이용해서 미래 경로를 예측하며 제어한다.

### 스터디 포인트

```text
vehicle dynamics
kinematic bicycle model
optimization
constraints
CasADi
```

### 추천도

```text
실차 중요도: 중간~높음
스터디 가치: 매우 높음
구현 난이도: 높음
```

---

# 15. 추천 최종 노드 구조

최종적으로는 다음 구조가 가장 좋다.

```text
f1tenth_autonomy
├── localization
│   ├── odom_relay_node
│   └── particle_filter_node
│
├── planning
│   ├── waypoint_planner_node
│   ├── goal_pose_planner_node
│   ├── velocity_profile_node
│   └── waypoint_manager_node
│
├── control
│   ├── pure_pursuit_node
│   └── mpc_node
│
├── safety
│   ├── scan_preprocessor_node
│   ├── safety_brake_node
│   └── watchdog_node
│
├── vehicle_interface
│   └── drive_adapter_node
│
├── behavior
│   └── behavior_selector_node
│
├── tools
│   ├── lap_timer_node
│   ├── data_logger_node
│   └── debug_marker_node
│
└── bringup
    ├── sim_autonomy.launch.py
    ├── real_autonomy.launch.py
    └── debug.launch.py
```

---

# 16. 현재 구조에서 추천하는 개선 구조

## 현재 구조

```text
localization_node
        ↓
waypoint_planner_node
        ↓
pure_pursuit_node
        ↓
/drive 또는 /commands/...
```

## 개선 구조

```text
localization_node or particle_filter_node
        ↓
waypoint_manager_node
        ↓
behavior_selector_node
        ↓
pure_pursuit_node
        ↓
/control/drive_cmd
        ↓
vehicle_interface_node
        ↓
sim:  /drive
real: /commands/motor/speed
      /commands/servo/position
```

안전 계층은 옆에서 항상 감시한다.

```text
/scan
  ↓
scan_preprocessor_node
  ↓
safety_brake_node
  ↓
/safety/stop_required
  ↓
control or vehicle_interface
  ↓
stop override
```

---

# 17. 구현 우선순위

## Step 1 — 지금 바로 추가 추천

```text
1. safety_brake_node
2. goal_pose_planner_node
3. vehicle_interface_node
4. debug_marker_node
```

이유:

```text
safety_brake_node
    → 벽에 박는 문제를 줄임
    → 실차 안전성 확보

goal_pose_planner_node
    → RViz goal pose를 path로 변환
    → 스터디와 디버깅에 좋음

vehicle_interface_node
    → sim / real 출력 구조를 깔끔하게 분리
    → 실차 이식성 향상

debug_marker_node
    → RViz에서 상태를 보기 좋게 표시
    → 팀원 교육과 발표에 좋음
```

---

## Step 2 — 한 바퀴 주행 성능 개선

```text
5. waypoint_manager_node
6. velocity_profile_node
7. lap_timer_node
8. data_logger_node
```

이유:

```text
waypoint_manager_node
    → 경로 추종 안정성 증가

velocity_profile_node
    → 코너 감속, 직선 가속 가능

lap_timer_node
    → 주행 성능 비교 가능

data_logger_node
    → 튜닝 근거 확보
```

---

## Step 3 — 실차와 대회용

```text
9. scan_preprocessor_node
10. particle_filter_node
11. gap_follow_node
12. behavior_selector_node
13. calibration tool
14. watchdog_node
15. mpc_node
```

이유:

```text
scan_preprocessor_node
    → 실차 LiDAR 노이즈 대응

particle_filter_node
    → 맵 기반 위치 추정

gap_follow_node
    → 장애물 회피 가능

behavior_selector_node
    → 상황별 주행 모드 선택

calibration tool
    → servo / motor 실차 보정

watchdog_node
    → 명령 끊김 안전정지

mpc_node
    → 고속 주행 성능 개선
```

---

# 18. 당장 다음에 만들 패키지 추천

가장 먼저 만들 패키지는 `safety`가 좋다.

## 패키지 구조

```text
src/safety
├── package.xml
├── setup.py
├── safety
│   ├── __init__.py
│   └── safety_brake_node.py
├── launch
│   └── safety.launch.py
└── config
    └── params.yaml
```

## `safety_brake_node` 토픽 구조

```text
subscribe:
  /scan
  /localization/odom

publish:
  /safety/stop_required
  /safety/min_front_distance
```

## control과 연결

```text
/safety/stop_required == true
        ↓
pure_pursuit_node speed = 0
```

더 좋은 구조는 다음과 같다.

```text
pure_pursuit_node
        ↓
/control/drive_cmd
        ↓
vehicle_interface_node
        ↓
safety override
        ↓
/drive or /commands/...
```

---

# 19. 실차 주행까지 가는 현실적인 순서

## 이번 주

```text
1. 팀원 PC에서 GitHub clone
2. docker build 성공 확인
3. rocker 실행 확인
4. RViz 시뮬 실행 확인
5. autonomy.launch.py 실행 확인
6. /drive publish 확인
7. drive_mode:=real에서 /commands 토픽 확인
```

---

## 다음 주

```text
1. 실차 컴퓨터 SSH 접속
2. ROS2 환경 확인
3. /scan 확인
4. /commands/motor/speed 확인
5. /commands/servo/position 확인
6. 바퀴 띄우고 servo / motor 테스트
7. servo_center 보정
8. speed_to_erpm_gain 보정
9. SLAM으로 맵 제작
10. localization 확인
11. raceline.csv 준비
12. drive_mode:=real 저속 주행 테스트
```

---

# 20. 실차 주행 전 체크리스트

```text
[ ] 바퀴 띄우고 motor 테스트
[ ] servo_center 찾기
[ ] emergency stop 준비
[ ] max_speed 0.5 이하로 시작
[ ] /scan 정상 확인
[ ] /localization/odom 정상 확인
[ ] /planning/path 정상 확인
[ ] /commands/motor/speed publish 확인
[ ] /commands/servo/position publish 확인
[ ] safety_brake_node 동작 확인
[ ] 트랙 주변 사람 없음
```

초기 실차 파라미터는 보수적으로 시작한다.

```yaml
target_speed: 0.3
min_speed: 0.0
max_speed: 0.5
lookahead_distance: 1.0
servo_center: 0.5
speed_to_erpm_gain: 2000.0
```

---

# 21. 최종 요약

현재 완료된 것:

```text
GitHub repo 배포 가능
Dockerfile 안정화
Docker build 성공
ROS2 Humble F1TENTH 시뮬 환경 구성
localization_node 구현
waypoint_planner_node 구현
pure_pursuit_node 구현
f1tenth_bringup 통합 launch 구현
drive_mode:=sim / real 출력 분리 구현
usage.md 작성
```

현재 한계:

```text
벽 회피 없음
safety brake 없음
goal pose 미연결
실제 트랙 waypoint 없음
실차 localization 없음
calibration 없음
data logging 없음
```

다음에 추가할 핵심 노드:

```text
1. safety_brake_node
2. goal_pose_planner_node
3. vehicle_interface_node
4. debug_marker_node
5. waypoint_manager_node
6. velocity_profile_node
7. data_logger_node
8. lap_timer_node
9. particle_filter_node
10. gap_follow_node
11. behavior_selector_node
12. watchdog_node
13. calibration tool
14. mpc_node
```

가장 먼저 할 일:

```text
safety_brake_node 추가
```

이유:

```text
현재 벽에 부딪힘
실차에서 가장 위험한 부분
LiDAR 공부 가능
구현 난이도 낮음
효과 큼
```

