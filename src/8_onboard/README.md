# F1TENTH 온보드 자율주행

ROS 2 Humble 기반 실차용 저장소입니다. `track03`에서 AMCL 위치 추정,
전역 raceline 추종, LiDAR 정적 장애물 회피와 제어기 비교를 수행합니다.

- 시뮬레이션: [Kimz1xq/f1tenth](https://github.com/Kimz1xq/f1tenth)
- 실차 출력: `/auto` → Ackermann mux → VESC
- 지도: `maps/track03.yaml`
- 기본 실차 제어기: `forza_map`
- 제어기는 항상 비활성 상태로 시작하며 `/control/enable`로 켭니다.

## 구성

```text
f1tenth-onboard/
├── autonomy_ws/src/
│   ├── planning/          raceline 발행, LiDAR 정적 장애물 회피, AEB
│   ├── control/           Pure Pursuit, UNICORN L1, Forza MAP, MPC, MPCC
│   └── f1tenth_bringup/   sim/real 공통 launch
├── vehicle_overrides/     온보드 bringup·VESC·joystick 보정본
├── maps/track03.{pgm,yaml}
├── config/amcl.yaml
└── run_autonomy.sh        중복·고아 autonomy 세션 방지 실행기
```

`build/`, `install/`, `log/`, bag, 결과 CSV는 Git에 저장하지 않습니다.
VESC 하드웨어 보정값은 `vehicle_overrides/`에 보관하며 제어기 실험 때 변경하지
않습니다.

## 제어기

| 이름 | 방식 | 용도 |
|---|---|---|
| `pure_pursuit` | Pure Pursuit | 기존 저속 기준선 |
| `kyeongho_pp` | Pure Pursuit + speed cap/watchdog | `kye0ngho/f1_tenth` 실차 어댑터 |
| `unicorn_l1` | 속도·곡률 기반 L1/Pursuit | UNICORN 계열 비교 |
| `forza_map` | Model- and Acceleration-based Pursuit | 기본 실차 검증 |
| `mpc` | Linear MPC | MPC 기준선 |
| `mpcc` | Nonlinear MPCC | 실험용 |

`UNICORN L1`과 `Forza MAP`은 MPC가 아닙니다. `Forza MAP`은
[ForzaETH Race Stack](https://github.com/ForzaETH/race_stack/tree/ros2-humble)의
MAP 전략을 현재 ROS 2 토픽과 TF에 연결한 어댑터입니다.

## 한 번만: 빌드

온보드 컨테이너에서 코드가 바뀐 경우 실행합니다.

```bash
cd /home/misys/shared_dir/autonomy_ws
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash

colcon build --symlink-install --packages-select \
  planning control f1tenth_bringup

source install/setup.bash
```

## 공통 접속

현재 사용하는 두 실차 온보드 접속 주소입니다.

| 차량 | SSH 접속 |
|---|---|
| 차량 A | `jeonbotdae@172.20.10.10` |
| 차량 B | `misys@172.20.10.9` |

차량 A:

```bash
ssh -tt jeonbotdae@172.20.10.10 \
  'docker start f1tenth >/dev/null 2>&1 || true; docker exec -it f1tenth bash'
```

차량 B:

```bash
ssh -tt misys@172.20.10.9 \
  'docker start f1tenth >/dev/null 2>&1 || true; docker exec -it f1tenth bash'
```

온보드 컨테이너의 새 터미널에는 다음 환경이 자동 적용됩니다.

```text
ROS_DOMAIN_ID=30
ROS_LOCALHOST_ONLY=0
ROS2CLI_NO_DAEMON=1
```

각 온보드 터미널에서 실행할 공통 source 명령입니다.

```bash
source /opt/ros/humble/setup.bash
source /home/misys/f1tenth_ws/install/setup.bash
source /home/misys/shared_dir/autonomy_ws/install/setup.bash
```

## 실차 실행 순서

아래 터미널은 서로 별개입니다. 터미널 1~4와 6은 온보드 컨테이너이며,
터미널 5의 RViz만 노트북에서 실행합니다.

### 터미널 1 — 차량 Bringup

```bash
ros2 launch f1tenth_stack bringup_launch.py
```

정상 로그:

```text
Opened joystick: Wireless Controller
Connected to VESC
Connected to a network device
```

### 터미널 2 — Map Server

```bash
ros2 run nav2_map_server map_server --ros-args \
  -r __node:=map_server \
  -p yaml_filename:=/home/misys/shared_dir/maps/track03.yaml \
  -p topic:=map \
  -p frame_id:=map \
  -p use_sim_time:=false
```

`Creating`에서 대기하는 것은 정상입니다.

### 터미널 3 — AMCL

```bash
ros2 run nav2_amcl amcl --ros-args \
  -r __node:=amcl \
  --params-file /home/misys/shared_dir/config/amcl.yaml
```

### 터미널 4 — Lifecycle 활성화 및 상태 확인

```bash
for node in map_server amcl; do
  until ros2 lifecycle get /$node >/dev/null 2>&1; do sleep 1; done
  state=$(ros2 lifecycle get /$node | awk '{print $1}')
  [ "$state" = "unconfigured" ] && ros2 lifecycle set /$node configure
  state=$(ros2 lifecycle get /$node | awk '{print $1}')
  [ "$state" = "inactive" ] && ros2 lifecycle set /$node activate
done

ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
```

둘 다 `active [3]`이어야 합니다.

### 터미널 5 — 노트북 RViz

노트북 호스트에서 실행합니다. 실차 검증 중에는 시뮬레이터 launch를 같이
실행하지 않습니다.

```bash
xhost +si:localuser:root

docker exec -it \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=30 \
  -e ROS_LOCALHOST_ONLY=0 \
  f1tenth_gym_ros_humble-sim-1 \
  bash -lc '
    source /opt/ros/humble/setup.bash
    source /sim_ws/install/setup.bash
    exec rviz2 -d /sim_ws/install/f1tenth_gym_ros/share/f1tenth_gym_ros/launch/gym_bridge.rviz
  '
```

RViz에서 `2D Pose Estimate`로 실제 차량 위치와 방향을 지정합니다. 빨간
LaserScan이 검은 지도 벽과 일치해야 합니다.

```bash
ros2 run tf2_ros tf2_echo map base_link
```

`map → base_link`가 연속 출력되는지 확인하고 `Ctrl+C`를 누릅니다.

### 터미널 6 — 자율주행 노드

처음에는 `1.0 m/s`로 확인합니다. `maximum_speed`는 launch에서 허용할
명령 상한이며 VESC나 차량의 실제 성능을 높이는 설정은 아닙니다.

```bash
cd /home/misys/shared_dir

./run_autonomy.sh \
  mode:=real \
  track:=track03 \
  controller:=forza_map \
  speed:=1.0 \
  maximum_speed:=20.0 \
  steering_lookup_table:=auto
```

실차에서는 장애물 플래너가 항상 실행됩니다. 장애물이 없으면 전역경로를
그대로 사용하고, 정적 장애물이 감지되면 회피경로로 자동 전환합니다.

다른 제어기는 `controller` 값만 바꿉니다.

```text
controller:=pure_pursuit
controller:=kyeongho_pp
controller:=unicorn_l1
controller:=mpc
controller:=mpcc
```

`kyeongho_pp` 실차 실행 예시:

```bash
cd /home/misys/shared_dir

./run_autonomy.sh \
  mode:=real \
  controller:=kyeongho_pp \
  speed:=1.0 \
  maximum_speed:=20.0 \
  waypoint_csv:=/home/misys/shared_dir/autonomy_ws/install/planning/share/planning/waypoints/track03_raceline.csv \
  min_command_speed:=0.30
```

이 어댑터는 [kye0ngho/f1_tenth](https://github.com/kye0ngho/f1_tenth)의
Pure Pursuit, 회피 속도 제한, stall watchdog 및 kill-switch 구조를 참고합니다.
원본처럼 VESC ERPM·서보 토픽을 직접 출력하지 않고 `/auto`에
`AckermannDriveStamped`를 발행하므로 기존 mux와 차량 캘리브레이션을 유지합니다.

### 터미널 4 — 주행 시작/정지

주행 시작:

```bash
ros2 service call /control/enable \
  std_srvs/srv/SetBool "{data: true}"
```

주행 정지:

```bash
ros2 service call /control/enable \
  std_srvs/srv/SetBool "{data: false}"
```

## 주행 전 최소 확인

```bash
ros2 lifecycle get /map_server
ros2 lifecycle get /amcl
timeout 5 ros2 topic hz /scan
timeout 5 ros2 topic hz /odom
ros2 run tf2_ros tf2_echo map base_link
```

주행 조건:

- `map_server`, `amcl`: `active [3]`
- TF: `map → odom → base_link → laser`
- `/scan`: 약 40 Hz
- RViz의 LaserScan과 지도 벽이 일치
- `AEB_STOP`, `NO_COLLISION_FREE_PATH`, scan timeout이 없음

첫 검증은 바퀴를 띄운 상태에서 조향 방향을 확인하고, 넓은 공간에서
`1.0 m/s`로 확인한 뒤 속도를 단계적으로 높입니다.

## 전체 초기화

이전 lifecycle, TF cache, AEB 상태 또는 autonomy 프로세스가 남았을 때
노트북 호스트에서 실행합니다. 실행 중인 온보드 터미널은 종료됩니다.

```bash
# 차량 A
ssh -t jeonbotdae@172.20.10.10 'docker restart f1tenth'

# 차량 B
ssh -t misys@172.20.10.9 'docker restart f1tenth'

docker restart f1tenth_gym_ros_humble-sim-1
```

초기화 후 터미널 1부터 다시 실행합니다.

## 핵심 문제 확인

| 증상 | 확인 사항 |
|---|---|
| RViz에서 맵이 안 보임 | Map QoS=`Reliable`, `Transient Local` |
| `AEB_SCAN_TIMEOUT` | `/scan` 발행과 `odom → laser` TF |
| `NO_COLLISION_FREE_PATH` | AMCL 초기 자세와 Scan/지도 정렬 |
| `extrapolation into the future` | 노트북·온보드 NTP 동기화 |
| 두 번째 실행부터 이상함 | `run_autonomy.sh` 사용 또는 전체 초기화 |

맵을 다시 발행해야 할 때만 다음을 사용합니다.

```bash
ros2 lifecycle set /map_server deactivate
ros2 lifecycle set /map_server activate
```

## Sim-to-real 기준

시뮬레이션과 실차는 동일한 `planning`, `control`, `f1tenth_bringup`, 지도와
raceline을 사용합니다. 차이는 `mode`와 하드웨어 토픽/TF뿐입니다.

| 항목 | 시뮬레이션 | 실차 |
|---|---|---|
| base frame | `ego_racecar/base_link` | `base_link` |
| odometry | `/ego_racecar/odom` | `/odom` |
| drive | `/drive` | `/auto` |
| 초기 자세 | 자동 | RViz `2D Pose Estimate` |

동일 속도로 먼저 시뮬레이션하고 실차에서는 lap time, CTE, safety stop,
충돌, 조향 saturation을 기록해 비교합니다.
