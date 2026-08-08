# 2026 전BOT대 Roboracer

## 파이프라인

1. **Mapping** — SLAM으로 맵 생성 → 전역 경로 최적화 → CSV 저장
2. **Tracking** — CSV(목표 경로) + Localization(현재 위치) → 오차 계산 → 조향/속도 명령
3. **Avoidance** — 전역 경로는 유지, 장애물 구간만 로컬 경로 생성

## 폴더 구조

~~~
2026_Roboracer_JBD/
├── README.md
└── src/
    ├── 0_EKF/                    # 휠 오도메트리 + IMU 융합 → odom -> base_link
    │   ├── local_ekf.yaml        #   robot_localization 파라미터
    │   ├── imu_relay.py          #   VESC raw IMU → EKF 입력 형식 변환
    │   ├── run_ekf_bag.sh        #   bag 으로 EKF 단독 검증
    │   └── compare_ekf.py        #   EKF 출력 vs 원본 odom 궤적 비교
    ├── 1_slam_mapping/           # SLAM → 맵 저장
    │   ├── slam_toolbox/         #   slam_toolbox launch + 파라미터
    │   ├── maps/                 #   맵(.pgm/.yaml), test_bags/
    │   └── slam_analysis_to_map/ #   맵 품질 분석 스크립트
    ├── 2_global_planner/         # 전역 경로 최적화 → CSV
    ├── 3_localization/           # map 기준 현재 위치 추정 (AMCL 등)
    ├── 4_local_planner/          # 장애물 회피 로컬 경로
    ├── 5_controller/             # Pure Pursuit / MPC
    └── 6_bringup/                # 여러 단계를 엮는 launch 파일
        ├── ekf_slam_bag_launch.py #  EKF + slam_toolbox + RViz 한 번에 실행
        └── ekf_slam.rviz          #  위 실행에 쓰는 RViz 화면 설정
~~~

`0_EKF` 는 `odom -> base_link` 를 만든다. 맵 없이 도는 로컬 오도메트리라
매핑 때도 주행 때도 계속 쓰이므로 파이프라인 앞(0번)에 둔다.
`3_localization` 은 맵이 있어야 도는 `map -> odom` 담당이다.

## 실행

~~~bash
# bag 재생 기반 EKF + slam_toolbox + RViz
ros2 launch src/6_bringup/ekf_slam_bag_launch.py
ros2 launch src/6_bringup/ekf_slam_bag_launch.py bag:=0808_test_3 play:=true

# EKF 단독 검증 (bag → /odometry/filtered 녹화 → 궤적 비교)
./src/0_EKF/run_ekf_bag.sh 0808_test_2
python3 src/0_EKF/compare_ekf.py 0808_test_2

# slam_toolbox 단독
ros2 launch src/1_slam_mapping/slam_toolbox/offline_launch.py
~~~

## 6_bringup 안의 두 파일

한 쌍이다. 하나는 **무엇을 실행할지**, 하나는 **무엇을 화면에 띄울지** 담당한다.

### `ekf_slam_bag_launch.py` — 실행 대본

이 실험은 노드가 5개 필요하다. 런치 파일이 없으면 터미널 5개를 열어야 한다.

~~~bash
터미널1  python3 imu_relay.py --ros-args -p in_topic:=/sensors/imu/raw ...
터미널2  ros2 run robot_localization ekf_node --ros-args --params-file local_ekf.yaml ...
터미널3  ros2 run slam_toolbox sync_slam_toolbox_node --ros-args --params-file ...
터미널4  rviz2 -d ekf_slam.rviz
터미널5  ros2 bag play 0808_test_2 --clock 100 --topics /scan /odom ...
~~~

이 5줄을 파일 하나에 적어둔 것이고, 그래서 `ros2 launch` 한 줄로 끝난다.

인자:

| 인자 | 기본값 | 설명 |
|---|---|---|
| `bag` | `0808_test_2` | 재생할 bag 이름 (`maps/test_bags/` 안) |
| `rate` | `1.0` | bag 재생 속도 |
| `play` | `false` | `true` 면 런치가 bag 재생까지 한다 |
| `rviz` | `true` | RViz 띄울지 |

**TF 소유권** — 이 런치의 핵심 설계다. 겹치면 TF 트리가 깨진다.

| 변환 | 담당 |
|---|---|
| `map -> odom` | slam_toolbox |
| `odom -> base_link` | ekf_filter_node |
| `base_link -> laser` | bag 의 `/tf_static` |

bag 안의 `/tf` 는 VESC 가 만든 `odom -> base_link` 라서 EKF 와 충돌한다.
그래서 재생 토픽을 `/scan /odom /sensors/imu/raw /tf_static` 으로 제한한다.

### `ekf_slam.rviz` — 화면 배치도

RViz 는 그냥 켜면 빈 화면이라 볼 항목을 매번 손으로 추가해야 한다.
그 세팅을 저장해둔 파일이고, 런치가 `rviz2 -d` 로 넘겨준다.

| 표시 항목 | 토픽 | 내용 |
|---|---|---|
| Grid | — | 바닥 격자 |
| Map | `/map` | slam_toolbox 가 만드는 지도 |
| LaserScan | `/scan` | 라이다 점 |
| EKF odom (filtered) | `/odometry/filtered` | EKF 추정 위치 (화살표) |
| VESC odom (raw) | `/odom` | 원본 휠 오도메트리 (화살표) |
| TF | — | 좌표계 (map, odom, base_link, laser) |

Fixed Frame 은 `map`, 시점은 `TopDownOrtho` (위에서 내려다보는 2D).

화살표가 두 종류인 게 핵심이다. **EKF 결과와 VESC 원본을 나란히 놓고 눈으로 비교**하려고
만든 화면이다. `compare_ekf.py` 가 숫자로 하는 일을 이건 그림으로 한다.

표시 항목을 바꾸려면 RViz 에서 화면을 꾸민 뒤 **File → Save Config As** 로 덮어쓴다.

## 브랜치 규칙

- `main` : 차에 올려서 검증된 코드만
- `feat/*` : 기능 개발 중
- `fix/*` : 버그 수정
- `exp/*` : 실험
