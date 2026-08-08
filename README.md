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

## 브랜치 규칙

- `main` : 차에 올려서 검증된 코드만
- `feat/*` : 기능 개발 중
- `fix/*` : 버그 수정
- `exp/*` : 실험
