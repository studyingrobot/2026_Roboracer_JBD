# MPC + 로컬 회피 통합 진행 상황

**중단 시점**: 2026-08-19
**목표**: `2_global_planner` 가 뽑은 최적 전역경로 CSV 를 MPC 로 추종해서 시뮬에서 주행

---

## 어디까지 했나

| # | 단계 | 상태 |
|---|---|---|
| 1 | 전역경로 CSV 완성 | ✅ |
| 2 | 어떤 코드를 쓸지 결정 | ✅ |
| 3 | 파일 8개 확보 | ✅ |
| 4 | import 경로 수정 | ✅ |
| 5 | 의존성 설치 (cvxpy, osqp) | ✅ |
| 6 | 회피 코어 유닛테스트 | ✅ 10/10 |
| 7 | MPC 로드 검증 | ✅ |
| 8 | `mpc_params.yaml` ROS 포맷 변환 | ◻️ **여기부터** |
| 9 | 시뮬레이터 설치 | ◻️ 남은 것 중 최대 |
| 10 | 경로 퍼블리시 → RViz 확인 | ◻️ |
| 11 | MPC dry-run | ◻️ |
| 12 | 속도 프로파일 연동 | ◻️ |
| 13 | 저속 주행 → 튜닝 | ◻️ |
| 14 | 회피 노드 투입 | ◻️ |

---

## 가져온 코드

출처: `github.com/Kimz1xq/f1tenth` (MIT, Copyright 2020 Hongrui Zheng)
클론하지 않고 필요한 파일만 `curl` 로 받음.

```
src/4_local_planner/
  waypoint_planner_node.py         178줄   CSV → nav_msgs/Path 퍼블리셔
  local_planner_core.py            273줄   회피 알고리즘 (numpy/math 만 의존)
  local_obstacle_planner_node.py   871줄   ROS 배관 (scan/map/TF/마커)
  test_local_planner_core.py       135줄   유닛테스트

src/5_controller/
  linear_mpc_node.py               703줄   Linear Time-Varying MPC
  mpc_params.yaml                          파라미터 (아직 ROS 포맷 아님)
  MPC_GUIDE.md                             원저자 튜닝 가이드
  LICENSE-MIT                              MIT 고지
```

전부 원본과 바이트 단위 일치 확인함.

### 계보

```
F1TENTH Lab 7 템플릿 (xLab, MIT)
  └ jasonf27 제출본            수식 이해용 참고자료
      └ Kimz1xq linear_mpc_node.py   ← 실제로 쓰는 코드
```

`Kimz1xq/f1tenth-onboard` 의 같은 파일은 **바이트 단위로 동일**하지만 그쪽은
LICENSE 가 없어서 시뮬 레포(MIT) 에서 가져왔다.

### 검토했으나 쓰지 않기로 한 것

`HMCL-UNIST/unicorn-racing-stack` — ForzaETH race_stack 계열. 로컬 플래너만
떼어낼 수 없는 구조(f110_msgs + C++ frenet_conversion + perception +
state_machine 이 전부 딸려옴). 브랜치가 jazzy/ros1 뿐이라 humble 도 아니고,
LICENSE 파일도 없음. 나중에 도커로 격리해서 **참고자료로만** 볼 것.

---

## 수정한 것

`local_obstacle_planner_node.py:28`, `test_local_planner_core.py:5`

```python
from planning.local_planner_core import (...)   # 원본 (colcon 패키지 전제)
from local_planner_core import (...)            # 수정 후
```

이 레포는 `package.xml` 이 없는 평면 스크립트 구조라 `planning` 패키지가
존재하지 않는다. 접두사만 제거.

---

## 검증 결과

```
pytest test_local_planner_core.py       10 passed
python3 -c "import linear_mpc_node"     OK
설치됨: cvxpy 1.7.5, osqp 1.1.3, clarabel 0.11.1, scs 3.2.11
```

`local_planner_core.py` 는 ROS 없이 검증 완료. 나중에 회피가 이상하면
원인 후보에서 제외하고 시작할 것.

---

## 토픽 배선

```
[회피 없이 — 먼저 이걸로 주행]
  waypoint_planner ──/planning/path──> linear_mpc ──/drive──> 차

[회피 투입 — 파라미터 한 줄로 끼워넣음]
  waypoint_planner ──/planning/global_path──> local_obstacle_planner
                                                     │
                          /scan /map /odom ──────────┤
                                                     ├──/planning/path──> linear_mpc
                                                     └──/safety/emergency_stop──> linear_mpc
```

`local_obstacle_planner_node` 의 기본값이 `global_path_topic:/planning/global_path`,
`local_path_topic:/planning/path` 라서, 회피를 끼울 때 **waypoint_planner 의
`path_topic` 만 `/planning/global_path` 로 바꾸면 된다.** MPC 는 건드리지 않는다.

### MPC 가 요구하는 인터페이스

| 종류 | 이름 | 비고 |
|---|---|---|
| 구독 | `/ego_racecar/odom` | `twist.twist.linear.x` (속력) 만 사용 |
| 구독 | `/planning/path` | 닫힌 경로, map 프레임 |
| TF | `map -> base_link` | **위치·yaw 는 odom 이 아니라 TF 에서 읽음** |
| 구독 | collision / emergency_stop | 없으면 False 유지, 필수 아님 |
| 발행 | `/drive` | AckermannDriveStamped |
| 서비스 | `/control/enable` | 기본 disabled(dry-run), SetBool |

시뮬(f1tenth_gym_ros)은 gym_bridge 가 `map -> base_link` TF 를 직접 쏘므로
**시뮬 단계에서는 AMCL 없이 동작한다.** 실차 갈 때 `3_localization` 을 채워야 함.

---

## 다음에 할 일

### 8. `mpc_params.yaml` 변환

현재 파일은 ROS 파라미터 포맷이 아니다. 원저자의 `control.launch.py` 가
직접 파싱하는 커스텀 구조(`common:` / `speed_template:` / `profiles:`)라
`--params-file` 로 바로 못 먹인다. 이렇게 바꿔야 한다.

```yaml
linear_mpc_node:
  ros__parameters:
    enabled: false
    ...
```

`common` + 프로파일 하나(`tuned_v2` 권장)를 합치고 껍데기를 씌운다.
동시에 아래 값을 이 트랙에 맞게 고쳐야 한다.

| 파라미터 | 기본값 | 바꿀 값 | 이유 |
|---|---|---|---|
| `target_speed` | 0.55 | ~3.0 | 기본값은 이 CSV(2.97~7.49 m/s)에 비해 너무 느림 |
| `max_speed` | 0.80 | ~7.5 | 위와 동일 |
| `min_reference_speed` | 0.30 | ~2.5 | 위와 동일 |
| `horizon_steps` | 20 | ~10 | 20이면 5 m/s 에서 10 m 예측 = 랩(20.5 m)의 절반 |
| `max_acceleration` | 1.50 | 재검토 | `inputs/veh_dyn_info/ggv.csv` 와 맞춰야 함 |
| `wheelbase` | 0.33 | 확인 | `params/f110.ini` 값과 같은지 |

### 9. 시뮬레이터 설치

**공식** `f1tenth_gym_ros` 를 별도로 클론한다 (Kimz1xq 것은 그 레포 전용으로
개조돼 있음). 도커 / 네이티브 둘 다 가능 — ROS humble 과
`raceline-opt:py38` 이미지가 이미 있음.

- 맵: `src/1_slam_mapping/maps/0809_test_6_map.pgm` + `.yaml`
  (resolution 0.05, origin `[-3.63, -1.14, 0]`)
- 시작 자세: 레이스라인 첫 점 `(0.6399, 2.2083)` 근처
- 확인점: RViz 에 맵과 차가 뜨면 통과

### 10. 경로 퍼블리시

```bash
python3 src/4_local_planner/waypoint_planner_node.py --ros-args \
  -p waypoint_csv:=src/2_global_planner/0809_test_6_raceline.csv \
  -p path_topic:=/planning/path
```

헤더 없는 3컬럼 CSV 를 자동으로 `x, y, speed` 로 읽는다. **변환 불필요.**

확인점: RViz 에서 `/planning/markers` 초록 라인이 맵 위 트랙 안에 얹히는지.
어긋나면 맵 origin 정합 문제 — MPC 붙이기 전에 반드시 해결할 것.

### 11. MPC dry-run

```bash
python3 src/5_controller/linear_mpc_node.py --ros-args \
  --params-file src/5_controller/mpc_params.yaml
```

`enabled: false`, `solve_when_disabled: true` 로 둔다. 차는 움직이지 않는다.

확인점:
- `ros2 topic echo /mpc/proposed_drive` 에 값이 나옴
- `ros2 topic echo /mpc/solve_time_ms` 가 100 ms 미만
- RViz 에 `/mpc/reference_path`(주황), `/mpc/predicted_path`(파랑) 가
  트랙을 따라 앞으로 뻗음

### 12. 속도 프로파일 연동 ★

**dry-run 이 도는 걸 확인한 다음에 손댈 것.** 순서를 바꾸면 원인을 못 가린다.

문제: `nav_msgs/Path` 에 속도 필드가 없어서 `waypoint_planner_node` 가 CSV 의
vx 를 읽고도 버린다. MPC 는 `build_reference()` 에서 속도를 스스로 만든다.

```python
reference[2] = clip(target_speed / (1 + corner_slowdown_gain*|curvature|), ...)
```

즉 **TUM 옵티마이저가 뽑은 속도 프로파일이 통째로 무시된다.**

고칠 곳 4군데:

1. `waypoint_planner_node.py` `build_path_msg()` — `pose.pose.position.z = speed`
2. `linear_mpc_node.py:215` `path_callback()` — x, y 뽑을 때 z 도 같이
3. `linear_mpc_node.py:242` `set_closed_path()` — `self.path_speed` 배열 추가 (266~267행 근처)
4. `linear_mpc_node.py:450` 과 `:468` — `target_speed` 공식 대신
   `self.interpolate_path(self.path_speed, sample_s)` 사용

**두 곳 다 바꿔야 한다.** 450행은 예측 구간의 공간 샘플링 간격을, 468행은
속도 상태의 목표값을 정한다. 한쪽만 바꾸면 코너에서 경로를 잘라먹는다
(원본 주석에 그 이유가 적혀 있음).

간이 대안: `target_speed` / `max_speed` 만 올리고 곡률 감속에 맡긴다.
YAML 3줄이지만 최적화 결과의 절반이 날아간다.

### 13~14. 주행 및 회피

저속(`target_speed` 1.5 m/s)부터. 시작/정지:

```bash
ros2 service call /control/enable std_srvs/srv/SetBool "{data: true}"
ros2 service call /control/enable std_srvs/srv/SetBool "{data: false}"
```

튜닝 순서: `q_x`/`q_y`(경로오차) → `r_steering`/`rd_steering`(조향 억제)
→ `horizon_steps`. 자세한 건 `5_controller/MPC_GUIDE.md`.

---

## 이 트랙 실측값

`src/2_global_planner/0809_test_6_raceline.csv`

```
점 개수      104 (마지막 점 = 첫 점, 중복 1개)
랩 길이      20.50 m
점 간격      평균 0.197 m
속도         2.97 ~ 7.49 m/s (평균 5.09)
예상 랩타임  약 4.0 s
x 범위       0.195 ~ 8.731
y 범위       -0.636 ~ 3.025
```

주의사항:

- 마지막 행이 첫 행과 완전히 동일하다. `path_callback` 이 그 경우만 잘라내니
  문제없다. 단 **다른 위치에 중복 인접점이 있으면 `set_closed_path` 가
  콜백 안에서 예외를 던져 노드가 죽는다.** 현재 파일은 깨끗함.
- 랩이 20.5 m 로 짧다. 남의 파라미터를 그대로 쓰면 예측 지평이나 회피
  스플라인이 랩의 상당 부분을 차지한다.

### 전역경로 재추출이 필요할 때

현재 CSV 는 3컬럼(x, y, vx)이다. `helper_funcs_glob/src/export_traj_race.py` 를
보면 writer 가 두 개 있다.

- 38~41행: `s_m; x_m; y_m; psi_rad; kappa_radpm; vx_mps; ax_mps2` — 7컬럼 원본
- 59~61행: `traj_race[:, [1,2,5]]` — 지금 쓰는 3컬럼

Frenet 기반 회피나 곡률 정보가 필요해지면 7컬럼 쪽을 켜서 다시 뽑는다.
`main_globaltraj_f110.py:162` 의 `traj_ltpl_export` 주석을 풀면 트랙 폭
(`width_right_m`, `width_left_m`)과 법선벡터까지 나온다.
트랙 폭 원본은 `inputs/tracks/0809_test_6_map.csv` 에도 있다.

---

## 환경 메모

**IPv6 때문에 curl 이 무한정 매달린다.** GitHub 에서 뭘 받을 때 `-4` 를 붙이거나
영구 수정:

```bash
echo 'precedence ::ffff:0:0/96  100' | sudo tee -a /etc/gai.conf
```

**pytest 플러그인 충돌.** apt 의 pytest 6.2.5 와 pip `--user` 의 최신 anyio 가
부딪혀 `ModuleNotFoundError: No module named '_pytest.scope'` 가 난다.

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest ...   # 임시
pip install --user -U pytest                              # 영구
```

**`(.venv)` 를 빠져나온 상태에서 작업할 것.** 그건 `Raceline-Optimization` 전용
파이썬 3.10 환경이다. 여기에 MPC 의존성을 섞으면 나중에 전역경로를 다시
뽑을 때 꼬인다. MPC/회피 노드는 `rclpy` 가 있는 시스템 파이썬으로 돌아야 한다.

---

## 라이선스 현황

| 위치 | 라이선스 |
|---|---|
| `2_global_planner/Raceline-Optimization/` | LGPL v3 |
| `4_local_planner/`, `5_controller/` | MIT (Hongrui Zheng, 2020) |
| 나머지 | 자체 |

가져온 파일 상단에 출처 주석을 넣어둘 것.

```python
# Adapted from https://github.com/Kimz1xq/f1tenth (MIT). See LICENSE-MIT.
```
