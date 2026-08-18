# 2_global_planner — 전역 경로 최적화

맵(`.pgm`/`.png`)에서 **최적 전역경로 CSV**를 뽑는 단계다. 주행 중 도는 ROS 노드가
아니라, 맵이 바뀔 때 한 번 돌리고 결과 CSV만 남기는 **오프라인 도구**다.

## 구성

```
2_global_planner/
├── README.md
├── run_optimization.sh          중심선 → 최적화 → 검증 한 번에
├── Raceline-Optimization/       ★ git subtree (CL2-UWaterloo, LGPL v3) — 직접 수정 금지
├── tools/
│   ├── Dockerfile               Python 3.8 실행 환경
│   └── run_globaltraj.py        opt_type 을 CLI 로 지정하는 런처
└── outputs/                     결과물 (git 추적 안 함)
```

## 전체 흐름

```
맵.pgm ──┐
         │ generate_centerline.py          (7_sim 쪽 스크립트)
         ▼
  중심선 CSV  +  최적화기 입력 CSV
                (# x_m,y_m,w_tr_right_m,w_tr_left_m)
                        │
                        │ main_globaltraj_f110.py   (Docker / Python 3.8)
                        ▼
              <name>_raceline.csv          (x,y,vx / 헤더 없음)
                        │
                        │ validate_raceline.py      (벽 안 뚫는지 검사)
                        ▼
              waypoint_planner_node → local planner → MPC
```

**포맷이 양쪽 끝에서 그대로 맞는다.** 변환 코드가 필요 없다.

- 입력: `generate_centerline.py --optimizer-output` 의 헤더가
  `# x_m,y_m,w_tr_right_m,w_tr_left_m` 로 최적화기 입력 형식과 완전히 동일하다.
- 출력: `export_traj_race_f110()` 이 `x, y, vx` 3컬럼을 헤더 없이 쓰는데,
  `waypoint_planner_node` 에 헤더 없는 CSV 를 `0=x, 1=y, 2=speed` 로 읽는
  폴백이 있어서 그대로 물린다.

## 사용법

```bash
./run_optimization.sh --map ../1_slam_mapping/maps/0809_test_6_map.yaml \
                      --name 0809_test_6 \
                      --start 0,0,0
```

결과는 전부 `outputs/` 에 남는다.

| 파일 | 내용 |
|---|---|
| `<name>_centerline.csv` | 중심선 (`x,y,yaw,curvature,speed`) |
| `<name>_optimizer_input.csv` | 최적화기 입력 (중심선 + 좌우 트랙폭) |
| `<name>_centerline.png` | 중심선 미리보기 |
| `<name>_raceline.csv` | **★ 최적 전역경로** (`x,y,vx`, 헤더 없음) |
| `<name>_raceline.png` | 검증 미리보기 |

주요 옵션:

```bash
--opt-type mincurv      최소곡률 (기본, 권장)
--opt-type shortest_path 최단경로
--start 0,0,3.14159     주행 방향이 반대로 나올 때 yaw 뒤집기
--half-width 0.65       최적화기에 넘길 좌우 반폭 상한
--skip-centerline       1단계 건너뛰고 기존 입력 CSV 재사용
```

### 선행 조건

1단계(중심선)와 3단계(검증)는 **f1tenth 시뮬 레포의 스크립트**를 쓴다.
아직 없으면 두 단계를 건너뛰고 2단계만 돈다.

```bash
# 시뮬 레포를 src/7_sim 에 두거나
export F1TENTH_SIM_DIR=/path/to/f1tenth
```

Docker 이미지는 최초 실행 시 자동으로 빌드된다 (약 2분). 수동 빌드:

```bash
docker build -f tools/Dockerfile -t raceline-opt:py38 .
```

## 최적화 방식별 동작 상태

`main_globaltraj_f110.py` 에는 4가지 모드가 있는데, **실제로 돌려본 결과는 다음과 같다.**

| opt_type | 상태 | 비고 |
|---|---|---|
| `mincurv` | ✅ 동작 확인 | 최소곡률. solver 0.02s, 전체 1.6s. **기본값이자 권장** |
| `shortest_path` | ✅ 동작 확인 | 최단경로. 예제 트랙에서 곡률 한계 초과 경고가 떴다 |
| `mincurv_iqp` | ❌ **동작 안 함** | 아래 참고 |
| `mintime` | ⚠️ 미검증 | 최소랩타임. CasADi/IPOPT 로 비선형 최적제어를 푼다. 오래 걸리고 차량 파라미터에 매우 민감 |

### `mincurv_iqp` 가 안 되는 이유 (upstream 버그)

```
TypeError: iqp_handler() missing 4 required positional arguments:
           'spline_len', 'psi', 'kappa', and 'dkappa'
```

`requirements.txt` 가 설치하는 `trajectory_planning_helpers` (0.79) 의
`iqp_handler()` 는 인자 4개를 더 받는데, `main_globaltraj_f110.py` 와
`main_globaltraj.py` 둘 다 옛 시그니처로 호출한다. **CL2-UWaterloo 레포 자체의
버전 불일치**이지 우리 설정 문제가 아니다.

`mincurv` 로 충분하므로 당장은 그냥 쓰지 않는다. 필요해지면 호출부에
누락된 인자를 채우는 패치를 `tools/run_globaltraj.py` 에 추가하면 된다.

## 반드시 손봐야 할 것 — 차량 파라미터

`Raceline-Optimization/params/f110.ini` 는 **추정치 덩어리**다. 파일 첫 줄에
개발자가 직접 이렇게 써놨다.

> "값을 모를 때는 대체로 `racecar.ini` 값의 1/10로 잡았다"

우리 차와 다른 부분:

| 항목 | f110.ini | 우리 MPC (`mpc_params.yaml`) |
|---|---|---|
| wheelbase | 0.275 + 0.275 = **0.55 m** | **0.33 m** |
| v_max | 15.0 m/s | max_speed 0.8 m/s 수준에서 검증 중 |

그리고 `inputs/veh_dyn_info/ggv.csv` 의 가감속 한계:

```
# v_mps, ax_max_mps2, ay_max_mps2
0.0, 12.0, 12.0      ← 12 m/s² ≈ 1.2 G
```

F1TENTH 실차에서 1.2G 는 과하다. **이걸 안 낮추면 실제로 낼 수 없는 속도
프로파일이 나온다.** 마찰계수 기준으로 다시 잡아야 한다 (시뮬 레포
`config/tracks.yaml` 에 `friction_mu: 1.0489` 가 기록되어 있으니 참고).

실측으로 맞출 항목: `wheelbase_front/rear`, `width`, `mass`, `v_max`,
`curvlim`, `ggv.csv`.

## 알아둘 점

**속도 프로파일이 현재 파이프라인에서 버려진다.** 최적화기가 계산한 `vx_mps` 를
`waypoint_planner_node` 가 읽기는 하지만, 발행하는 `nav_msgs/Path` 에는 속도
필드가 없어서 전달되지 않는다. MPC 는 자체 `target_speed` + 곡률 기반 감속으로
속도를 정한다. 최적화의 절반을 버리는 셈이라, `/planning/speed_profile` 토픽을
추가하는 게 개선 포인트다.

**`map_converter.ipynb` 는 쓰지 않는다.** 맵 → 중심선 기능이
`generate_centerline.py` 와 겹치는데, 그쪽이 CLI 라 자동화가 쉽고 골격 사이클
예외처리도 더 꼼꼼하다.

**`Raceline-Optimization/` 은 직접 고치지 않는다.** git subtree 라서 나중에
`git subtree pull` 할 때 충돌한다. 동작을 바꿔야 하면 `tools/` 쪽에서 감싼다.
`tools/run_globaltraj.py` 가 `opt_type` 을 메모리에서만 치환하는 것도 그래서다.

업데이트를 받아오려면:

```bash
git subtree pull --prefix=src/2_global_planner/Raceline-Optimization \
                 raceline-opt master --squash
```

## 라이선스

`Raceline-Optimization/` 은 **LGPL v3** 다 (이 레포의 나머지, 그리고 f1tenth
시뮬 레포의 MIT 와 다르다). 별도 프로세스로 실행되는 오프라인 도구라 우리 ROS
코드와 링크되지 않지만, 대회 제출물에 포함된다면 조건을 확인할 것.
`Raceline-Optimization/LICENSE` 는 지우지 말 것.

원본: <https://github.com/CL2-UWaterloo/Raceline-Optimization>
(TUM 의 `global_racetrajectory_optimization` 을 F1TENTH 용으로 포크한 것)
