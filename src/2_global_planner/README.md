# 2_global_planner — 전역 경로 최적화

맵(`.pgm`)에서 **최적 전역경로 CSV**를 뽑는 단계다. 주행 중 도는 ROS 노드가 아니라,
맵이 바뀔 때 한 번 돌리고 결과 CSV만 남기는 **오프라인 도구**다.

## 구성

```
2_global_planner/
├── README.md
├── 0809_test_6_raceline.csv     ★ 결과물 (x, y, vx / 헤더 없음)
└── Raceline-Optimization/       CL2-UWaterloo 클론 (LGPL v3), .git 제거 후 그대로 품음
    ├── map_converter.ipynb        맵 → 중심선 CSV
    ├── main_globaltraj_f110.py    중심선 → 레이싱라인 CSV
    ├── params/f110.ini            차량 제원 (이미 1/10 스케일)
    ├── maps/                      입력 맵
    ├── inputs/tracks/             중심선 CSV
    ├── outputs/                   결과물 (클론 .gitignore 가 제외)
    └── .venv/                     Python 환경 (클론 .gitignore 가 제외)
```

`Raceline-Optimization/.gitignore` 가 살아 있어서 `.venv`(795 MB)와 `outputs/` 는
자동으로 git 추적에서 빠진다. 상위에서 따로 무시할 필요 없다.

## 전체 흐름

```
맵.pgm + 맵.yaml
      │ map_converter.ipynb        거리변환 → 스켈레톤 → DFS → 미터 변환
      ▼
중심선 CSV  (# x_m,y_m,w_tr_right_m,w_tr_left_m)
      │ main_globaltraj_f110.py    스플라인 → 법선 → QP(곡률최소) → 속도 프로파일
      ▼
raceline CSV  (x, y, vx / 헤더 없음)
      │
      ▼
5_controller (Pure Pursuit)
```

앞 단계가 **"트랙이 어디인가"**, 뒤 단계가 **"그 안에서 어디로 달릴 것인가"** 를 푼다.

---

## 1. 환경 구축 (최초 1회)

업스트림은 Python 3.8 기준이라 3.10 에서 그대로는 설치되지 않는다.
`requirements.txt` 의 버전 핀(`numpy==1.18.1` 등)이 소스 빌드로 들어가 깨진다.

```bash
cd Raceline-Optimization
sudo apt install python3-dev python3-tk python3.10-venv
python3 -m venv .venv && source .venv/bin/activate
pip install -U pip
pip install -r requirements-py310.txt      # 핀을 푼 사본
pip install ipykernel                      # VS Code 노트북용
```

`requirements-py310.txt` 는 원본에서 두 가지를 바꿨다.

- 모든 `==` 제거 → 3.10 용 휠이 설치된다 (소스 빌드 없음)
- `argparse` 삭제 → Python 2 시절 백포트라 표준 라이브러리를 덮어쓴다

### 필수 패치 — 이걸 안 하면 최적화가 안 돈다

`trajectory_planning_helpers` 가 scipy 1.3 시절 코드라 최신 scipy 에서 터진다.

```
ValueError: Input vector should be 1-D.
```

```bash
F=.venv/lib/python3.10/site-packages/trajectory_planning_helpers/spline_approximation.py
cp $F $F.bak
sed -i 's/euclidean(p, s)/euclidean(p, np.concatenate(s))/' $F
```

`splev` 가 x, y 를 리스트로 돌려줘서 `s` 가 2차원이 되는데, 예전 scipy 는
`euclidean()` 이 내부에서 squeeze 를 해줬고 지금은 거부한다. 형태만 맞추는 수정이다.

**이 파일은 `.venv` 안이라 커밋되지 않는다.** 환경을 새로 만들 때마다 다시 해야 한다.

### 환경 확인

```bash
python3 -c "import numpy, casadi, trajectory_planning_helpers as tph; print(numpy.__file__)"
```

경로에 `.venv` 가 나와야 한다. `/opt/ros/humble/...` 가 나오면 ROS 가
`PYTHONPATH` 를 통해 자기 numpy 를 끼워넣은 것이니 `unset PYTHONPATH` 후 다시 쓴다.
새 터미널마다 `source .venv/bin/activate` + `unset PYTHONPATH` 가 한 세트다.

---

## 2. 맵 → 중심선 (`map_converter.ipynb`)

```bash
cp ../../1_slam_mapping/maps/<맵이름>.pgm  maps/
cp ../../1_slam_mapping/maps/<맵이름>.yaml maps/
```

VS Code 로 `map_converter.ipynb` 를 열고 커널을 `.venv` 로 고른다.
노트북이 `maps/<이름>.pgm` 을 상대경로로 찾으므로 작업 디렉터리가
`Raceline-Optimization/` 이어야 한다.

**맵마다 바꿀 값**

| 셀 | 변수 | 설명 |
|---|---|---|
| 3 | `MAP_NAME` | 맵 파일 이름 (확장자 제외) |
| 3 | `TRACK_WIDTH_MARGIN` | 추가 안전 마진 [m], 기본 0.0 |
| 7 | `THRESHOLD` | 중심선 추출 임계값, 기본 0.17 |
| 11 | `LEFT_START_Y` | DFS 시작 높이. 업스트림 기본값 `map_height // 2 - 120` 은 큰 맵 기준이라 작은 맵에서 **음수**가 된다. `map_height // 2` 로 바꿔 둠 |

**가지치기 셀 (9번)** — 업스트림에는 없고 이 저장소에서 추가한 셀이다.

```python
# 가지(spur) 제거 — 끝점이 하나도 없을 때까지 반복
from scipy.ndimage import convolve

K = np.ones((3, 3))
while True:
    nb = convolve(centerline.astype(int), K, mode='constant') - centerline
    ends = (nb == 1) & centerline          # 이웃이 1개뿐인 픽셀 = 가지 끝
    if not ends.any():
        break
    centerline = centerline & ~ends

plt.figure(figsize=(10, 10))
plt.imshow(centerline, origin='lower', cmap='gray')
```

트랙에 넓은 방(코너 광장)이 있으면 스켈레톤이 모서리로 가지를 뻗는다. DFS 가 그
가지까지 훑어서 경로에 왕복 구간이 생기므로 반드시 제거해야 한다. 닫힌 고리에는
끝점이 없어서, 끝점을 계속 깎으면 가지만 사라지고 고리는 남는다. `THRESHOLD` 를
올려서는 해결되지 않는다 (0.17~0.50 전부 끝점이 남는다).

**중간 확인**

| 셀 | 볼 것 |
|---|---|
| 5 | 트랙만 흰색, 나머지 검정 (회색 205 는 임계값 210 에 걸려 자동으로 벽 처리된다) |
| 9 | 가지치기 후 끊김 없는 고리 하나 |
| 12 | `Starting position for left edge:` 좌표가 트랙 위인지 |
| 14 | 10/25/50/전체 누적 그림이 한 방향으로 순서대로 그려지는지 |

14번이 튀면 13번 셀의 `DIRECTIONS` 순서를 바꾼다.

**19번 셀은 두 번 실행하면 안 된다.** `transformed_data = data` 가 복사가 아니라
같은 배열을 가리키고 `*=` 가 제자리 연산이라, 재실행하면 resolution 이 두 번
곱해진다. 에러도 안 난다. 다시 돌릴 때는 **17번 셀부터** 실행한다.

결과: `inputs/tracks/<MAP_NAME>.csv`

---

## 3. 중심선 → 레이싱라인

`params/f110.ini` 는 이미 1/10 스케일 값이라 손댈 것이 없다.
(차폭 0.296 m, 차장 0.568 m, 질량 3.74 kg, `curvlim` 3.0 rad/m, `v_max` 15 m/s,
mincurv `width_opt` 0.4 m)

`main_globaltraj_f110.py` 81 행:

```python
opt_type = 'mincurv'
```

| 값 | 상태 |
|---|---|
| `shortest_path` | 동작. 최단 경로 |
| `mincurv` | **동작. 현재 사용** |
| `mincurv_iqp` | **깨짐** — 아래 참고 |
| `mintime` | 무겁고 타이어 파라미터에 민감. 업스트림 기본값 |

```bash
python3 main_globaltraj_f110.py --map_name <MAP_NAME>
```

결과: `outputs/<MAP_NAME>/traj_race_cl-<타임스탬프>.csv` (`x_m, y_m, vx_mps`, 헤더 없음)

파일명에 공백과 콜론이 들어가므로(`TIME = str(datetime.now())`), 쓸 것만 골라
고정 이름으로 복사한다.

```bash
cp "outputs/<MAP_NAME>/traj_race_cl-<타임스탬프>.csv" ../<맵이름>_raceline.csv
```

---

## 알려진 문제

**`mincurv_iqp` 사용 불가.** 설치되는 `trajectory_planning_helpers` 0.79 의
`iqp_handler()` 는 `spline_len`, `psi`, `kappa`, `dkappa` 를 더 요구하는데 repo 의
호출부는 그 인자들이 추가되기 전 버전 기준이고, `prep_track()` 도 그 값들을
반환하지 않는다. 쓰려면 호출 앞에 직접 계산해서 넘겨야 한다.

```python
spline_lengths_interp = tph.calc_spline_lengths.calc_spline_lengths(
    coeffs_x=coeffs_x_interp, coeffs_y=coeffs_y_interp)

psi_r, kappa_r, dkappa_r = tph.calc_head_curv_an.calc_head_curv_an(
    coeffs_x=coeffs_x_interp, coeffs_y=coeffs_y_interp,
    ind_spls=np.arange(reftrack_interp.shape[0]),
    t_spls=np.zeros(reftrack_interp.shape[0]),
    calc_curv=True, calc_dcurv=True)
```

`mincurv` 는 QP 를 한 번 풀고 IQP 는 선형화 오차를 줄이며 3회 이상 반복한다.
지금 트랙 규모에서는 1회로도 충분한 결과가 나온다.

**결과 그림의 초록 삼각형.** `helper_funcs_glob/src/result_plots.py` 69 행의
시작점 방향 화살표인데 `head_width=2.0` 이 미터 단위로 하드코딩돼 있다.
실제 서킷(수 km) 기준이라 20 m 트랙에서는 거대하게 보인다. 계산과 무관하다.

**결과 그림의 노란 지그재그.** 원본 CSV 의 트랙 경계다. 중심선이 픽셀 계단이라
법선 방향이 점마다 튄다. 최적화는 스플라인으로 매끄럽게 만든 경계(검정 실선)를
쓰므로 결과에는 거의 영향이 없다. 신경 쓰이면 노트북
20번 셀의
`transformed_data = transformed_data[::4]` 주석을 풀어 점을 1/4로 줄인다.

**`np.float` 관련 AttributeError.** 나오면 `pip install "numpy<2"` 로 내린다.
(2026-08-18 기준 numpy 2.2.6 에서 문제 없음)

---

## 현재 결과 — 0809_test_6

| 단계 | 산출물 | 값 |
|---|---|---|
| 맵 | `0809_test_6_map.pgm` | 357×143 px, 0.05 m/px → 17.85 × 7.15 m |
| 중심선 | `inputs/tracks/0809_test_6_map.csv` | 458점, 길이 24.55 m, 트랙폭 1.10~2.08 m |
| 레이싱라인 | `0809_test_6_raceline.csv` | 104점, 길이 **20.50 m** |

속도 2.97 ~ 7.49 m/s, 예상 랩타임 **4.26 s**.

원본 맵 픽셀에 직접 대조한 검증 결과:

- 트랙 밖으로 나간 점 **0개**
- 벽까지 최소 여유 **0.200 m** — `width_opt` 0.4 의 정확히 절반이라
  제약을 만족하면서 허용된 폭을 끝까지 쓴 것이다
- 시작–끝 거리 0.000 m (닫힌 고리)

중심선 대비 **4.05 m 짧아진 것**이 코너를 깎아낸 양이다.
