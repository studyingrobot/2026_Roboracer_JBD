#!/usr/bin/env bash
# 맵 -> 중심선 -> 최적 raceline -> 검증 을 한 번에 돌리는 래퍼.
#
# 최적화기는 Python 3.8 고정 의존성이라 Docker 안에서 돈다 (tools/Dockerfile).
# 중심선 생성/검증은 f1tenth 시뮬 레포의 스크립트를 쓰므로 그쪽 경로가 필요하다.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
IMAGE="raceline-opt:py38"

# f1tenth 시뮬 레포 위치 (generate_centerline.py / validate_raceline.py 가 여기 있다).
SIM_DIR="${F1TENTH_SIM_DIR:-$HERE/../7_sim}"

MAP_YAML=""
NAME=""
OPT_TYPE="mincurv"
HALF_WIDTH="0.65"
SPACING="0.10"
START="0,0,0"
SKIP_CENTERLINE=0

usage() {
    cat <<'EOF'
사용법:
  ./run_optimization.sh --map <맵.yaml> --name <트랙이름> [옵션]

필수:
  --map PATH          맵 yaml 경로 (.pgm/.png 은 yaml 이 가리키는 것을 씀)
  --name NAME         트랙 이름. 결과 파일 이름의 접두사가 된다

옵션:
  --opt-type TYPE     mincurv (기본) | shortest_path | mintime
                      ※ mincurv_iqp 는 upstream 버그로 동작하지 않는다
  --start X,Y,YAW     중심선 시작점과 진행 방향 (기본 0,0,0)
                      주행 방향이 반대로 나오면 yaw 를 3.14159 로
  --half-width M      최적화기에 넘길 좌우 반폭 상한 (기본 0.65)
  --spacing M         중심선 샘플 간격 (기본 0.10)
  --skip-centerline   1단계를 건너뛰고 outputs/<name>_optimizer_input.csv 를 그대로 사용
  -h, --help          이 도움말

결과물은 모두 outputs/ 아래에 남는다:
  <name>_centerline.csv        중심선 (x,y,yaw,curvature,speed)
  <name>_optimizer_input.csv   최적화기 입력 (x_m,y_m,w_tr_right_m,w_tr_left_m)
  <name>_centerline.png        중심선 미리보기
  <name>_raceline.csv          ★ 최적 전역경로 (x,y,vx / 헤더 없음)
  <name>_raceline.png          검증 미리보기
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        --map)             MAP_YAML="$2"; shift 2 ;;
        --name)            NAME="$2"; shift 2 ;;
        --opt-type)        OPT_TYPE="$2"; shift 2 ;;
        --start)           START="$2"; shift 2 ;;
        --half-width)      HALF_WIDTH="$2"; shift 2 ;;
        --spacing)         SPACING="$2"; shift 2 ;;
        --skip-centerline) SKIP_CENTERLINE=1; shift ;;
        -h|--help)         usage; exit 0 ;;
        *) echo "알 수 없는 인자: $1" >&2; usage; exit 1 ;;
    esac
done

[[ -n "$NAME" ]] || { echo "ERROR: --name 이 필요하다" >&2; usage; exit 1; }
if [[ $SKIP_CENTERLINE -eq 0 && -z "$MAP_YAML" ]]; then
    echo "ERROR: --map 이 필요하다 (또는 --skip-centerline)" >&2
    exit 1
fi

if [[ "$OPT_TYPE" == "mincurv_iqp" ]]; then
    echo "ERROR: mincurv_iqp 는 upstream 버그로 동작하지 않는다 (README 참고). mincurv 를 쓸 것." >&2
    exit 1
fi

OUT="$HERE/outputs"
mkdir -p "$OUT"

CENTERLINE="$OUT/${NAME}_centerline.csv"
OPT_INPUT="$OUT/${NAME}_optimizer_input.csv"
RACELINE="$OUT/${NAME}_raceline.csv"

IFS=',' read -r START_X START_Y START_YAW <<< "$START"

# ---------------------------------------------------------------- 1. 중심선
if [[ $SKIP_CENTERLINE -eq 0 ]]; then
    GEN="$SIM_DIR/algorithms/planning/scripts/generate_centerline.py"
    if [[ ! -f "$GEN" ]]; then
        cat >&2 <<EOF
ERROR: generate_centerline.py 를 찾을 수 없다.
       찾은 경로: $GEN

       f1tenth 시뮬 레포가 아직 없다. 둘 중 하나로 해결할 것:
         1) 시뮬 레포를 src/7_sim 에 넣는다
         2) F1TENTH_SIM_DIR=<레포경로> 환경변수로 알려준다
         3) 최적화기 입력 CSV 를 직접 만들어 두고 --skip-centerline 을 쓴다
            (형식: # x_m,y_m,w_tr_right_m,w_tr_left_m)
EOF
        exit 1
    fi

    echo "== 1/3 중심선 생성 =="
    python3 "$GEN" \
        --map-yaml "$MAP_YAML" \
        --output "$CENTERLINE" \
        --optimizer-output "$OPT_INPUT" \
        --preview "$OUT/${NAME}_centerline.png" \
        --spacing "$SPACING" \
        --max-half-width "$HALF_WIDTH" \
        --start-x "$START_X" --start-y "$START_Y" --start-yaw "$START_YAW"
else
    echo "== 1/3 중심선 생성 건너뜀 =="
    [[ -f "$OPT_INPUT" ]] || { echo "ERROR: $OPT_INPUT 가 없다" >&2; exit 1; }
fi

# ---------------------------------------------------------------- 2. 최적화
echo "== 2/3 전역경로 최적화 (opt_type=$OPT_TYPE) =="

docker image inspect "$IMAGE" >/dev/null 2>&1 || {
    echo "-- 이미지가 없다. 빌드한다 (최초 1회, 약 2분) --"
    docker build -f "$HERE/tools/Dockerfile" -t "$IMAGE" "$HERE"
}

# 최적화기는 inputs/tracks/<name>.csv 를 기대한다.
cp "$OPT_INPUT" "$HERE/Raceline-Optimization/inputs/tracks/${NAME}.csv"

docker run --rm \
    --user "$(id -u):$(id -g)" \
    -e MPLCONFIGDIR=/tmp/mpl \
    -v "$HERE:/work" -w /work \
    "$IMAGE" \
    python3 tools/run_globaltraj.py \
        --opt-type "$OPT_TYPE" \
        --map_name "$NAME" \
        --export_path "/work/outputs/${NAME}_raceline.csv"

# ---------------------------------------------------------------- 3. 검증
echo "== 3/3 검증 =="
VAL="$SIM_DIR/algorithms/planning/scripts/validate_raceline.py"
if [[ -f "$VAL" && -n "$MAP_YAML" ]]; then
    python3 "$VAL" \
        --map-yaml "$MAP_YAML" \
        --raceline "$RACELINE" \
        --preview "$OUT/${NAME}_raceline.png"
else
    echo "   validate_raceline.py 없음 -> 건너뜀 (시뮬 레포 추가 후 수동 실행 권장)"
fi

echo
echo "완료: $RACELINE"
echo "  형식: x,y,vx (헤더 없음). waypoint_planner_node 가 그대로 읽는다."
