#!/usr/bin/env bash
# local_ekf.yaml 을 수정하지 않고 test_bags 에 적용해서 돌린다.
#   ./run_ekf_bag.sh 0808_test_2 [출력bag경로]
BAG_NAME=${1:?사용법: ./run_ekf_bag.sh <bag이름> [출력bag]}
HERE=$(cd "$(dirname "$0")" && pwd)
ROOT=$(cd "$HERE/../.." && pwd)
BAGS=$ROOT/src/1_slam_mapping/maps/test_bags
BAG=$BAGS/$BAG_NAME
OUT=${2:-$BAGS/${BAG_NAME}_ekf}
YAML=$HERE/local_ekf.yaml
LOG=${TMPDIR:-/tmp}/ekf_bag_$$

source /opt/ros/humble/setup.bash
rm -rf "$OUT"
mkdir -p "$LOG"

PIDS=()
cleanup() { for p in "${PIDS[@]:-}"; do kill "$p" 2>/dev/null; done; }
trap cleanup EXIT

# 1) VESC raw IMU -> EKF 가 기대하는 단위/frame_id/covariance 로 변환
python3 "$HERE/imu_relay.py" --ros-args \
  -p use_sim_time:=true \
  -p in_topic:=/sensors/imu/raw \
  -p out_topic:=/camera/camera/imu > "$LOG/relay.log" 2>&1 &
PIDS+=($!)

# 2) EKF. yaml 은 그대로 두고 토픽만 remap 한다.
#    bag 에 이미 vesc 의 odom->base_link TF 가 있으므로 EKF 의 /tf 는 /tf_ekf 로 돌린다.
ros2 run robot_localization ekf_node --ros-args \
  --params-file "$YAML" \
  -r /wheel/odom:=/odom \
  -r /tf:=/tf_ekf \
  -p use_sim_time:=true > "$LOG/ekf.log" 2>&1 &
PIDS+=($!)

sleep 3
ros2 bag record -o "$OUT" /odometry/filtered --use-sim-time > "$LOG/rec.log" 2>&1 &
REC=$!
PIDS+=($REC)
sleep 2

# 3) bag 의 /tf 는 재생하지 않는다 (EKF 와 충돌).
ros2 bag play "$BAG" --clock 100 --topics /odom /sensors/imu/raw /tf_static

sleep 2
kill -INT $REC 2>/dev/null
for _ in $(seq 20); do [ -d "$OUT" ] && ls "$OUT"/*.db3 >/dev/null 2>&1 && break; sleep 0.5; done
sleep 2
echo "결과 bag: $OUT"
echo "로그:     $LOG"
grep -h WARN "$LOG"/ekf.log | sort -u | head -5
grep -h "bias" "$LOG"/relay.log | head -3
