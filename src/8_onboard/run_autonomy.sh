#!/usr/bin/env bash

# Run one F1TENTH autonomy session and guarantee process-group cleanup.
# Controller/planner parameters remain launch arguments; only workspace paths
# differ between the simulator and the onboard container.

readonly session_file=/tmp/f1tenth_autonomy_session.pgid
readonly lock_file=/tmp/f1tenth_autonomy_session.lock
session_pgid=''
signal_received=0

group_has_ros_processes() {
  local group_id=$1
  ps -eo pgid=,args= | awk -v target="$group_id" '
    $1 == target && ($0 ~ /ros2 launch f1tenth_bringup autonomy.launch.py/ ||
                     $0 ~ /\/sim_ws\/install\// ||
                     $0 ~ /\/home\/misys\/shared_dir\/autonomy_ws\/install\// ||
                     $0 ~ /\/opt\/ros\/humble\/lib\/nav2_/ ||
                     $0 ~ /\/opt\/ros\/humble\/lib\/rviz2\//) {
      found = 1
    }
    END { exit(found ? 0 : 1) }
  '
}

terminate_group() {
  local group_id=${1:-}
  local attempt
  [[ $group_id =~ ^[1-9][0-9]*$ ]] || return 0
  group_has_ros_processes "$group_id" || return 0

  kill -INT -- "-$group_id" 2>/dev/null || true
  for attempt in {1..50}; do
    group_has_ros_processes "$group_id" || return 0
    sleep 0.1
  done

  kill -TERM -- "-$group_id" 2>/dev/null || true
  for attempt in {1..20}; do
    group_has_ros_processes "$group_id" || return 0
    sleep 0.1
  done

  kill -KILL -- "-$group_id" 2>/dev/null || true
}

cleanup() {
  terminate_group "$session_pgid"
  if [[ -f $session_file ]] && [[ $(<"$session_file") == "$session_pgid" ]]; then
    rm -f "$session_file"
  fi
}

handle_signal() {
  signal_received=1
  cleanup
}

exec 9>"$lock_file"
if ! flock -n 9; then
  echo 'Another managed autonomy session is already running.' >&2
  exit 1
fi

# Recover a process group left by a killed terminal or interrupted launch.
if [[ -f $session_file ]]; then
  previous_pgid=$(<"$session_file")
  terminate_group "$previous_pgid"
  rm -f "$session_file"
fi

# Also clean an older direct ros2 launch once, before the managed session.
while read -r previous_pgid; do
  [[ -n $previous_pgid ]] && terminate_group "$previous_pgid"
done < <(ps -eo pgid=,args= | awk '
  /[r]os2 launch f1tenth_bringup autonomy.launch.py/ {print $1}
' | sort -nu)

# If a terminal or ros2 launch crashed, its children can be re-parented to PID
# 1 after the launch parent disappears.  Recover only those orphaned autonomy
# groups; unrelated ROS processes and normally parented bringup nodes are left
# untouched.
while read -r previous_pgid; do
  [[ -n $previous_pgid ]] && terminate_group "$previous_pgid"
done < <(ps -eo ppid=,pgid=,args= | awk '
  $1 == 1 && ($0 ~ /\/sim_ws\/install\/f1tenth_gym_ros\/lib\/f1tenth_gym_ros\/gym_bridge/ ||
              $0 ~ /\/sim_ws\/install\/planning\/lib\/planning\/waypoint_planner_node/ ||
              $0 ~ /\/sim_ws\/install\/control\/lib\/control\// ||
              $0 ~ /\/home\/misys\/shared_dir\/autonomy_ws\/install\/planning\/lib\/planning\/waypoint_planner_node/ ||
              $0 ~ /\/home\/misys\/shared_dir\/autonomy_ws\/install\/control\/lib\/control\// ||
              $0 ~ /\/opt\/ros\/humble\/lib\/nav2_(amcl|map_server|lifecycle_manager)\// ||
              $0 ~ /\/opt\/ros\/humble\/lib\/robot_state_publisher\/robot_state_publisher.*ego_robot_state_publisher/) {
    print $2
  }
' | sort -nu)

# ROS setup scripts are not nounset-safe. Environment detection is limited to
# deployment paths and never changes controller or track parameters.
source /opt/ros/humble/setup.bash
# minjae changes
# 예전에는 /sim_ws 를 먼저 확인했다.  if/elif 라 한쪽만 실행되므로, 차량
# 컨테이너에 /sim_ws 가 남아 있으면 온보드 워크스페이스와 f1tenth_ws
# (VESC/조이스틱/mux)를 통째로 건너뛰고도 launch 는 멀쩡히 떴다.  실차에서
# 빌드한 코드가 아닌 것이 도는데 증상이 없어 알아채기 어렵다.
# 시뮬레이터 머신에는 shared_dir 이 없으므로 여전히 /sim_ws 가 선택된다.
autonomy_workspace=''
if [[ -f /home/misys/shared_dir/autonomy_ws/install/setup.bash ]]; then
  if [[ -f /home/misys/f1tenth_ws/install/setup.bash ]]; then
    source /home/misys/f1tenth_ws/install/setup.bash
  fi
  source /home/misys/shared_dir/autonomy_ws/install/setup.bash
  autonomy_workspace=/home/misys/shared_dir/autonomy_ws
elif [[ -f /sim_ws/install/setup.bash ]]; then
  source /sim_ws/install/setup.bash
  autonomy_workspace=/sim_ws
else
  echo 'Autonomy install/setup.bash was not found.' >&2
  exit 1
fi
# 어느 워크스페이스가 잡혔는지 매 실행마다 남긴다.  ros2 pkg prefix 로
# 따로 확인하지 않아도 잘못 잡힌 것을 바로 볼 수 있다.
echo "autonomy workspace: $autonomy_workspace" >&2
# minjae changes
set -u

trap handle_signal INT TERM HUP TSTP
trap cleanup EXIT

# The wrapper alone owns the session lock.  Do not let ros2 launch or any of
# its descendants inherit fd 9; otherwise a killed terminal leaves the flock
# held by orphaned ROS children and prevents the next run from recovering it.
setsid ros2 launch f1tenth_bringup autonomy.launch.py "$@" 9>&- &
session_pgid=$!
printf '%s\n' "$session_pgid" >"$session_file"

wait "$session_pgid"
status=$?
cleanup

if (( signal_received )); then
  exit 130
fi
exit "$status"
