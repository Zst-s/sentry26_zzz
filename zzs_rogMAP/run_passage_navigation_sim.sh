#!/usr/bin/env bash
# Standalone simulation overlay; no serial, real driver or production install changes.
set -eo pipefail
sim_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
sim_build="$sim_workspace/.passage_nav_sim"
unset AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH
source /opt/ros/humble/setup.bash
source "$sim_workspace/install/local_setup.bash"
export ROS_DOMAIN_ID=229 ROS_LOCALHOST_ONLY=1
export LD_PRELOAD="/lib/x86_64-linux-gnu/libusb-1.0.so.0${LD_PRELOAD:+:$LD_PRELOAD}"
mkdir -p "$sim_build"
exec 9>"$sim_build/run.lock"
flock -n 9 || { echo "仿真已运行，请先在原终端 Ctrl+C" >&2; exit 1; }
colcon --log-base "$sim_build/log" build \
  --base-paths "$sim_workspace/src/rm_perception/sentry_passage_sim" \
  --build-base "$sim_build/build" --install-base "$sim_build/install" \
  --executor sequential --cmake-args -DBUILD_TESTING=ON
source "$sim_build/install/local_setup.bash"
ctest --test-dir "$sim_build/build/sentry_passage_sim" --output-on-failure
if [[ "${1:-}" == "--build-only" ]]; then exit 0; fi
if [[ "${1:-}" == "--check" ]]; then
  shift
  sim_run="$sim_build/runs/$(date +%Y%m%d_%H%M%S)_$$"
  mkdir -p "$sim_run"
  python3 - <<'CHECK_PORT'
import socket
with socket.socket() as sock:
    if sock.connect_ex(('127.0.0.1', 11359)) == 0:
        raise SystemExit('仿真端口11359已占用，请先关闭旧实例')
CHECK_PORT
  setsid python3 -c 'import os,signal,sys; signal.signal(signal.SIGINT,signal.SIG_DFL); os.execvp(sys.argv[1],sys.argv[1:])' \
    ros2 launch sentry_passage_sim navigation_lab.launch.py gui:=false rviz:=false \
    artifacts_dir:="$sim_run" "$@" >"$sim_run/launch.log" 2>&1 &
  sim_pid=$!
  cleanup_sim() {
    # This dedicated process group contains only this test's launch and children.
    kill -INT -- "-$sim_pid" 2>/dev/null || true
    for sim_wait in {1..100}; do
      kill -0 -- "-$sim_pid" 2>/dev/null || break
      sleep .1
    done
    kill -TERM -- "-$sim_pid" 2>/dev/null || true
    sleep .2
    kill -KILL -- "-$sim_pid" 2>/dev/null || true
    wait "$sim_pid" 2>/dev/null || true
  }
  trap cleanup_sim EXIT
  trap 'exit 130' INT
  trap 'exit 143' TERM
  echo "隔离仿真自动过洞检查，结果目录：$sim_run"
  set +e
  ros2 run sentry_passage_sim check_navigation.py --output "$sim_run/validation.json" \
    2>&1 | tee "$sim_run/check.log"
  sim_result=${PIPESTATUS[0]}
  set -e
  exit "$sim_result"
fi
sim_run="$sim_build/runs/$(date +%Y%m%d_%H%M%S)_$$"
mkdir -p "$sim_run"
echo "隔离仿真资产目录：$sim_run"
exec ros2 launch sentry_passage_sim navigation_lab.launch.py artifacts_dir:="$sim_run" "$@"
