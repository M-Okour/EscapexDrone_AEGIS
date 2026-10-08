#!/usr/bin/env bash
# End-to-end check: launch the whole stack in sim, auto-start the mission and
# wait until it reports DONE. Used by CI; also handy before opening a PR.
#
#   source install/setup.bash && ./scripts/smoke_test.sh
set -uo pipefail

TIMEOUT=${TIMEOUT:-90}
LOG=$(mktemp)

ros2 launch exd_bringup exd.launch.py sim:=true auto_start:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
trap 'kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null' EXIT

echo "waiting up to ${TIMEOUT}s for mission to finish..."
start=$(date +%s)
state=""
while (( $(date +%s) - start < TIMEOUT )); do
  state=$(timeout 5 ros2 topic echo --once /exd/mission/status 2>/dev/null \
          | awk '/^state:/ {print $2}' | tr -d "'\"")
  echo "  [$(( $(date +%s) - start ))s] mission state: ${state:-<none>}"
  if [[ "$state" == "DONE" ]]; then
    echo "SMOKE TEST PASSED"
    exit 0
  fi
  if [[ "$state" == "ABORTED" ]]; then
    break
  fi
  if ! kill -0 $LAUNCH_PID 2>/dev/null; then
    echo "launch exited early"
    break
  fi
  sleep 2
done

echo "SMOKE TEST FAILED (last state: ${state:-<none>})"
echo "----- launch log -----"
cat "$LOG"
exit 1
