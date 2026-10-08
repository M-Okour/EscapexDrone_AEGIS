#!/usr/bin/env bash
# End-to-end check: launch the whole stack in sim, auto-start the mission and
# wait until it reports DONE. Used by CI; also handy before opening a PR.
#
#   source install/setup.bash && ./scripts/smoke_test.sh
set -uo pipefail

TIMEOUT=${TIMEOUT:-90}
LOG=$(mktemp)
MSG=$(mktemp)

# Start the ros2 CLI daemon up front: if the first `ros2 topic echo` spawned it
# inside $(...), the daemon would keep the pipe open and the script would hang.
ros2 daemon start >/dev/null 2>&1 || true

ros2 launch exd_bringup exd.launch.py sim:=true auto_start:=true >"$LOG" 2>&1 &
LAUNCH_PID=$!
cleanup() {
  kill -INT $LAUNCH_PID 2>/dev/null
  for _ in $(seq 1 20); do kill -0 $LAUNCH_PID 2>/dev/null || break; sleep 0.5; done
  kill -KILL $LAUNCH_PID 2>/dev/null
  ros2 daemon stop >/dev/null 2>&1 || true
}
trap cleanup EXIT

echo "waiting up to ${TIMEOUT}s for mission to finish..."
start=$(date +%s)
state=""
while (( $(date +%s) - start < TIMEOUT )); do
  timeout 5 ros2 topic echo --once /exd/mission/status >"$MSG" 2>/dev/null </dev/null || true
  state=$(awk '/^state:/ {print $2}' "$MSG" | tr -d "'\"")
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
if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
  # Surface the important lines as an annotation on the PR / run page.
  summary=$(grep -E 'ERROR|Error|Traceback|rror:|died|status:' "$LOG" | tail -40 \
            | sed -e 's/%/%25/g' | awk '{printf "%s%%0A", $0}')
  echo "::error title=Smoke test failed (last state: ${state:-none})::${summary}"
fi
exit 1
