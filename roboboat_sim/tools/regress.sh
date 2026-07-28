#!/usr/bin/env bash
# Re-verify that the tasks which used to work still work.
#
#   tools/regress.sh              # every task that has a frozen contract
#   tools/regress.sh channel      # just one
#
# Run this after every change to the course, the behaviour trees or the Nav2
# config. A task is regressed when its contract in tuning/regression_<task>.json
# stops holding -- not when a metric moves, which rebuilding course geometry
# does by construction. See tools/check_regression.py for the two tiers.
set -o pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ $# -gt 0 ]]; then
  TASKS=("$@")
else
  TASKS=()
  for f in "$WS"/tuning/regression_*.json; do
    [[ -e "$f" ]] || continue
    name="${f##*/regression_}"
    TASKS+=("${name%.json}")
  done
fi

if [[ ${#TASKS[@]} -eq 0 ]]; then
  echo "no frozen contracts in $WS/tuning/; nothing to regress"
  exit 0
fi

echo "regression suite: ${TASKS[*]}"
FAILED=()
for task in "${TASKS[@]}"; do
  contract="$WS/tuning/regression_${task}.json"
  if [[ ! -f "$contract" ]]; then
    echo "== $task: no contract at $contract"; FAILED+=("$task"); continue
  fi
  echo
  echo "== $task ============================================================"
  bash "$WS/tools/task_trial.sh" "$task" "regress_${task}"
  # Score the card against the contract even if the run itself failed: the
  # contract check is the thing that prints WHY, and a run that fails its own
  # pass criterion should still produce a readable diagnosis.
  python3 "$WS/tools/check_regression.py" \
      --scorecard "$WS/tuning/regress_${task}_card.json" \
      --contract "$contract" || FAILED+=("$task")
done

echo
if [[ ${#FAILED[@]} -gt 0 ]]; then
  echo "REGRESSED: ${FAILED[*]}"
  exit 1
fi
echo "all ${#TASKS[@]} task contract(s) hold"
