#!/usr/bin/env python3
"""Verify every MPPI critic actually loaded the weight the config asked for.

This exists because of a real, expensive failure. Nav2's own MPPI README
documents TwirlingCritic with `twirling_cost_power` / `twirling_cost_weight`,
but the code reads `cost_power` / `cost_weight`. Undeclared keys in a ROS
parameter file are silently ignored, so the critic ran at its 10.0 default
while the config said 5.0, and an entire A/B experiment measured a change
that never applied. Both arms were identical and the difference was reported
as a result.

Nothing about that is detectable from the config file alone. The only
authority is what the controller logs at startup:

    TwirlingCritic instantiated with 1 power and 10.000000 weight.

So compare the log against the YAML and fail loudly on a mismatch.

    python3 tools/check_critics.py --log /tmp/bench_x_nav.log

Note: CostCritic and ObstaclesCritic normalise their weight by 254 before
logging it, so their logged value is expected to differ from the YAML.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PARAMS = ROOT / 'src' / 'roboboat_bringup' / 'config' / 'nav2_mppi.yaml'

# Critics do not log in one format, and two of them log under a class name
# that is not the name you configure them by. Tried in order; first match wins.
LOG_PATTERNS = [
    # CostCritic: "... 1 power and 300.000000 / 0.015000 weights."
    # The second number is the one derived from cost_weight.
    (re.compile(r'(\w+Critic) instantiated with (\d+) power and '
                r'[\d.]+ / ([\d.]+) weights'), 3),
    # GoalAngleCritic: "... 1 power, 3.000000 weight, 0.800000 angular ..."
    (re.compile(r'(\w+Critic) instantiated with (\d+) power, ([\d.]+) weight'), 3),
    # The common form: "... 1 power and 4.000000 weight."
    (re.compile(r'(\w+Critic) instantiated with (\d+) power and ([\d.]+) weight'), 3),
]

# Critics that scale weight internally before logging it.
NORMALISED = {'CostCritic': 254.0, 'ObstaclesCritic': 254.0}
# Logged class name -> the name used in the config.
LOG_ALIASES = {
    'InflationCostCritic': 'CostCritic',
    'ReferenceTrajectoryCritic': 'PathAlignCritic',
}


def configured_weights(params_file: Path) -> dict[str, float]:
    doc = yaml.safe_load(params_file.read_text())
    follow = doc['controller_server']['ros__parameters']['FollowPath']
    out = {}
    for name in follow.get('critics', []):
        block = follow.get(name)
        if isinstance(block, dict) and 'cost_weight' in block:
            out[name] = float(block['cost_weight'])
    return out


def logged_weights(log_file: Path) -> dict[str, float]:
    out = {}
    for line in log_file.read_text(errors='ignore').splitlines():
        for pattern, group in LOG_PATTERNS:
            match = pattern.search(line)
            if match:
                name = LOG_ALIASES.get(match.group(1), match.group(1))
                out[name] = float(match.group(group))
                break
    return out


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--log', type=Path, required=True)
    parser.add_argument('--params', type=Path, default=DEFAULT_PARAMS)
    args = parser.parse_args()

    if not args.log.exists():
        print(f'no log at {args.log}', file=sys.stderr)
        return 2

    want, got = configured_weights(args.params), logged_weights(args.log)
    if not got:
        print('no critic instantiation lines in the log; did the controller '
              'start?', file=sys.stderr)
        return 2

    failures, unverified = [], []
    for name, weight in sorted(want.items()):
        if name not in got:
            # Some critics simply never log a weight. Absence is not evidence
            # of a wrong key, so say so rather than failing the run.
            unverified.append(name)
            continue
        expected = weight / NORMALISED.get(name, 1.0)
        if abs(got[name] - expected) > 1e-3:
            failures.append(
                f'{name}: config says {weight} (expect {expected:.4f} logged) '
                f'but controller loaded {got[name]} '
                '-- key name almost certainly wrong')

    for line in failures:
        print(f'  CRITIC MISMATCH  {line}')
    if failures:
        print('  the experiment did not test what it claims to test')
        return 1
    checked = len(want) - len(unverified)
    note = f' ({len(unverified)} log no weight: {", ".join(unverified)})' \
        if unverified else ''
    print(f'  critics verified: {checked}/{len(want)} weights match{note}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
