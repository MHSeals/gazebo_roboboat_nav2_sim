#!/usr/bin/env python3
"""Compare benchmark scorecards between two configs.

    python3 tools/compare_runs.py baseline candidate

Aggregates every tuning/<label>_r*.json for each label and prints a side by
side table with the spread across repeats.

The spread is the point. Two identical configs on this course have been
measured 0.31 m apart in minimum clearance, so a single run cannot support
"this change helped". A difference smaller than the baseline's own range is
noise, and the tool says so rather than leaving it to optimism.
"""

from __future__ import annotations

import argparse
import json
import statistics
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TUNING = ROOT / 'tuning'

# Smallest difference worth acting on, per metric. Without these the min-max
# rule becomes absurdly sensitive once the harness gets precise: a baseline
# spread of 0.1 s made a 0.7 s change out of 183 s read as "better", and 0.6 Hz
# of odom jitter read as a config improvement. A verdict needs to clear both
# the observed noise AND a floor of practical relevance.
#
# Values are set from what matters to the vehicle, not from the statistics:
# goal error only matters against the 0.50 m goal tolerance; clearance only
# matters in tenths of a metre for a 1.1 m beam; time only matters in seconds.
MIN_EFFECT = {
    'goals_reached': 0.5,
    'encounter_min_mean': 0.10,
    'min_clearance_m': 0.05,
    'mean_error_m': 0.12,
    'reached_seconds': 5.0,
    'path_length_m': 2.0,
    'mean_efficiency': 0.02,
    'mean_speed_ms': 0.03,
    'cmd_jerk': 0.08,
    'cmd_period_p95_s': 0.005,
    'odom_rate_hz': 5.0,
}

# (key, label, higher_is_better, format)
METRICS = [
    ('goals_reached', 'goals reached', True, '{:.1f}'),
    # Primary safety metric: mean closest approach across every obstacle the
    # boat came near. Run-level min_clearance is a single-sample extremum and
    # is kept only for reference.
    ('encounter_min_mean', 'mean approach (m)', True, '{:+.3f}'),
    ('min_clearance_m', 'min clearance (m)', True, '{:+.3f}'),
    ('mean_error_m', 'goal error (m)', False, '{:.3f}'),
    ('reached_seconds', 'time to goals (s)', False, '{:.1f}'),
    ('path_length_m', 'path length (m)', False, '{:.1f}'),
    ('mean_efficiency', 'path directness', True, '{:.3f}'),
    ('mean_speed_ms', 'mean speed (m/s)', True, '{:.3f}'),
    # First difference of command over time, i.e. mean absolute command
    # acceleration -- not jerk, despite the field name kept for continuity.
    ('cmd_jerk', 'command accel', False, '{:.2f}'),
    ('cmd_period_p95_s', 'p95 cmd period (s)', False, '{:.4f}'),
    ('odom_rate_hz', 'odom rate (Hz)', True, '{:.1f}'),
]


def load(label: str) -> list[dict]:
    """Load scorecards, refusing runs where the stack never worked.

    A failed bringup still writes a card, and it reports a huge minimum
    clearance because the boat never moved. Aggregating one of those inflates
    the observed range so far that every future verdict becomes 'noise' --
    the harness would silently stop being able to detect anything.
    """
    cards = sorted(TUNING.glob(f'{label}_r*.json'))
    if not cards:
        sys.exit(f'no scorecards for {label!r} in {TUNING}')

    good, bad = [], []
    for path in cards:
        card = json.loads(path.read_text())
        if card.get('goals_reached', 0) == 0 or not card.get('path_length_m'):
            bad.append(path.name)
            continue
        good.append(card)
    for name in bad:
        print(f'  skipping {name}: stack never navigated (invalid run)')
    if not good:
        sys.exit(f'every scorecard for {label!r} is invalid')
    return good


def stat(cards: list[dict], key: str):
    values = [c.get(key) for c in cards if c.get(key) is not None]
    if not values:
        return None, None, None
    lo, hi = min(values), max(values)
    return statistics.mean(values), lo, hi


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline')
    parser.add_argument('candidate')
    args = parser.parse_args()

    base, cand = load(args.baseline), load(args.candidate)
    print(f'{args.baseline}: {len(base)} run(s)   '
          f'{args.candidate}: {len(cand)} run(s)\n')

    head = f'{"metric":<22}{"baseline":>20}{"candidate":>20}   verdict'
    print(head)
    print('-' * len(head))

    for key, label, higher_better, fmt in METRICS:
        b_mean, b_lo, b_hi = stat(base, key)
        c_mean, c_lo, c_hi = stat(cand, key)
        if b_mean is None or c_mean is None:
            continue

        b_txt = fmt.format(b_mean) + (f' [{b_lo:.2f}–{b_hi:.2f}]'
                                      if len(base) > 1 else '')
        c_txt = fmt.format(c_mean) + (f' [{c_lo:.2f}–{c_hi:.2f}]'
                                      if len(cand) > 1 else '')

        delta = c_mean - b_mean
        improved = delta > 0 if higher_better else delta < 0
        # The floor is the widest of: the baseline's own spread, the
        # candidate's own spread, and the minimum effect worth acting on.
        # Using only the baseline's spread lets a noisy candidate look
        # decisive -- experiment 1 produced a 0.7 s "improvement" whose own
        # runs differed by 4.7 s.
        spread = max((b_hi - b_lo) if len(base) > 1 else 0.0,
                     (c_hi - c_lo) if len(cand) > 1 else 0.0)
        floor = max(spread, MIN_EFFECT.get(key, 0.0))
        if abs(delta) <= floor or abs(delta) < 1e-9:
            verdict = 'noise'
        else:
            verdict = 'better' if improved else 'WORSE'
        print(f'{label:<22}{b_txt:>20}{c_txt:>20}   {verdict}')

    print()
    if len(base) < 2:
        print('NOTE: baseline has a single run, so nothing can be called '
              'noise. Run the baseline at least twice before trusting any '
              'verdict above.')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
