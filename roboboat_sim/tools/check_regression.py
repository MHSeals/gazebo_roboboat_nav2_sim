#!/usr/bin/env python3
"""Check a task scorecard against its frozen contract.

The point of this file is to make "don't break the channel task" mechanical
rather than remembered. Every change to the course, the behaviour trees or the
Nav2 config has to leave the contract satisfied, and the check is cheap enough
to run after each one.

Two tiers, deliberately:

  require   pass/fail. Breaking one of these fails the build.
  drift     reported, never fatal. Rebuilding the course geometry moves path
            length and clearance by construction -- treating a metric snapshot
            as a contract would mean the first legitimate course edit reds the
            build and the whole harness gets switched off. Drift is how the
            numbers stay visible without being load-bearing.

    python3 tools/check_regression.py --scorecard card.json \
        --contract tuning/regression_channel.json
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def _mean(values: list[float]) -> float | None:
    vals = [v for v in values if v is not None]
    return sum(vals) / len(vals) if vals else None


def check(card: dict, contract: dict) -> int:
    require = contract.get('require', {})
    reference = contract.get('reference', {})
    warn = contract.get('drift_warn', {})

    failures: list[str] = []
    warnings: list[str] = []

    kind = card.get('type', 'gate_transit')
    total = card.get('gates_total', 0)
    transited = card.get('gates_transited', 0)
    clearance = card.get('min_clearance_m')
    contact = bool(card.get('contact', True))
    offsets = [g.get('offset_from_centre_m') for g in card.get('gates', [])]

    # -------------------------------------------------------------- require
    if require.get('all_gates_transited'):
        if total <= 0:
            failures.append('scorecard reports no gates at all')
        elif transited != total:
            missed = [g['gate'] for g in card.get('gates', [])
                      if not g.get('transited')]
            failures.append(
                f'{transited}/{total} gates transited; missed {", ".join(missed)}')

    # A task type with its own pass rule -- circling a mark in the commanded
    # direction, say -- cannot be scored by counting gates. Trust the scorer's
    # own verdict and let the contract add safety terms on top of it.
    if require.get('task_passed') and not card.get('passed'):
        reasons = [k for k in ('entered', 'exited', 'circled', 'well_sampled')
                   if k in card and not card[k]]
        reasons += [k for k in ('backtracked', 'wrong_direction', 'contact')
                    if card.get(k)]
        failures.append(
            f'the task did not pass'
            + (f' ({", ".join(reasons)})' if reasons else ''))

    if require.get('contact') is False and contact:
        failures.append(f'contact occurred (min clearance {clearance})')

    floor = require.get('min_clearance_m_at_least')
    if floor is not None:
        if clearance is None:
            failures.append('scorecard has no min_clearance_m')
        elif clearance < floor:
            failures.append(
                f'min clearance {clearance:.3f} m below the floor of {floor:.3f} m')

    # The scale-invariant safety term: clearance as a fraction of the most the
    # hull could clear the tightest gate by. Unlike an absolute floor, this
    # means the same thing after the gate widths change.
    frac_floor = require.get('min_clearance_fraction_at_least')
    fraction = card.get('clearance_fraction')
    if frac_floor is not None:
        if fraction is None:
            failures.append(
                'scorecard has no clearance_fraction; re-run with a task_run '
                'new enough to emit it')
        elif fraction < frac_floor:
            failures.append(
                f'clearance fraction {fraction:.3f} below the floor of '
                f'{frac_floor:.3f} (measured {clearance:.3f} m of a possible '
                f'{card.get("clearance_ceiling_m")} m)')

    # ---------------------------------------------------------------- drift
    ref_clear = reference.get('min_clearance_m')
    frac_warn = warn.get('clearance_fraction_below')
    if None not in (frac_warn, fraction) and fraction < frac_warn:
        warnings.append(
            f'clearance fraction {fraction:.3f} is below {frac_warn:.3f} '
            f'(reference {reference.get("clearance_fraction")})')

    ref_len = reference.get('path_length_m')
    excess = warn.get('path_length_m_excess_frac')
    length = card.get('path_length_m')
    if None not in (ref_len, excess, length) and length > ref_len * (1 + excess):
        warnings.append(
            f'course distance {length:.1f} m exceeds the reference '
            f'{ref_len:.1f} m by more than {excess:.0%}')

    ref_time = reference.get('elapsed_s')
    slower = warn.get('elapsed_s_excess_frac')
    elapsed = card.get('elapsed_s')
    if None not in (ref_time, slower, elapsed) and elapsed > ref_time * (1 + slower):
        warnings.append(
            f'elapsed {elapsed:.1f} s exceeds the reference {ref_time:.1f} s '
            f'by more than {slower:.0%}')

    worst_cap = warn.get('worst_offset_from_centre_m')
    worst = max([o for o in offsets if o is not None], default=None)
    if None not in (worst_cap, worst) and worst > worst_cap:
        warnings.append(
            f'worst gate offset {worst:.2f} m exceeds {worst_cap:.2f} m')

    # --------------------------------------------------------------- report
    task = contract.get('task', 'task')
    print(f'regression: {task} ({kind})')
    if total:
        print(f'  gates           : {transited}/{total}')
    if 'swept_rad' in card:
        print(f'  swept           : {card["swept_rad"]:.2f} rad '
              f'({card["laps_completed"]:.2f} laps {card.get("direction", "")})')
    print(f'  contact         : {"YES" if contact else "none"}')
    if clearance is not None:
        print(f'  min clearance   : {clearance:.3f} m'
              + (f'  (reference {ref_clear:.3f}'
                 + (f' +/- {reference["min_clearance_m_sd"]:.3f}'
                    if reference.get('min_clearance_m_sd') else '') + ')'
                 if ref_clear else ''))
    if fraction is not None:
        print(f'  of the possible : {fraction:.1%}'
              + (f'  (reference {reference["clearance_fraction"]:.1%})'
                 if reference.get('clearance_fraction') else ''))
    if card.get('elapsed_s'):
        print(f'  elapsed         : {card["elapsed_s"]:.1f} s (sim clock)')
    if length is not None:
        print(f'  course distance : {length:.1f} m'
              + (f'  (reference {ref_len:.1f})' if ref_len else ''))
    mean_off = _mean(offsets)
    if mean_off is not None:
        ref_off = reference.get('mean_offset_from_centre_m')
        print(f'  mean offset     : {mean_off:.2f} m'
              + (f'  (reference {ref_off:.2f})' if ref_off else ''))

    for w in warnings:
        print(f'  DRIFT   {w}')
    for f in failures:
        print(f'  FAIL    {f}')

    print(f'  verdict         : {"PASS" if not failures else "FAIL"}'
          + (f' with {len(warnings)} drift note(s)' if warnings else ''))
    return 1 if failures else 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scorecard', type=Path, required=True)
    parser.add_argument('--contract', type=Path, required=True)
    args = parser.parse_args()

    for path in (args.scorecard, args.contract):
        if not path.exists():
            print(f'FATAL: {path} does not exist')
            return 2

    return check(json.loads(args.scorecard.read_text()),
                 json.loads(args.contract.read_text()))


if __name__ == '__main__':
    sys.exit(main())
