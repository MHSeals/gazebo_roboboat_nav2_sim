#!/usr/bin/env python3
"""Build one self-contained HTML report covering a whole run through the course.

The per-run replay (tools/make_replay.py) is a playback: it shows you the boat
moving. This is the other thing you want afterwards -- every task on one page,
scored, with the numbers that decide pass or fail and the ones that only look
like they do.

    python3 tools/make_report.py \
        --run channel:tuning/showcase_channel_card.json:tuning/showcase_channel_trace.json \
        --run sprint:tuning/showcase_sprint_card.json:tuning/showcase_sprint_trace.json \
        --out report.html

No external requests: the traces are decimated and embedded, the SVG is drawn
inline, and there is no script tag pointing anywhere. It opens from a phone.

On colour. The tracks use the validated two-slot categorical palette, and every
series is also directly labelled so identity never rests on hue. The buoys do
NOT: a red lateral mark is drawn red because it *is* red, and which side of it
you pass is the whole task. That is a map legend, not a data encoding, and
mixing the two up would be the actual accessibility failure here.
"""

from __future__ import annotations

import argparse
import base64
import html
import json
import math
import sys
import tempfile
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from course_tasks import FOOTPRINT_HALF_Y  # noqa: E402

# Validated: 2 slots, --pairs all, PASS in both modes (CVD dE 24.7 light /
# 26.8 dark; normal-vision 33.6 / 31.8). See tools/validate_palette output in
# the journal. Do not add a third series without re-running the validator.
SERIES = [
    {'light': '#2a78d6', 'dark': '#3987e5'},
    {'light': '#eb6834', 'dark': '#d95926'},
]
STATUS = {'good': '#0ca30c', 'critical': '#d03b3b', 'warning': '#fab219'}

# Three roles, chosen from the subject rather than from habit. Chart lettering
# is condensed -- that is how depths and mark names are set on a real chart --
# so headings and every uppercase label are Open Sans Condensed. Running prose
# is Lato, which is quiet and is not the face every generated page reaches for.
# Every measured value is DejaVu Sans Mono, because a number on this page is an
# instrument reading and should look like one.
FONTS = {
    '__FONT_CHART__': ['/usr/share/fonts/truetype/open-sans/OpenSans-CondBold.ttf',
                       '/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf'],
    '__FONT_BODY__': ['/usr/share/fonts/truetype/lato/Lato-Regular.ttf',
                      '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'],
    '__FONT_BODYSB__': ['/usr/share/fonts/truetype/lato/Lato-Semibold.ttf',
                        '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'],
    '__FONT_MONO__': ['/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
                      '/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf'],
}
GLYPHS = ''.join(chr(c) for c in range(32, 127)) + '\u00b7\u2014\u2013\u00b0\u00b1\u00d7\u2192\u2191\u2193\u2248\u2713\u2717\u2011\u00a0'


def subset_font(path: Path) -> str:
    """Subset a TTF to WOFF2 and return it base64-encoded.

    The Artifact CSP blocks font CDNs, so a linked webfont would fall back
    silently and the page would quietly lose its typography. Inlining is the
    only way to be sure the face that ships is the face that renders.
    """
    from fontTools import subset
    from fontTools.ttLib import TTFont

    font = TTFont(str(path))
    options = subset.Options(flavor='woff2', desubroutinize=True,
                             layout_features=['kern', 'liga'])
    sub = subset.Subsetter(options=options)
    sub.populate(text=GLYPHS)
    sub.subset(font)
    with tempfile.NamedTemporaryFile(suffix='.woff2', delete=False) as handle:
        tmp = Path(handle.name)
    try:
        font.save(str(tmp))
        return base64.b64encode(tmp.read_bytes()).decode('ascii')
    finally:
        tmp.unlink(missing_ok=True)


def inline_fonts(page: str) -> str:
    for placeholder, candidates in FONTS.items():
        encoded = ''
        for candidate in candidates:
            path = Path(candidate)
            if not path.exists():
                continue
            try:
                encoded = subset_font(path)
                break
            except ImportError:
                print('fontTools not installed; system faces will be used',
                      file=sys.stderr)
                break
            except Exception as exc:                      # noqa: BLE001
                print(f'could not subset {path.name}: {exc}', file=sys.stderr)
        page = page.replace(placeholder, encoded)
    return page


# --------------------------------------------------------------- geometry


def bounds(runs: list[dict]) -> tuple[float, float, float, float]:
    xs, ys = [], []
    for r in runs:
        for o in r['trace']['obstacles']:
            reach = o.get('r', max(o.get('hx', 0), o.get('hy', 0)))
            xs += [o['x'] - reach, o['x'] + reach]
            ys += [o['y'] - reach, o['y'] + reach]
        for f in r['frames']:
            xs.append(f[1])
            ys.append(f[2])
    pad = 3.0
    return min(xs) - pad, min(ys) - pad, max(xs) + pad, max(ys) + pad


def decimate(frames: list[list], step: float = 0.4) -> list[list]:
    """Thin the trace for drawing. Geometry was already scored at full rate."""
    out, last = [], -1e9
    for f in frames:
        if f[0] - last >= step:
            out.append(f)
            last = f[0]
    if frames and out and out[-1] is not frames[-1]:
        out.append(frames[-1])
    return out


# ------------------------------------------------------------------- svg


def course_svg(runs: list[dict], width: int = 920) -> str:
    x0, y0, x1, y1 = bounds(runs)
    span_x, span_y = x1 - x0, y1 - y0
    scale = width / span_x
    height = int(span_y * scale)

    def px(x, y):
        # SVG y grows downward; the course does not.
        return (x - x0) * scale, (y1 - y) * scale

    parts = [
        f'<svg viewBox="0 0 {width} {height}" class="map" '
        f'role="img" aria-label="Course map: buoy positions and the track '
        f'driven for each task.">',
        f'<rect x="0" y="0" width="{width}" height="{height}" class="water"/>',
    ]

    # Obstacles, in their literal colours. Drawn once from the first run --
    # every task runs on the same course.
    for o in runs[0]['trace']['obstacles']:
        colour = o.get('colour', '#888888')
        if o.get('shape') == 'box':
            cx, cy = px(o['x'], o['y'])
            w, h = o['hx'] * 2 * scale, o['hy'] * 2 * scale
            deg = -math.degrees(o.get('yaw', 0.0))
            parts.append(
                f'<rect x="{cx - w / 2:.1f}" y="{cy - h / 2:.1f}" '
                f'width="{w:.1f}" height="{h:.1f}" fill="{colour}" '
                f'transform="rotate({deg:.1f} {cx:.1f} {cy:.1f})" '
                f'class="obst"/>')
        else:
            cx, cy = px(o['x'], o['y'])
            parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" '
                         f'r="{max(2.0, o["r"] * scale):.1f}" '
                         f'fill="{colour}" class="obst"/>')

    # Tracks. 2 px, and a surface-coloured casing underneath so a track
    # crossing another stays readable where they overlap.
    for i, r in enumerate(runs):
        pts = ' '.join(f'{px(f[1], f[2])[0]:.1f},{px(f[1], f[2])[1]:.1f}'
                       for f in r['draw'])
        parts.append(f'<polyline points="{pts}" class="track-casing"/>')
        parts.append(f'<polyline points="{pts}" class="track" '
                     f'style="stroke:var(--series-{i + 1})"/>')

    # The tightest moment of each run, which is the number the contract turns on.
    for i, r in enumerate(runs):
        tight = min(r['draw'], key=lambda f: f[11])
        cx, cy = px(tight[1], tight[2])
        parts.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="7" '
                     f'class="pinch" style="stroke:var(--series-{i + 1})"/>')
        parts.append(
            f'<text x="{cx + 11:.1f}" y="{cy + 4:.1f}" class="maplabel">'
            f'{r["name"]} tightest &#8212; {tight[11]:.2f} m</text>')

    # Start.
    sx, sy = px(runs[0]['draw'][0][1], runs[0]['draw'][0][2])
    parts.append(f'<circle cx="{sx:.1f}" cy="{sy:.1f}" r="4" class="startdot"/>')
    parts.append(f'<text x="{sx + 9:.1f}" y="{sy + 4:.1f}" class="maplabel">start</text>')

    # Scale bar, because every distance on this page is in metres.
    bar = 10.0
    bx, by = 16, height - 18
    parts.append(f'<line x1="{bx}" y1="{by}" x2="{bx + bar * scale:.1f}" '
                 f'y2="{by}" class="scalebar"/>')
    parts.append(f'<text x="{bx}" y="{by - 7}" class="maplabel">10 m</text>')
    parts.append('</svg>')
    return '\n'.join(parts)


CHARTS: list[dict] = []


def line_chart(series: list[dict], ylabel: str, unit: str,
               width: int = 440, height: int = 190,
               floor: float | None = None,
               floor_label: str = '') -> str:
    """One measure over time, one line per task, with a crosshair layer.

    Both tasks share an axis because both are seconds and both are the same
    measure -- which is the only condition under which two series belong on one
    pair of axes.
    """
    pad_l, pad_r, pad_t, pad_b = 46, 12, 14, 26
    xs = [p[0] for s in series for p in s['points']]
    ys = [p[1] for s in series for p in s['points']]
    if floor is not None:
        ys.append(floor)
    xmin, xmax = 0.0, max(xs)
    ymin, ymax = min(0.0, min(ys)), max(ys)
    if ymax - ymin < 1e-9:
        ymax = ymin + 1.0
    ymax *= 1.08

    def sx(v):
        return pad_l + (v - xmin) / (xmax - xmin) * (width - pad_l - pad_r)

    def sy(v):
        return height - pad_b - (v - ymin) / (ymax - ymin) * (height - pad_t - pad_b)

    out = [f'<svg viewBox="0 0 {width} {height}" class="chart" role="img" '
           f'aria-label="{html.escape(ylabel)} against time, one line per task. '
           f'The table below carries the same numbers.">']

    # Recessive grid.
    steps = 4
    for i in range(steps + 1):
        v = ymin + (ymax - ymin) * i / steps
        y = sy(v)
        out.append(f'<line x1="{pad_l}" y1="{y:.1f}" x2="{width - pad_r}" '
                   f'y2="{y:.1f}" class="grid"/>')
        out.append(f'<text x="{pad_l - 6}" y="{y + 3.5:.1f}" '
                   f'class="tick tick-y">{v:.2f}</text>')
    for i in range(5):
        v = xmin + (xmax - xmin) * i / 4
        out.append(f'<text x="{sx(v):.1f}" y="{height - 8}" '
                   f'class="tick tick-x">{v:.0f}</text>')

    if floor is not None:
        out.append(f'<line x1="{pad_l}" y1="{sy(floor):.1f}" '
                   f'x2="{width - pad_r}" y2="{sy(floor):.1f}" class="floor"/>')
        out.append(f'<text x="{width - pad_r}" y="{sy(floor) - 5:.1f}" '
                   f'class="floorlabel">{html.escape(floor_label)}</text>')

    for s in series:
        pts = ' '.join(f'{sx(t):.1f},{sy(v):.1f}' for t, v in s['points'])
        out.append(f'<polyline points="{pts}" class="line" '
                   f'style="stroke:var(--series-{s["slot"]})"/>')

    idx = len(CHARTS)
    CHARTS.append({
        'unit': unit,
        'label': ylabel,
        'xmin': xmin, 'xmax': xmax, 'ymin': ymin, 'ymax': ymax,
        'padl': pad_l, 'padr': pad_r, 'padt': pad_t, 'padb': pad_b,
        'w': width, 'h': height,
        'series': [{'name': s['name'], 'slot': s['slot'],
                    'points': [[round(t, 2), round(v, 3)]
                               for t, v in s['points']]}
                   for s in series],
    })
    out.append(f'<rect class="hit" x="{pad_l}" y="{pad_t}" '
               f'width="{width - pad_l - pad_r}" '
               f'height="{height - pad_t - pad_b}" data-chart="{idx}"/>')
    out.append(f'<line class="crosshair" y1="{pad_t}" y2="{height - pad_b}"/>')
    for i, _ in enumerate(series):
        out.append(f'<circle class="focus" r="4.5" data-slot="{i + 1}" '
                   f'style="fill:var(--series-{series[i]["slot"]})"/>')
    out.append('</svg>')
    return '\n'.join(out)


# ----------------------------------------------------------------- pieces


def stat(label: str, value: str, note: str = '', tone: str = '') -> str:
    cls = f' tone-{tone}' if tone else ''
    return (f'<div class="stat{cls}"><div class="stat-label">{html.escape(label)}</div>'
            f'<div class="stat-value">{value}</div>'
            + (f'<div class="stat-note">{note}</div>' if note else '')
            + '</div>')


def verdict_chip(passed: bool) -> str:
    if passed:
        return ('<span class="chip chip-good"><span class="chip-icon" '
                'aria-hidden="true">&#10003;</span>PASS</span>')
    return ('<span class="chip chip-bad"><span class="chip-icon" '
            'aria-hidden="true">&#10007;</span>FAIL</span>')


def task_section(run: dict, slot: int) -> str:
    card = run['card']
    name = run['name']
    stats = [
        stat('result', verdict_chip(card.get('passed', False))),
        stat('elapsed', f'{card.get("elapsed_s", 0):.1f}<span class="u">s</span>',
             'simulator clock'),
        stat('distance', f'{card.get("path_length_m", 0):.1f}<span class="u">m</span>',
             'driven'),
    ]

    if card.get('type') == 'gate_transit':
        stats.insert(1, stat(
            'gates',
            f'{card["gates_transited"]}<span class="u">/{card["gates_total"]}</span>',
            'transited in order, between the buoys'))
    else:
        stats.insert(1, stat(
            'laps', f'{card.get("laps_completed", 0):.2f}',
            f'{card.get("direction", "")} about {html.escape(str(card.get("mark", "")))}'))

    frac = card.get('clearance_fraction')
    ceiling = card.get('clearance_ceiling_m')
    stats.append(stat(
        'closest pass',
        f'{card.get("min_clearance_m", 0):.3f}<span class="u">m</span>',
        (f'{frac:.0%} of the {ceiling:.2f} m a perfect centreline allows'
         if frac and ceiling else 'hull to nearest buoy surface')))

    rows = ''
    if card.get('type') == 'gate_transit':
        rows = '<div class="tblwrap"><table class="tbl">' \
               '<caption>Every gate, as scored</caption><thead><tr>' \
               '<th scope="col">gate</th><th scope="col">transited</th>' \
               '<th scope="col">offset from centre</th>' \
               '<th scope="col">free water</th></tr></thead><tbody>'
        for g in card['gates']:
            off = (f'{g["offset_from_centre_m"]:.2f} m'
                   if g['offset_from_centre_m'] is not None else '—')
            mark = ('<span class="yes">yes</span>' if g['transited']
                    else '<span class="no">MISSED</span>')
            rows += (f'<tr><td>{html.escape(g["gate"])}</td><td>{mark}</td>'
                     f'<td class="num">{off}</td>'
                     f'<td class="num">{g["free_water_m"]:.2f} m</td></tr>')
        rows += '</tbody></table></div>'
    else:
        need = card.get('laps_required_rad', 2 * math.pi)
        rows = ('<div class="tblwrap"><table class="tbl">'
                '<caption>How the lap was scored</caption><tbody>'
                f'<tr><th scope="row">swept about the mark</th>'
                f'<td class="num">{card["swept_rad"]:.3f} rad</td></tr>'
                f'<tr><th scope="row">required</th>'
                f'<td class="num">{need:.3f} rad</td></tr>'
                f'<tr><th scope="row">gate subtense allowed</th>'
                f'<td class="num">{card.get("gate_slack_rad", 0):.3f} rad</td></tr>'
                f'<tr><th scope="row">entrance gate outbound</th>'
                f'<td>{"yes" if card.get("entered") else "NO"}</td></tr>'
                f'<tr><th scope="row">entrance gate inbound</th>'
                f'<td>{"yes" if card.get("exited") else "NO"}</td></tr>'
                f'<tr><th scope="row">counter-winding laundered</th>'
                f'<td>{"YES" if card.get("backtracked") else "no"}</td></tr>'
                '</tbody></table></div>')

    return (f'<section class="task"><div class="task-head">'
            f'<h2 id="task-{html.escape(name)}">{html.escape(name)}</h2>'
            f'<span class="swatch" style="background:var(--series-{slot})" '
            f'aria-hidden="true"></span>'
            f'<span class="task-kind">{html.escape(card.get("type", ""))}</span>'
            f'</div><div class="stats">{"".join(stats)}</div>{rows}</section>')


# ------------------------------------------------------------------ page


def build(runs: list[dict], title: str) -> str:
    for i, r in enumerate(runs):
        r['slot'] = i + 1
        r['frames'] = r['trace']['frames']
        r['draw'] = decimate(r['frames'])

    clearance = [{'slot': r['slot'], 'name': r['name'],
                  'points': [(f[0], f[11]) for f in r['draw'] if f[11] < 50]}
                 for r in runs]
    speed = [{'slot': r['slot'], 'name': r['name'],
              'points': [(f[0], math.hypot(f[4], f[5])) for f in r['draw']]}
             for r in runs]

    legend = ''.join(
        f'<span class="leg"><span class="swatch" '
        f'style="background:var(--series-{r["slot"]})" aria-hidden="true"></span>'
        f'{html.escape(r["name"])}</span>' for r in runs)

    all_pass = all(r['card'].get('passed') for r in runs)
    total_t = sum(r['card'].get('elapsed_s', 0) for r in runs)
    total_d = sum(r['card'].get('path_length_m', 0) for r in runs)
    tightest = min(r['card'].get('min_clearance_m', 9e9) for r in runs)

    hero = ''.join([
        stat('run verdict', verdict_chip(all_pass)),
        stat('tasks', f'{sum(1 for r in runs if r["card"].get("passed"))}'
                      f'<span class="u">/{len(runs)}</span>', 'scored as the task is'),
        stat('total time', f'{total_t:.0f}<span class="u">s</span>',
             'simulator clock, both tasks'),
        stat('total distance', f'{total_d:.0f}<span class="u">m</span>', 'driven'),
        stat('tightest pass', f'{tightest:.3f}<span class="u">m</span>',
             'hull to buoy, across the whole run'),
    ])

    tbl_rows = ''
    for r in runs:
        c = r['card']
        tbl_rows += (
            f'<tr><td>{html.escape(r["name"])}</td>'
            f'<td>{"PASS" if c.get("passed") else "FAIL"}</td>'
            f'<td class="num">{c.get("elapsed_s", 0):.1f}</td>'
            f'<td class="num">{c.get("path_length_m", 0):.1f}</td>'
            f'<td class="num">{c.get("min_clearance_m", 0):.3f}</td>'
            f'<td class="num">'
            + (f'{c["clearance_fraction"]:.0%}' if c.get('clearance_fraction')
               else '—')
            + '</td></tr>')

    sections = ''.join(task_section(r, r['slot']) for r in runs)

    clearance_svg = line_chart(clearance, 'clearance', 'm', floor=0.0,
                               floor_label='contact')
    speed_svg = line_chart(speed, 'speed over ground', 'm/s')

    return TEMPLATE.format(
        title=html.escape(title),
        hero=hero,
        legend=legend,
        map_svg=course_svg(runs),
        clearance_chart=clearance_svg,
        speed_chart=speed_svg,
        charts_json=json.dumps(CHARTS, separators=(',', ':')),
        sections=sections,
        summary_rows=tbl_rows,
    )


TEMPLATE = """<title>{title}</title>
<style>
  @font-face {{ font-family: "ChartLettering"; font-weight: 700;
    font-display: swap;
    src: url(data:font/woff2;base64,__FONT_CHART__) format("woff2"); }}
  @font-face {{ font-family: "ReportBody"; font-weight: 400;
    font-display: swap;
    src: url(data:font/woff2;base64,__FONT_BODY__) format("woff2"); }}
  @font-face {{ font-family: "ReportBody"; font-weight: 600;
    font-display: swap;
    src: url(data:font/woff2;base64,__FONT_BODYSB__) format("woff2"); }}
  @font-face {{ font-family: "Readout"; font-weight: 400;
    font-display: swap;
    src: url(data:font/woff2;base64,__FONT_MONO__) format("woff2"); }}

  /* Neutrals are biased cool -- a chart is printed on cool stock with a cyan
     water tint, not on warm cream. The accent is the same blue the channel
     track is drawn in, so the page's accent and its data agree rather than
     competing. */
  .viz-root {{
    color-scheme: light;
    --surface-0: #f6f8f8; --surface-1: #ffffff; --surface-2: #eaf0f1;
    --border: #d3dde0; --hair: #c2d0d4; --grid: #e2eaec;
    --text-primary: #0f191d; --text-secondary: #47585e; --text-muted: #5d6f75;
    --series-1: #2a78d6; --series-2: #eb6834;
    --good: #0ca30c; --critical: #d03b3b;
    --water: #e4eef1;

    --f-chart: "ChartLettering", "Open Sans Condensed", ui-sans-serif, sans-serif;
    --f-body: "ReportBody", ui-sans-serif, system-ui, sans-serif;
    --f-mono: "Readout", ui-monospace, "DejaVu Sans Mono", monospace;
  }}
  @media (prefers-color-scheme: dark) {{
    :root:where(:not([data-theme="light"])) .viz-root {{
      color-scheme: dark;
      --surface-0: #0d1417; --surface-1: #141d21; --surface-2: #1b262b;
      --border: #26343a; --hair: #33454c; --grid: #1f2c31;
      --text-primary: #eef4f5; --text-secondary: #a6b7bd; --text-muted: #7c8e94;
      --series-1: #3987e5; --series-2: #d95926;
      --water: #0f1c22;
    }}
  }}
  :root[data-theme="dark"] .viz-root {{
    color-scheme: dark;
    --surface-0: #0d1417; --surface-1: #141d21; --surface-2: #1b262b;
    --border: #26343a; --hair: #33454c; --grid: #1f2c31;
    --text-primary: #eef4f5; --text-secondary: #a6b7bd; --text-muted: #7c8e94;
    --series-1: #3987e5; --series-2: #d95926;
    --water: #0f1c22;
  }}

  .viz-root {{
    background: var(--surface-0); color: var(--text-primary);
    font: 400 16px/1.6 var(--f-body);
    padding: 40px 20px 72px; margin: 0 auto; max-width: 1000px;
    -webkit-text-size-adjust: 100%;
  }}
  .viz-root *:focus-visible {{ outline: 2px solid var(--series-1);
                              outline-offset: 2px; }}

  .eyebrow {{ font: 700 .74rem/1 var(--f-chart); text-transform: uppercase;
             letter-spacing: .16em; color: var(--text-muted); }}
  h1 {{ font: 700 clamp(1.9rem, 4.5vw, 2.6rem)/1.06 var(--f-chart);
       text-transform: uppercase; letter-spacing: .01em;
       margin: 10px 0 0; text-wrap: balance; }}
  h2 {{ font: 700 1.32rem/1.15 var(--f-chart); text-transform: uppercase;
       letter-spacing: .04em; margin: 0; }}
  h3 {{ font: 700 .78rem/1 var(--f-chart); text-transform: uppercase;
       letter-spacing: .16em; color: var(--text-muted);
       margin: 0 0 14px; padding-bottom: 9px;
       border-bottom: 1px solid var(--hair); }}
  .sub {{ color: var(--text-secondary); margin: 12px 0 0; max-width: 62ch;
         font-size: 1.02rem; }}
  .masthead {{ padding-bottom: 26px; border-bottom: 2px solid var(--hair);
              margin-bottom: 32px; }}
  section, .block {{ margin-top: 40px; }}

  .stats {{ display: grid; gap: 1px; margin: 18px 0 0;
           background: var(--border); border: 1px solid var(--border);
           grid-template-columns: repeat(auto-fit, minmax(158px, 1fr)); }}
  .stat {{ background: var(--surface-1); padding: 14px 16px 15px; }}
  .stat-label {{ font: 700 .68rem/1 var(--f-chart); text-transform: uppercase;
                letter-spacing: .14em; color: var(--text-muted); }}
  .stat-value {{ font: 400 1.62rem/1.15 var(--f-mono); margin-top: 7px;
                font-variant-numeric: tabular-nums;
                color: var(--text-primary); letter-spacing: -.02em; }}
  .stat-value .u {{ font-size: .82rem; color: var(--text-secondary);
                   margin-left: 2px; }}
  .stat-note {{ font-size: .78rem; color: var(--text-secondary);
               margin-top: 7px; line-height: 1.4; }}

  .chip {{ display: inline-flex; align-items: center; gap: 7px;
          font: 700 1.15rem/1.15 var(--f-chart); text-transform: uppercase;
          letter-spacing: .08em; }}
  .chip-icon {{ font-size: 1rem; }}
  .chip-good {{ color: var(--good); }}
  .chip-bad  {{ color: var(--critical); }}

  figure {{ margin: 0; }}
  figcaption {{ font-size: .82rem; color: var(--text-secondary); margin-top: 10px;
               max-width: 68ch; }}
  .legend {{ display: flex; flex-wrap: wrap; gap: 18px; margin: 0 0 12px;
            font: 700 .72rem/1 var(--f-chart); text-transform: uppercase;
            letter-spacing: .1em; color: var(--text-secondary); }}
  .leg {{ display: inline-flex; align-items: center; gap: 7px; }}
  .swatch {{ width: 10px; height: 10px; display: inline-block; }}

  /* Wide content scrolls inside its own box; the page body never does. */
  .mapwrap, .chartwrap, .tblwrap {{ overflow-x: auto;
    -webkit-overflow-scrolling: touch; }}
  svg.map {{ width: 100%; min-width: 560px; height: auto;
            border: 1px solid var(--hair); display: block;
            background: var(--water); }}
  .water {{ fill: var(--water); }}
  .obst {{ opacity: .95; }}
  .track-casing {{ fill: none; stroke: var(--water); stroke-width: 5;
                  stroke-linejoin: round; stroke-linecap: round; }}
  .track {{ fill: none; stroke-width: 2; stroke-linejoin: round;
           stroke-linecap: round; }}
  .pinch {{ fill: none; stroke-width: 2; }}
  .startdot {{ fill: var(--text-primary); }}
  .maplabel {{ font: 700 11px var(--f-chart); text-transform: uppercase;
              letter-spacing: .07em;
              fill: var(--text-secondary); paint-order: stroke;
              stroke: var(--water); stroke-width: 3.5px; }}
  .scalebar {{ stroke: var(--text-secondary); stroke-width: 2; }}

  .charts {{ display: grid; gap: 20px;
            grid-template-columns: repeat(auto-fit, minmax(330px, 1fr)); }}
  svg.chart {{ width: 100%; height: auto; display: block;
              background: var(--surface-1); border: 1px solid var(--hair);
              touch-action: pan-y; }}
  .grid {{ stroke: var(--grid); stroke-width: 1; }}
  .tick {{ font: 10.5px var(--f-mono); fill: var(--text-muted);
          font-variant-numeric: tabular-nums; }}
  .tick-y {{ text-anchor: end; }}
  .tick-x {{ text-anchor: middle; }}
  .line {{ fill: none; stroke-width: 2; stroke-linejoin: round;
          stroke-linecap: round; }}
  .floor {{ stroke: var(--critical); stroke-width: 1.5;
           stroke-dasharray: 4 3; }}
  .floorlabel {{ font: 700 10px var(--f-chart); fill: var(--critical);
                text-anchor: end; text-transform: uppercase;
                letter-spacing: .12em; }}
  .hit {{ fill: transparent; }}
  .crosshair {{ stroke: var(--text-muted); stroke-width: 1;
               stroke-dasharray: 3 3; opacity: 0; }}
  .focus {{ opacity: 0; stroke: var(--surface-1); stroke-width: 2; }}

  .tip {{ position: fixed; z-index: 9; pointer-events: none; opacity: 0;
         transition: opacity .08s; background: var(--surface-1);
         border: 1px solid var(--hair);
         padding: 9px 11px; font: 400 .78rem/1.5 var(--f-mono);
         color: var(--text-primary);
         box-shadow: 0 8px 26px rgba(0,0,0,.22); max-width: 260px; }}
  .tip b {{ font-variant-numeric: tabular-nums; }}
  .tip .row {{ display: flex; align-items: center; gap: 6px;
              white-space: nowrap; }}

  .task {{ border-top: 1px solid var(--hair); padding-top: 24px;
          margin-top: 34px; }}
  .task-head {{ display: flex; align-items: center; gap: 11px;
               flex-wrap: wrap; }}
  .task-kind {{ font: 700 .68rem/1 var(--f-chart); text-transform: uppercase;
               letter-spacing: .12em; color: var(--text-muted);
               background: var(--surface-2); padding: 5px 9px; }}

  /* color is set explicitly rather than inherited: without a doctype the
     page renders in quirks mode, where a table does NOT inherit colour from
     its ancestors, and the whole table body came out black-on-black in dark
     mode. Measured at 1.13:1. */
  table.tbl {{ border-collapse: collapse; width: 100%; margin-top: 20px;
              font-size: .86rem; min-width: 420px;
              color: var(--text-primary); }}
  table.tbl caption {{ text-align: left; font: 700 .68rem/1 var(--f-chart);
                      text-transform: uppercase; letter-spacing: .14em;
                      color: var(--text-muted); padding-bottom: 9px; }}
  .tbl th, .tbl td {{ border-bottom: 1px solid var(--border);
                     padding: 6px 8px; text-align: left;
                     color: var(--text-primary); }}
  .tbl thead th {{ font: 700 .68rem/1.3 var(--f-chart); text-transform: uppercase;
                  letter-spacing: .12em; color: var(--text-muted);
                  border-bottom-color: var(--hair); }}
  .tbl th[scope="row"] {{ font-weight: 500; color: var(--text-secondary); }}
  .num {{ text-align: right; font-family: var(--f-mono); font-size: .82rem;
         font-variant-numeric: tabular-nums; }}
  .yes {{ color: var(--good); font-weight: 600; }}
  .no {{ color: var(--critical); font-weight: 700;
        font-family: var(--f-chart); letter-spacing: .08em; }}

  .note {{ background: var(--surface-1); border: 1px solid var(--hair);
          padding: 20px 22px; margin-top: 16px;
          font-size: .88rem; color: var(--text-secondary); }}
  .note p {{ margin: 0 0 9px; }} .note p:last-child {{ margin: 0; }}
  .note strong {{ color: var(--text-primary); }}
</style>

<div class="viz-root">
  <header class="masthead">
    <div class="eyebrow">Nav2 MPPI &#183; Gazebo Harmonic &#183; ROS&#160;2 Jazzy</div>
    <h1>{title}</h1>
    <p class="sub">One pass through the RoboBoat course in the physics-free
    simulator. Each task is scored the way the competition scores it &#8212; not
    by whether navigation reported success, which it does for runs that miss
    the point of the task entirely.</p>
  </header>

  <div class="stats">{hero}</div>

  <section>
  <h3>The course, and what was driven</h3>
  <div class="legend">{legend}
    <span class="leg"><span class="swatch" style="background:#d81a1a"
      aria-hidden="true"></span>red mark</span>
    <span class="leg"><span class="swatch" style="background:#1ab340"
      aria-hidden="true"></span>green mark</span>
  </div>
  <figure>
    <div class="mapwrap">{map_svg}</div>
    <figcaption>Buoys are drawn in their own colours because which side you
    pass is the task, not a series encoding. Rings mark each run&#8217;s closest
    approach to anything on the course.</figcaption>
  </figure>
  </section>

  <section>
  <h3>Clearance and speed through the run</h3>
  <div class="charts">
    <figure>
      <div class="chartwrap">{clearance_chart}</div>
      <figcaption>Distance from the hull to the nearest buoy surface, seconds
      into each task. Touching is zero, not the axis floor.</figcaption>
    </figure>
    <figure>
      <div class="chartwrap">{speed_chart}</div>
      <figcaption>Speed over ground. The sprint is slower throughout &#8212;
      circling costs yaw the channel never pays.</figcaption>
    </figure>
  </div>
  </section>

  <section>
  <h3>Task by task</h3>
  {sections}
  </section>

  <section>
  <h3>The ledger</h3>
  <div class="tblwrap"><table class="tbl">
    <caption>The same numbers as above, for reading rather than looking</caption>
    <thead><tr><th scope="col">task</th><th scope="col">result</th>
      <th scope="col">time (s)</th><th scope="col">distance (m)</th>
      <th scope="col">closest (m)</th>
      <th scope="col">of possible</th></tr></thead>
    <tbody>{summary_rows}</tbody>
  </table></div>
  </section>

  <section>
  <h3>What this is and is not</h3>
  <div class="note">
    <p>The water has no hydrodynamics.
    The hull is driven by a surrogate 3&#8209;DOF model with thruster
    allocation and lag; buoys are visual-only and the lidar rasterises them.
    That is deliberate &#8212; the point of this sim is the navigation stack, and
    it holds real&#8209;time factor 1.0 with MPPI at 20&#160;Hz on four cores.</p>
    <p>Perception is stood in for by a single scan plane. Colour classification,
    placard reading and payload delivery are all above that line and are not
    simulated; where a task is scored on what the boat reports, this sim would
    publish ground truth on the topic a real classifier would use, so that
    everything downstream of perception is genuinely exercised and nothing
    pretends to be perception.</p>
    <p><strong>Closest pass is the number to watch.</strong> Reported as a
    fraction of the most a perfectly centred hull could clear that gate by, so
    it stays comparable when gate widths change. The sprint gate is 2.4&#160;m
    against the channel&#8217;s 2.8&#160;m, so its ceiling is 0.47&#160;m rather
    than 0.65&#160;m &#8212; and the boat uses more of it.</p>
    <p><strong>These runs were filmed, and getting there took five
    attempts.</strong> Rendering the follow camera in software on the same four
    cores that run MPPI starved the control loop badly enough to abort healthy
    runs &#8212; once at 0.91 of a completed lap, on a goal acknowledgement
    rather than anything to do with navigation. The camera is down to
    480&#215;300 at 4&#160;Hz and the acknowledgement window is up to
    20&#160;s. Both numbers exist because a camera that changes the result is
    not observing the run.</p>
  </div>
  </section>
</div>

<div class="tip" id="tip" role="status" aria-live="polite"></div>
<script>
(function () {{
  var CHARTS = {charts_json};
  var tip = document.getElementById('tip');

  document.querySelectorAll('svg.chart').forEach(function (svg) {{
    var hit = svg.querySelector('.hit');
    if (!hit) return;
    var cfg = CHARTS[+hit.dataset.chart];
    var cross = svg.querySelector('.crosshair');
    var dots = [].slice.call(svg.querySelectorAll('.focus'));

    function sx(v) {{
      return cfg.padl + (v - cfg.xmin) / (cfg.xmax - cfg.xmin)
             * (cfg.w - cfg.padl - cfg.padr);
    }}
    function sy(v) {{
      return cfg.h - cfg.padb - (v - cfg.ymin) / (cfg.ymax - cfg.ymin)
             * (cfg.h - cfg.padt - cfg.padb);
    }}
    function nearest(pts, t) {{
      var lo = 0, hi = pts.length - 1;
      while (lo < hi) {{
        var mid = (lo + hi) >> 1;
        if (pts[mid][0] < t) lo = mid + 1; else hi = mid;
      }}
      if (lo > 0 && Math.abs(pts[lo - 1][0] - t) < Math.abs(pts[lo][0] - t)) lo--;
      return pts[lo];
    }}

    function move(ev) {{
      var t = ev.touches ? ev.touches[0] : ev;
      var pt = svg.createSVGPoint();
      pt.x = t.clientX; pt.y = t.clientY;
      var loc = pt.matrixTransform(svg.getScreenCTM().inverse());
      var frac = (loc.x - cfg.padl) / (cfg.w - cfg.padl - cfg.padr);
      frac = Math.max(0, Math.min(1, frac));
      var time = cfg.xmin + frac * (cfg.xmax - cfg.xmin);

      var px = sx(time);
      cross.setAttribute('x1', px); cross.setAttribute('x2', px);
      cross.style.opacity = 1;

      var rows = '<div class="row"><b>' + time.toFixed(1) + ' s</b></div>';
      cfg.series.forEach(function (s, i) {{
        var dot = dots[i];
        if (time > s.points[s.points.length - 1][0] || !s.points.length) {{
          if (dot) dot.style.opacity = 0;
          return;
        }}
        var p = nearest(s.points, time);
        if (dot) {{
          dot.setAttribute('cx', sx(p[0]));
          dot.setAttribute('cy', sy(p[1]));
          dot.style.opacity = 1;
        }}
        rows += '<div class="row"><span class="swatch" style="background:var(--series-'
             + s.slot + ')"></span>' + s.name
             + ' <b>' + p[1].toFixed(2) + '</b> ' + cfg.unit + '</div>';
      }});
      tip.innerHTML = rows;
      tip.style.opacity = 1;
      var w = tip.offsetWidth || 200;
      tip.style.left = Math.max(8, Math.min(window.innerWidth - w - 8,
                                            t.clientX + 16)) + 'px';
      tip.style.top = Math.max(8, t.clientY - 14) + 'px';
    }}
    function leave() {{
      tip.style.opacity = 0;
      cross.style.opacity = 0;
      dots.forEach(function (d) {{ d.style.opacity = 0; }});
    }}
    hit.addEventListener('mousemove', move);
    hit.addEventListener('mouseleave', leave);
    hit.addEventListener('touchstart', move, {{passive: true}});
    hit.addEventListener('touchmove', move, {{passive: true}});
    hit.addEventListener('touchend', leave);
  }});
}})();
</script>
"""


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', action='append', required=True,
                        metavar='NAME:CARD:TRACE',
                        help='repeatable; name, scorecard path, trace path')
    parser.add_argument('--out', type=Path, default=Path('report.html'))
    parser.add_argument('--title', default='RoboBoat course run')
    args = parser.parse_args()

    runs = []
    for spec in args.run:
        name, card_p, trace_p = spec.split(':', 2)
        card = json.loads(Path(card_p).read_text())
        trace = json.loads(Path(trace_p).read_text())
        runs.append({'name': name, 'card': card, 'trace': trace})

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(inline_fonts(build(runs, args.title)))
    kb = args.out.stat().st_size / 1024
    print(f'wrote {args.out} ({kb:.0f} kB, {len(runs)} task(s))')
    return 0


if __name__ == '__main__':
    sys.exit(main())
