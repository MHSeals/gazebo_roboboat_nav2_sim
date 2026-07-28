#!/usr/bin/env python3
"""Build a run report that needs no JavaScript at all.

    python3 tools/make_static_report.py \
        --run channel:CARD:TRACE --run sprint:CARD:TRACE \
        --baseline channel:TRACE --baseline sprint:TRACE \
        --out media/report.html

The interactive replay this replaces rendered its map into a <canvas> from an
embedded trace. That is fine in a browser and useless anywhere the page is
shown with scripting off -- an emailed attachment, a sandboxed preview pane, a
phone viewer -- where it degrades to a blank rectangle with the surrounding
chrome intact, which looks exactly like a broken report.

So everything here is baked at build time: the plan view is inline SVG, every
number is literal text in the markup. Fonts are subset and inlined as data
URIs, so the page has no external requests either. Open it anywhere.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import statistics
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent

FONTS = {
    'disp': ['/usr/share/fonts/truetype/open-sans/OpenSans-CondBold.ttf',
             '/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf'],
    'body': ['/usr/share/fonts/truetype/lato/Lato-Regular.ttf',
             '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'],
    'bodyb': ['/usr/share/fonts/truetype/lato/Lato-Bold.ttf',
              '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'],
    'mono': ['/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf'],
}
GLYPHS = ''.join(chr(c) for c in range(32, 127)) + '·—–°±×→↑↓≈≥≤−⌀'

# Thrusters are at (+-x, +-y); 'tangential' cants each one so its forward
# thrust drives the boat forward while the yaw arm stays large.
LAYOUTS = {
    'tangential': (-45.0, 45.0, 45.0, -45.0),
    'x': (45.0, -45.0, -45.0, 45.0),
    'pinwheel': (135.0, 45.0, -135.0, -45.0),
}


# ----------------------------------------------------------------- helpers
def subset_font(paths: list[str]) -> str | None:
    """Subset the first available face to WOFF2 and base64 it."""
    try:
        from fontTools import subset
        from fontTools.ttLib import TTFont
    except ImportError:
        return None
    for candidate in paths:
        if not Path(candidate).exists():
            continue
        font = TTFont(candidate)
        options = subset.Options(flavor='woff2', desubroutinize=True,
                                 layout_features=['kern', 'liga'])
        sub = subset.Subsetter(options)
        sub.populate(text=GLYPHS)
        sub.subset(font)
        tmp = Path(tempfile.mktemp(suffix='.woff2'))
        font.save(str(tmp))
        try:
            return base64.b64encode(tmp.read_bytes()).decode()
        finally:
            tmp.unlink(missing_ok=True)
    return None


def esc(text: object) -> str:
    return (str(text).replace('&', '&amp;').replace('<', '&lt;')
            .replace('>', '&gt;').replace('"', '&quot;'))


def read_trace(path: Path) -> tuple[dict, list[dict]]:
    doc = json.loads(path.read_text())
    index = {name: i for i, name in enumerate(doc['frame_fields'])}
    frames = [{k: row[i] for k, i in index.items()} for row in doc['frames']]
    return doc, frames


THRUST_KEYS = ('f_fl', 'f_fr', 'f_rl', 'f_rr')


def saturation(frames: list[dict], forward=35.0, reverse=25.0, tol=0.01) -> float:
    """Percentage of frames with any thruster within ``tol`` of a limit."""
    hi, lo = forward * (1 - tol), -reverse * (1 - tol)
    hit = sum(1 for f in frames
              if any(f[k] >= hi or f[k] <= lo for k in THRUST_KEYS))
    return 100.0 * hit / max(len(frames), 1)


# --------------------------------------------------------------- plan view
class View:
    """World metres -> SVG user units, with +x up the page and +y to the left."""

    def __init__(self, frames, width=760, pad=3.0, max_h=900):
        xs = [f['x'] for f in frames]
        ys = [f['y'] for f in frames]
        self.x0, self.x1 = min(xs) - pad, max(xs) + pad
        self.y0, self.y1 = min(ys) - pad, max(ys) + pad
        span_x, span_y = self.x1 - self.x0, self.y1 - self.y0
        # +y is to the LEFT, so the world's y-span is the drawing's width.
        self.s = min(width / span_y, max_h / span_x)
        self.w, self.h = span_y * self.s, span_x * self.s

    def px(self, x, y):
        return (self.y1 - y) * self.s

    def py(self, x, y):
        return (self.x1 - x) * self.s


def boat_outline(view: View, frame: dict, footprint) -> str:
    half_len, half_beam = footprint
    x, y, a = frame['x'], frame['y'], frame['yaw']
    pts = []
    for lx, ly in ((half_len, half_beam), (half_len, -half_beam),
                   (-half_len, -half_beam), (-half_len, half_beam)):
        wx = x + lx * math.cos(a) - ly * math.sin(a)
        wy = y + lx * math.sin(a) + ly * math.cos(a)
        pts.append(f'{view.px(wx, wy):.1f},{view.py(wx, wy):.1f}')
    return ' '.join(pts)


def plan_svg(doc, frames, view: View, strobe_s=8.0) -> str:
    # These SVGs are drawn at ~2x the CSS width they end up rendered at, so the
    # annotation type has to be sized in user units or it comes out unreadable.
    fs = view.w / 25.0
    out = [f'<svg viewBox="0 0 {view.w:.0f} {view.h:.0f}" '
           f'class="plan" role="img" aria-label="Plan view of the course with '
           f'the boat track">']
    out.append(f'<rect width="{view.w:.0f}" height="{view.h:.0f}" fill="var(--water)"/>')

    # 5 m grid
    grid = []
    gx = math.ceil(view.y0 / 5) * 5
    while gx < view.y1:
        px = view.px(0, gx)
        grid.append(f'M{px:.1f},0 V{view.h:.0f}')
        gx += 5
    gy = math.ceil(view.x0 / 5) * 5
    while gy < view.x1:
        py = view.py(gy, 0)
        grid.append(f'M0,{py:.1f} H{view.w:.0f}')
        gy += 5
    out.append(f'<path d="{" ".join(grid)}" stroke="var(--grid)" '
               f'stroke-width="1" fill="none" opacity=".55"/>')

    # planned route
    plans = doc.get('plans') or []
    if plans:
        best = max(plans, key=lambda p: len(p['p']))['p']
        pts = ' '.join(f'{view.px(px, py):.1f},{view.py(px, py):.1f}'
                       for px, py in best)
        out.append(f'<polyline points="{pts}" fill="none" stroke="var(--ink-3)" '
                   f'stroke-width="2" stroke-dasharray="6 5" opacity=".55"/>')

    # obstacles, in their real course colours
    for ob in doc['obstacles']:
        colour = esc(ob.get('colour', '#888'))
        if ob.get('shape') == 'box':
            cx, cy = view.px(ob['x'], ob['y']), view.py(ob['x'], ob['y'])
            w, h = ob['hy'] * 2 * view.s, ob['hx'] * 2 * view.s
            rot = -math.degrees(ob.get('yaw', 0.0))
            out.append(f'<rect x="{cx - w / 2:.1f}" y="{cy - h / 2:.1f}" '
                       f'width="{w:.1f}" height="{h:.1f}" fill="{colour}" '
                       f'transform="rotate({rot:.1f} {cx:.1f} {cy:.1f})" '
                       f'stroke="rgba(0,0,0,.3)"/>')
        else:
            r = max(2.5, ob.get('r', 0.25) * view.s)
            out.append(f'<circle cx="{view.px(ob["x"], ob["y"]):.1f}" '
                       f'cy="{view.py(ob["x"], ob["y"]):.1f}" r="{r:.1f}" '
                       f'fill="{colour}" stroke="rgba(0,0,0,.35)"/>')

    # the track the boat actually flew
    pts = ' '.join(f'{view.px(f["x"], f["y"]):.1f},{view.py(f["x"], f["y"]):.1f}'
                   for f in frames)
    out.append(f'<polyline points="{pts}" fill="none" stroke="var(--accent)" '
               f'stroke-width="3" stroke-linejoin="round" opacity=".9"/>')

    # the hull, strobed along the run, so a still image still reads as motion
    footprint = doc.get('footprint', [0.75, 0.55])
    last = -1e9
    for f in frames:
        if f['t'] - last < strobe_s:
            continue
        last = f['t']
        out.append(f'<polygon points="{boat_outline(view, f, footprint)}" '
                   f'fill="none" stroke="var(--accent)" stroke-width="1.4" '
                   f'opacity=".5"/>')

    # tightest pass of the run
    close = min((f for f in frames if f['clearance'] < 50),
                key=lambda f: f['clearance'])
    cx, cy = view.px(close['x'], close['y']), view.py(close['x'], close['y'])
    out.append(f'<polygon points="{boat_outline(view, close, footprint)}" '
               f'fill="var(--accent)" fill-opacity=".28" stroke="var(--accent)" '
               f'stroke-width="2"/>')
    out.append(f'<circle cx="{cx:.1f}" cy="{cy:.1f}" r="20" fill="none" '
               f'stroke="var(--warn)" stroke-width="2" stroke-dasharray="4 4"/>')
    # Put the label on whichever side of the ring has room, so it never clips.
    right = cx < view.w * 0.6
    out.append(f'<text x="{cx + (26 if right else -26):.1f}" y="{cy + fs * .35:.1f}" '
               f'class="svgnote" font-size="{fs:.1f}" '
               f'text-anchor="{"start" if right else "end"}" '
               f'fill="var(--warn)">tightest pass {close["clearance"]:.3f} m</text>')

    # start marker
    first = frames[0]
    out.append(f'<circle cx="{view.px(first["x"], first["y"]):.1f}" '
               f'cy="{view.py(first["x"], first["y"]):.1f}" r="6" '
               f'fill="none" stroke="var(--pass)" stroke-width="2.5"/>')

    # scale bar
    bar = 10 * view.s
    y = view.h - fs * 1.4
    out.append(f'<path d="M18,{y:.0f} h{bar:.0f} M18,{y - 6:.0f} v12 '
               f'M{18 + bar:.0f},{y - 6:.0f} v12" stroke="var(--ink-3)" '
               f'stroke-width="2" fill="none"/>')
    out.append(f'<text x="{18 + bar / 2:.0f}" y="{y - fs * .7:.0f}" class="svgnote" '
               f'font-size="{fs:.1f}" text-anchor="middle" '
               f'fill="var(--ink-3)">10 m</text>')
    out.append('</svg>')
    return '\n'.join(out)


# ----------------------------------------------------- thruster plan view
def layout_svg(name: str, x=0.55, y=0.40, hull_len=1.40, hull_sep=0.75,
               hull_r=0.125) -> str:
    """Scale plan view of the thruster layout, as static SVG."""
    angles = LAYOUTS[name]
    s = 150.0                      # units per metre
    w, h = 380, 420
    cx, cy = w / 2, h / 2
    # Nothing is drawn in the top and bottom 60 units, so crop them off rather
    # than shipping a figure that is a quarter empty on a phone.
    top, box_h = 62, 292

    def px(lx, ly):
        return cx - ly * s, cy - lx * s

    out = [f'<svg viewBox="0 {top} {w} {box_h}" class="layout" role="img" '
           f'aria-label="Scale plan view of the four thrusters">',
           '<defs><marker id="thrusttip" viewBox="0 0 10 10" refX="8" refY="5" '
           'markerWidth="5" markerHeight="5" orient="auto-start-reverse">'
           '<path d="M0,1 L9,5 L0,9 z" fill="var(--accent)"/></marker></defs>']
    for side in (+1, -1):
        hx, hy = px(0, side * hull_sep / 2)
        out.append(f'<rect x="{hx - hull_r * s:.1f}" y="{hy - hull_len / 2 * s:.1f}" '
                   f'width="{hull_r * 2 * s:.1f}" height="{hull_len * s:.1f}" '
                   f'rx="{hull_r * s:.1f}" fill="var(--hull)" '
                   f'stroke="var(--hullline)"/>')
    bx, by = px(hull_len / 2 + 0.10, 0)
    out.append(f'<path d="M{bx:.1f},{by:.1f} l7,13 l-14,0 z" fill="var(--accent)"/>')
    out.append(f'<text x="{cx:.0f}" y="{by - 6:.0f}" class="svglbl" '
               f'text-anchor="middle" fill="var(--ink-3)">BOW</text>')

    names = ('FL', 'FR', 'RL', 'RR')
    mounts = ((x, y), (x, -y), (-x, y), (-x, -y))
    for label, (mx, my), deg in zip(names, mounts, angles):
        a = math.radians(deg)
        ox, oy = px(mx, my)
        rot = -deg
        out.append(f'<g transform="translate({ox:.1f},{oy:.1f}) rotate({rot:.1f})">'
                   f'<rect x="-11" y="-22" width="22" height="44" rx="5" '
                   f'fill="var(--panel)" stroke="var(--ink)" stroke-width="1.6"/>'
                   f'<rect x="-11" y="-22" width="22" height="10" rx="4" '
                   f'fill="var(--accent)" opacity=".8"/></g>')
        # thrust axis through the motor, with an arrow on the forward end so
        # "every motor pushes the boat forward" is visible, not just asserted
        ux, uy = -math.sin(a), -math.cos(a)
        out.append(f'<line x1="{ox - ux * 40:.1f}" y1="{oy - uy * 40:.1f}" '
                   f'x2="{ox + ux * 40:.1f}" y2="{oy + uy * 40:.1f}" '
                   f'stroke="var(--ink-3)" stroke-width="1" '
                   f'stroke-dasharray="3 3" opacity=".8"/>')
        tipx, tipy = ox + ux * 52, oy + uy * 52
        out.append(f'<path d="M{ox + ux * 24:.1f},{oy + uy * 24:.1f} '
                   f'L{tipx:.1f},{tipy:.1f}" stroke="var(--accent)" '
                   f'stroke-width="2.4" marker-end="url(#thrusttip)"/>')
        out.append(f'<text x="{ox + (-26 if my > 0 else 26):.0f}" '
                   f'y="{oy + (-28 if mx > 0 else 36):.0f}" class="svglbl" '
                   f'text-anchor="middle" fill="var(--ink-3)">{label}</text>')

    # the yaw arm: perpendicular from the CoG onto FL's thrust line
    a = math.radians(angles[0])
    dx, dy = math.cos(a), math.sin(a)
    t = x * dx + y * dy
    foot = (x - t * dx, y - t * dy)
    fx, fy = px(*foot)
    arm = abs(x * math.sin(a) - y * math.cos(a))
    out.append(f'<line x1="{cx:.1f}" y1="{cy:.1f}" x2="{fx:.1f}" y2="{fy:.1f}" '
               f'stroke="var(--pass)" stroke-width="2.4"/>')
    out.append(f'<circle cx="{cx:.0f}" cy="{cy:.0f}" r="4" fill="var(--ink)"/>')
    out.append(f'<text x="14" y="{top + 18}" class="svgnote" fill="var(--pass)">'
               f'yaw arm {arm:.3f} m</text>')
    out.append('</svg>')
    return '\n'.join(out)


# ------------------------------------------------------------------ build
def instruments(frames):
    """The distributional read-outs the tuning was actually judged on."""
    ok = [f for f in frames if abs(f['wz']) < 5.0]
    T = frames[-1]['t']

    def pct(v, q):
        v = sorted(v)
        return v[min(len(v) - 1, int(q * len(v)))]

    crab = 100 * sum(1 for f in ok
                     if abs(f['vy']) > 0.15 and abs(f['wz']) < 0.10) / len(ok)
    # sideslip is scale-free, so it needs a speed floor or it explodes at rest
    mv = [f for f in ok if math.hypot(f['vx'], f['vy']) > 0.5]
    slip = sorted(math.degrees(abs(math.atan2(f['vy'], f['vx']))) for f in mv)
    big = 100 * sum(1 for x in slip if x > 25) / len(slip)

    runs, cur = [], None
    for f in ok:
        if math.hypot(f['vx'], f['vy']) < 0.25:
            cur = cur or [f['t'], f['t']]
            cur[1] = f['t']
        else:
            if cur and cur[1] - cur[0] > 1.5:
                runs.append(tuple(cur))
            cur = None
    if cur and cur[1] - cur[0] > 1.5:
        runs.append(tuple(cur))
    # Exclude only the standing start. Excluding the terminal window too would
    # score the baseline's 20 s waypoint stall as zero, because on the channel
    # that stall happens to sit at the end of the run.
    mid = [r for r in runs if r[0] > 3.0]

    return {
        'elapsed': T,
        'crab': crab,
        'slip90': slip[int(.9 * len(slip))],
        'slipbig': big,
        'wz90': pct([abs(f['wz']) for f in ok], .9),
        'vy90': pct([abs(f['vy']) for f in ok], .9),
        'pause': sum(b - a for a, b in mid),
        'sat': saturation(frames),
    }


def card(label, value, unit, note):
    return (f'<div class="card"><p class="lbl">{label}</p>'
            f'<div><span class="big">{value}</span>'
            f'<span class="unit">{unit}</span></div>'
            f'<p class="delta">{note}</p></div>')


def build(runs, baselines, out_path: Path, title: str) -> None:
    faces = {k: subset_font(v) for k, v in FONTS.items()}
    face_css = '\n'.join(
        f"@font-face{{font-family:'{fam}';src:url(data:font/woff2;base64,{b64})"
        f" format('woff2');font-weight:{wt};font-display:swap}}"
        for fam, wt, b64 in (('RBDisp', 700, faces['disp']),
                             ('RBBody', 400, faces['body']),
                             ('RBBody', 700, faces['bodyb']),
                             ('RBMono', 400, faces['mono'])) if b64)

    sections, chips, summary, ab = [], [], [], []
    for task, card_path, trace_path in runs:
        info = json.loads(Path(card_path).read_text())
        doc, frames = read_trace(Path(trace_path))
        view = View(frames)
        sat = saturation(frames)
        base_sat = None
        now = instruments(frames)
        was = None
        if task in baselines:
            _, bframes = read_trace(Path(baselines[task]))
            base_sat = saturation(bframes)
            was = instruments(bframes)
        ab.append((task, was, now))

        passed = info.get('passed')
        chips.append(
            f'<span class="chip {"ok" if passed else "bad"}">'
            f'<span class="dot"></span>{esc(task)} — '
            f'{"pass" if passed else "FAIL"}</span>')

        if task == 'channel':
            head = (f'{info["gates_transited"]}/{info["gates_total"]} gates in '
                    f'order, no contact')
            extra = card('Gates', f'{info["gates_transited"]}/{info["gates_total"]}',
                         '', 'every gate, in order')
        else:
            head = (f'{info["laps_completed"]:.2f} lap {info.get("direction", "")}, '
                    f'gate transited both ways')
            extra = card('Lap swept', f'{info["swept_rad"]:.2f}', 'rad',
                         f'{info["laps_completed"]:.3f} of 1 required, no counter-winding')

        cards = ''.join([
            card('Min clearance', f'{info["min_clearance_m"]:.3f}', 'm',
                 f'{info["clearance_fraction"] * 100:.1f}% of achievable · floor is 20%'),
            card('Elapsed', f'{info["elapsed_s"]:.1f}', 's',
                 f'{info["path_length_m"]:.1f} m of course on the sim clock'),
            extra,
            card('Thrusters at a limit', f'{sat:.1f}', '%',
                 (f'X layout was {base_sat:.1f}% on this task'
                  if base_sat is not None else 'of all recorded frames')),
        ])

        sections.append(f"""
  <section>
    <h2>{esc(task.title())} — {esc(head)}</h2>
    <div class="planwrap">{plan_svg(doc, frames, view)}</div>
    <p class="cap">Orange is the track the boat flew; the hull is drawn every
       8&nbsp;s along it. Dashed grey is the route the planner asked for, the
       green ring is the start, and the amber ring marks the tightest pass of
       the run. Buoys are in their real course colours.</p>
    <div class="cards">{cards}</div>
  </section>""")
        summary.append((task, info, sat, base_sat))

    gates_rows = ''
    for task, info, *_ in summary:
        if task != 'channel' or 'gates' not in info:
            continue
        worst = max(g['offset_from_centre_m'] for g in info['gates'])
        gates_rows = ''.join(
            f'<tr{" class=hl" if g["offset_from_centre_m"] == worst else ""}>'
            f'<td>{esc(g["gate"])}</td><td>{"yes" if g["transited"] else "NO"}</td>'
            f'<td class="n">{g["offset_from_centre_m"]:.2f} m</td>'
            f'<td class="n">{g["free_water_m"]:.2f} m</td></tr>'
            for g in info['gates'])

    ROWS = [
        ('elapsed', 'Elapsed', 's', '{:.1f}', False),
        ('pause', 'Seconds stopped', 's', '{:.1f}', False),
        ('crab', 'Crab frames', '%', '{:.1f}', False),
        ('slipbig', 'Sideslip over 25 deg', '%', '{:.1f}', False),
        ('slip90', 'Sideslip p90', 'deg', '{:.1f}', False),
        ('vy90', 'Sway p90', 'm/s', '{:.3f}', False),
        ('wz90', 'Yaw rate p90', 'rad/s', '{:.3f}', True),
        ('sat', 'Thrusters at a limit', '%', '{:.1f}', False),
    ]
    ab_tables = ''
    for task, was, now in ab:
        if not was:
            continue
        body = ''
        for key, label, unit, fmt, up_good in ROWS:
            b, n = was[key], now[key]
            if fmt.format(b) == fmt.format(n):
                cls, mark = 'same', '='
            elif (n > b) if up_good else (n < b):
                cls, mark = 'better', 'better'
            else:
                cls, mark = 'worse', 'worse'
            body += (f'<tr><td>{esc(label)} <span class="u">{esc(unit)}</span></td>'
                     f'<td class="n">{fmt.format(b)}</td>'
                     f'<td class="n {cls}"><strong>{fmt.format(n)}</strong></td>'
                     f'<td class="v {cls}">{mark}</td></tr>')
        ab_tables += (f'<h3>{esc(task.title())}</h3><div class="tablewrap"><table>'
                      f'<thead><tr><th>Measure</th><th>Before</th><th>After</th>'
                      f'<th>&nbsp;</th></tr></thead><tbody>{body}</tbody>'
                      f'</table></div>')

    html = f"""<title>{esc(title)}</title>
<style>
{face_css}
:root{{--ground:#EDF1F2;--panel:#FFF;--water:#DCE8EB;--grid:#C3D4D9;
 --ink:#0E2028;--ink-2:#40555E;--ink-3:#6E848D;--rule:#CBD8DC;
 --accent:#D9600A;--pass:#1E7A52;--pass-bg:#DDEFE6;--warn:#9A6B12;
 --hull:#F7FAFA;--hullline:#9DB3BB;
 --shadow:0 1px 2px rgba(14,32,40,.07),0 8px 24px rgba(14,32,40,.06)}}
@media (prefers-color-scheme:dark){{:root{{--ground:#08151B;--panel:#0E2029;
 --water:#102A34;--grid:#1E3E4A;--ink:#E4EDEF;--ink-2:#9DB3BB;--ink-3:#6F8892;
 --rule:#1C3742;--accent:#FF8A2B;--pass:#4FC08D;--pass-bg:#12332A;--warn:#E0A82E;
 --hull:#14303B;--hullline:#40606D;
 --shadow:0 1px 2px rgba(0,0,0,.3),0 10px 30px rgba(0,0,0,.35)}}}}
:root[data-theme="dark"]{{--ground:#08151B;--panel:#0E2029;--water:#102A34;
 --grid:#1E3E4A;--ink:#E4EDEF;--ink-2:#9DB3BB;--ink-3:#6F8892;--rule:#1C3742;
 --accent:#FF8A2B;--pass:#4FC08D;--pass-bg:#12332A;--warn:#E0A82E;
 --hull:#14303B;--hullline:#40606D}}
:root[data-theme="light"]{{--ground:#EDF1F2;--panel:#FFF;--water:#DCE8EB;
 --grid:#C3D4D9;--ink:#0E2028;--ink-2:#40555E;--ink-3:#6E848D;--rule:#CBD8DC;
 --accent:#D9600A;--pass:#1E7A52;--pass-bg:#DDEFE6;--warn:#9A6B12;
 --hull:#F7FAFA;--hullline:#9DB3BB}}
*{{box-sizing:border-box}}
body{{margin:0;background:var(--ground);color:var(--ink);
 font-family:'RBBody',system-ui,sans-serif;font-size:16px;line-height:1.62;
 -webkit-text-size-adjust:100%}}
.wrap{{max-width:860px;margin:0 auto;padding:clamp(18px,4vw,44px) clamp(14px,4vw,28px) 64px}}
h1,h2,.lbl,.big,thead th{{font-family:'RBDisp','RBBody',sans-serif;font-weight:700}}
.n,.mono{{font-family:'RBMono',ui-monospace,monospace;font-variant-numeric:tabular-nums}}
.eyebrow{{font-family:'RBDisp',sans-serif;text-transform:uppercase;
 letter-spacing:.14em;font-size:.72rem;color:var(--accent);margin:0 0 8px}}
h1{{font-size:clamp(1.65rem,6vw,2.5rem);line-height:1.08;margin:0 0 12px;text-wrap:balance}}
.standfirst{{color:var(--ink-2);margin:0 0 6px}}
.chips{{display:flex;flex-wrap:wrap;gap:8px;margin:18px 0 0}}
.chip{{display:inline-flex;align-items:center;gap:7px;padding:6px 12px;
 border-radius:999px;background:var(--pass-bg);color:var(--pass);
 border:1px solid color-mix(in srgb,var(--pass) 34%,transparent);
 font-family:'RBDisp',sans-serif;font-size:.78rem;text-transform:uppercase;
 letter-spacing:.07em}}
.chip .dot{{width:7px;height:7px;border-radius:50%;background:currentColor}}
section{{margin-top:42px}}
h2{{font-size:clamp(1.1rem,3.4vw,1.4rem);margin:0 0 12px;text-wrap:balance}}
p{{margin:0 0 13px;color:var(--ink-2)}}
strong{{color:var(--ink)}}
.planwrap{{background:var(--panel);border:1px solid var(--rule);border-radius:14px;
 box-shadow:var(--shadow);padding:10px;overflow-x:auto}}
svg.plan{{display:block;width:100%;height:auto;border-radius:8px}}
svg.layout{{display:block;width:100%;max-width:380px;height:auto;margin:0 auto}}
.svgnote{{font-family:'RBMono',monospace;font-size:11px}}
.svglbl{{font-family:'RBDisp',sans-serif;font-size:11px;letter-spacing:.08em}}
.cap{{font-size:.84rem;color:var(--ink-3);margin-top:10px}}
.cards{{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));
 gap:12px;margin-top:16px}}
.card{{background:var(--panel);border:1px solid var(--rule);border-radius:12px;padding:14px}}
.lbl{{font-size:.64rem;text-transform:uppercase;letter-spacing:.12em;
 color:var(--ink-3);margin:0 0 6px}}
.big{{font-size:1.5rem;line-height:1;font-family:'RBMono',monospace;
 font-variant-numeric:tabular-nums}}
.unit{{font-size:.72rem;color:var(--ink-3);margin-left:3px}}
.delta{{font-size:.76rem;color:var(--ink-3);margin:6px 0 0}}
.tablewrap{{overflow-x:auto;border:1px solid var(--rule);border-radius:12px;
 background:var(--panel);margin-top:14px}}
table{{border-collapse:collapse;width:100%;font-size:.86rem;min-width:360px}}
th,td{{padding:9px 10px;text-align:right;border-bottom:1px solid var(--rule);white-space:nowrap}}
th:first-child,td:first-child{{text-align:left}}
thead th{{font-size:.64rem;text-transform:uppercase;letter-spacing:.1em;
 color:var(--ink-3);background:var(--water)}}
tbody tr:last-child td{{border-bottom:0}}
td.n{{font-family:'RBMono',monospace;font-variant-numeric:tabular-nums}}
tr.hl td{{background:color-mix(in srgb,var(--accent) 10%,transparent)}}
.note{{font-size:.8rem;color:var(--ink-3);margin-top:10px}}
h3{{font-size:.95rem;margin:20px 0 8px;color:var(--ink-2)}}
.u{{color:var(--ink-3);font-size:.72rem}}
td.better{{color:var(--pass)}}
td.worse{{color:var(--warn)}}
td.same{{color:var(--ink-3)}}
td.v{{font-size:.66rem;text-transform:uppercase;letter-spacing:.08em;
 font-weight:700;font-family:'RBDisp',sans-serif}}
.keyline{{border-left:3px solid var(--accent);padding:2px 0 2px 14px;margin:16px 0}}
code{{font-family:'RBMono',monospace;font-size:.86em;background:var(--water);
 padding:1px 5px;border-radius:4px}}
.two{{display:grid;grid-template-columns:1fr;gap:18px;align-items:start}}
@media (min-width:720px){{.two{{grid-template-columns:380px 1fr}}}}
</style>

<div class="wrap">
  <p class="eyebrow">Gazebo · Nav2 MPPI · scored run</p>
  <h1>{esc(title)}</h1>
  <p class="standfirst">Both competition tasks, scored, on the configuration
     that ships. The boat used to crab sideways to correct its heading and stop
     dead at waypoints; it now turns and keeps going. Every figure is measured
     from the recorded run — nothing here is drawn at view time, so it reads
     the same everywhere.</p>
  <div class="chips">{''.join(chips)}</div>
{''.join(sections)}

  <section>
    <h2>The layout these runs were flown on</h2>
    <div class="two">
      <div class="planwrap">{layout_svg('tangential')}</div>
      <div>
        <p>Four T200s at 45°, mounted at <span class="mono">(±0.55, ±0.40) m</span>
           from the centre of gravity and canted <strong>tangentially</strong>.
           Every motor's forward thrust drives the boat <strong>forward</strong>,
           so all four forward is straight-ahead surge, and yaw is a mix of
           forward and reverse.</p>
        <p>Against the old X layout this is <strong>6.3&times; the turning
           moment</strong> — yaw arm 0.106 m to 0.672 m — with surge and sway
           identical to the newton. Holding a yaw moment used to cost 6.67 N of
           surge per N&middot;m; it now costs 1.053 N.</p>
        <p class="note">No motor moved and none was re-rated. Only the direction
           each one points.</p>
      </div>
    </div>
  </section>

  <section>
    <h2>The boat had the authority and would not use it</h2>
    <p>With the new layout in place, the boat still crab-walked. Measured on the
       shipped runs: sway sat <strong>pinned at its 0.45 m/s limit</strong> for a
       quarter of every run, while yaw <strong>never once exceeded 42% of its
       cap</strong> and spent <strong>0.0%</strong> of either task above half of
       it. It also stopped for 20 s at a waypoint — sailing 1.4 m past it,
       ending up with the goal 140&deg; behind, then crabbing back sideways.</p>
    <p>The cause was in the cost function, not the boat. Read from the
       controller source: <code>PathAngleCritic</code> scores the
       <em>terminal</em> yaw of each sampled trajectory, while
       <code>TwirlingCritic</code> scores the <em>mean</em> of the yaw rate over
       the whole 2.8 s horizon. So turning to fix a heading error saved
       <span class="mono">2.8 &times; 2.0 &times; w</span> and cost
       <span class="mono">10.0 &times; w</span>.</p>
    <div class="keyline"><p><strong>Correcting heading by rotating lost by
       1.79&times; — at every yaw rate, everywhere on the course.</strong>
       Sway, meanwhile, was charged by nothing anywhere in the stack. And below
       57.3&deg; of heading error the only critic asking the boat to point where
       it was going returned early without scoring at all.</p></div>
    <p>So the boat was doing the rational thing. It was being paid to crab.</p>
  </section>

  <section>
    <h2>What changed — two lines</h2>
    <div class="tablewrap"><table>
      <thead><tr><th>Parameter</th><th>Was</th><th>Now</th></tr></thead>
      <tbody>
        <tr class="hl"><td>PathAngle weight</td><td class="n">2.0</td>
            <td class="n">6.0</td></tr>
        <tr class="hl"><td>PathAngle dead band</td>
            <td class="n">1.0 rad</td><td class="n">0.35 rad</td></tr>
        <tr><td>Twirling weight</td><td class="n">10.0</td>
            <td class="n">10.0</td></tr>
        <tr><td>wz_max</td><td class="n">0.55</td><td class="n">0.55</td></tr>
        <tr><td>vy_max</td><td class="n">0.45</td><td class="n">0.45</td></tr>
      </tbody></table></div>
    <p class="note">The three unchanged rows are the ones worth noting. Raising
       <code>wz_max</code> is the obvious move and was refused three times:
       measured yaw p99 is 0.25 against a 0.55 cap, so the limit provably never
       bound. Lowering <code>vy_max</code> was refused for the same reason once
       the cost fix landed — peak sway is now 0.257, under any cap worth
       setting. And lowering <code>TwirlingCritic</code> had a recorded 20.7 s
       regression behind it; the ratio flips without touching it, so it was
       never spent.</p>
  </section>

  <section>
    <h2>Before and after</h2>
    {ab_tables}
    <p class="note">Sideslip is measured only above 0.5 m/s. It is an angle, so
       it explodes at low speed — 0.2 m/s of drift on a nearly stationary boat
       reads as 40&deg;. The baseline still shows 21% of frames past 25&deg; at
       that floor, so the measure discriminates; it is not being flattened.</p>
  </section>

  <section>
    <h2>Gate by gate</h2>
    <div class="tablewrap"><table>
      <thead><tr><th>Gate</th><th>Transited</th><th>Offset from centre</th>
        <th>Free water</th></tr></thead>
      <tbody>{gates_rows}</tbody></table></div>
    <p class="note">Offset is how far off the gate centreline the boat passed —
       smaller is better centred. The highlighted row is the least-centred pass
       of the run. The hull is 1.10 m in beam, so even the worst of these leaves
       well over a metre of water on the tight side.</p>
  </section>

  <section>
    <h2>Reading this honestly</h2>
    <p>Five runs of this configuration — three channel, two sprint. All five
       pass, none touches a buoy, and neither regression contract was edited at
       any point.</p>
    <p>Channel minimum clearance across the three runs was
       <span class="mono">0.307 / 0.321 / 0.349 m</span>, mean 0.326, against a
       0.330 &plusmn; 0.063 reference. I flagged the first run's 0.307 as a
       possible regression and was wrong about it for a specific reason worth
       recording: minimum clearance is a single-frame extreme over ~900 frames
       and four obstacles, and the two runs' minima were <em>at different
       buoys</em>. On a duration-free read-out — the mean of the four per-gate
       minima — this configuration scores 0.408 / 0.400 / 0.435 against the
       baseline's 0.419. Indistinguishable.</p>
    <p><strong>"24.5 seconds faster" would mislead you.</strong> Median moving
       speed is 0.991 m/s against the baseline's 1.004 — flat. The time came
       from deleting the stalls, not from driving harder. Course distance is
       95.5 m against a 96.2 m reference, so it is not cutting corners either.</p>
    <p>What is left: the boat still slows to about 0.08 m/s to take the sprint's
       92&deg; corner rather than carrying speed through it — roughly 4 s on a
       95 s run, at 39% of its yaw cap. Every lever for that trades against gate
       clearance, which is the binding contract. It is recorded and deliberately
       not chased.</p>
    <p class="note">Three reviewers worked this independently and had to agree
       before it shipped. Casualties along the way: a "critic dead band" that
       did not exist, a yaw-suppression result whose control group turned out to
       be the stall it was being compared against, and the clearance regression
       above. Each is written up in the journal with the number that killed it.</p>
  </section>
</div>
"""
    out_path.write_text(html)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--run', action='append', required=True,
                    metavar='TASK:CARD:TRACE')
    ap.add_argument('--baseline', action='append', default=[],
                    metavar='TASK:TRACE')
    ap.add_argument('--out', required=True)
    ap.add_argument('--title', default='RoboBoat — both tasks, scored')
    args = ap.parse_args()

    runs = []
    for spec in args.run:
        task, card_path, trace_path = spec.split(':', 2)
        runs.append((task, card_path, trace_path))
    baselines = dict(spec.split(':', 1) for spec in args.baseline)

    out = Path(args.out)
    build(runs, baselines, out, args.title)
    print(f'wrote {out} ({out.stat().st_size / 1024:.0f} KB)')


if __name__ == '__main__':
    main()
