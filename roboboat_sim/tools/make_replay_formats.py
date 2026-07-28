#!/usr/bin/env python3
"""Render a recorded run as a replay in several independent formats.

    python3 tools/make_replay_formats.py --record run.json --stem media/channel

Writes, from the one trace:

    <stem>.gif        animated GIF, plays in essentially any viewer
    <stem>.mp4        H.264, if ffmpeg is on PATH
    <stem>_smil.svg   animated SVG (SMIL) -- vector, no JavaScript
    <stem>_css.html   CSS-only replay with play/pause and speed, no JavaScript
    <stem>.png        a single still, the whole run in one frame

The point of shipping four is that attachment viewers differ wildly in what
they will execute: some run JavaScript, some run SMIL but not JavaScript, some
run neither and only decode images. At least one of these renders anywhere.
"""

from __future__ import annotations

import argparse
import base64
import json
import math
import shutil
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

WATER = (219, 232, 236)
GRID = (196, 213, 219)
INK = (14, 32, 40)
INK3 = (110, 132, 141)
TRACK = (217, 96, 10)
PLAN = (150, 170, 178)
HULL_F = (255, 244, 232)
PASS = (30, 122, 82)

FONT_CANDIDATES = [
    '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
    '/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf',
]


def load_font(size):
    for p in FONT_CANDIDATES:
        if Path(p).exists():
            return ImageFont.truetype(p, size)
    return ImageFont.load_default()


# --------------------------------------------------------------- trace input
def read_trace(path: Path):
    doc = json.loads(path.read_text())
    fields = doc['frame_fields']
    frames = [dict(zip(fields, row)) for row in doc['frames']]
    return doc, frames


class View:
    """World metres -> pixels, with +x up the image and +y to the left."""

    def __init__(self, frames, width, pad=3.0, max_h=10_000):
        xs = [f['x'] for f in frames]
        ys = [f['y'] for f in frames]
        self.x0, self.x1 = min(xs) - pad, max(xs) + pad
        self.y0, self.y1 = min(ys) - pad, max(ys) + pad
        span_x, span_y = self.x1 - self.x0, self.y1 - self.y0
        self.s = min(width / span_y, max_h / span_x)
        self.w = int(round(span_y * self.s))
        self.h = int(round(span_x * self.s))

    def p(self, x, y):
        return ((self.y1 - y) * self.s, (self.x1 - x) * self.s)


def hull_points(view, f, footprint):
    half_len, half_beam = footprint
    x, y, a = f['x'], f['y'], f['yaw']
    out = []
    for lx, ly in ((half_len, half_beam), (half_len, -half_beam),
                   (-half_len, -half_beam), (-half_len, half_beam)):
        out.append(view.p(x + lx * math.cos(a) - ly * math.sin(a),
                          y + lx * math.sin(a) + ly * math.cos(a)))
    return out


def global_plan(doc):
    """The longest plan the planner published -- the route it meant to fly."""
    plans = doc.get('plans') or []
    return max(plans, key=lambda pl: len(pl['p']))['p'] if plans else []


def markers(view, frames, fmt):
    """Green ring at the start, amber ring at the tightest pass. Static."""
    sx, sy = view.p(frames[0]['x'], frames[0]['y'])
    close = min((f for f in frames if f['clearance'] < 50),
                key=lambda f: f['clearance'])
    cx, cy = view.p(close['x'], close['y'])
    return (f'<circle cx="{fmt(sx)}" cy="{fmt(sy)}" r="7" fill="none" '
            f'stroke="rgb{PASS}" stroke-width="2.5"/>'
            f'<circle cx="{fmt(cx)}" cy="{fmt(cy)}" r="16" fill="none" '
            f'stroke="#9A6B12" stroke-width="2" stroke-dasharray="4 4"/>')


def hex_rgb(s):
    s = s.lstrip('#')
    return tuple(int(s[i:i + 2], 16) for i in (0, 2, 4))


# ------------------------------------------------------------- raster frames
def base_layer(doc, frames, view, plan_pts):
    """Everything that does not move: water, grid, planner route, buoys."""
    im = Image.new('RGB', (view.w, view.h), WATER)
    d = ImageDraw.Draw(im)

    gx = math.ceil(view.y0 / 5) * 5
    while gx < view.y1:
        px = view.p(0, gx)[0]
        d.line([(px, 0), (px, view.h)], fill=GRID, width=1)
        gx += 5
    gy = math.ceil(view.x0 / 5) * 5
    while gy < view.x1:
        py = view.p(gy, 0)[1]
        d.line([(0, py), (view.w, py)], fill=GRID, width=1)
        gy += 5

    if len(plan_pts) > 1:
        # dashed, by drawing every other short segment
        for i in range(0, len(plan_pts) - 1, 2):
            d.line([plan_pts[i], plan_pts[i + 1]], fill=PLAN, width=2)

    for ob in doc.get('obstacles', []):
        cx, cy = view.p(ob['x'], ob['y'])
        r = max(3.0, ob.get('r', 0.2) * view.s)
        col = hex_rgb(ob.get('colour', '#333333'))
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=col)
    return im


def draw_frame(base, view, frames, i, footprint, font, small, label, total_t):
    im = base.copy()
    d = ImageDraw.Draw(im, 'RGBA')

    trail = [view.p(f['x'], f['y']) for f in frames[:i + 1]]
    if len(trail) > 1:
        d.line(trail, fill=TRACK, width=3, joint='curve')

    f = frames[i]
    pts = hull_points(view, f, footprint)
    d.polygon(pts, fill=HULL_F + (215,), outline=TRACK, width=2)
    # a bow tick so heading is unmistakable
    bx, by = view.p(f['x'] + footprint[0] * 1.5 * math.cos(f['yaw']),
                    f['y'] + footprint[0] * 1.5 * math.sin(f['yaw']))
    nx, ny = view.p(f['x'], f['y'])
    d.line([(nx, ny), (bx, by)], fill=TRACK, width=2)

    speed = math.hypot(f['vx'], f['vy'])
    lines = [
        f"{label}",
        f"t     {f['t']:6.1f} / {total_t:.1f} s",
        f"speed {speed:6.2f} m/s",
        f"yaw   {f['wz']:6.2f} rad/s",
        f"clear {min(f['clearance'], 9.99):6.2f} m",
    ]
    pad = 8
    box_h = 5 + len(lines) * 17 + 6 + 4 * 11 + 6
    d.rectangle([pad, pad, pad + 196, pad + box_h], fill=(255, 255, 255, 236))
    y = pad + 5
    for n, line in enumerate(lines):
        d.text((pad + 8, y), line, font=font if n else small, fill=INK)
        y += 17

    # four thruster bars, signed, +35 N right / -25 N left of centre
    y += 6
    for name in ('f_fl', 'f_fr', 'f_rl', 'f_rr'):
        val = f[name]
        x0 = pad + 44
        mid = x0 + 60
        d.line([(x0, y), (x0 + 120, y)], fill=(210, 218, 222), width=5)
        w = val / 35.0 * 60 if val >= 0 else val / 25.0 * 60
        col = TRACK if val >= 0 else (40, 90, 140)
        d.line([(mid, y), (mid + w, y)], fill=col, width=5)
        d.line([(mid, y - 4), (mid, y + 4)], fill=INK3, width=1)
        d.text((pad + 8, y - 6), name[2:].upper(), font=small, fill=INK3)
        y += 11

    bar = 10 * view.s
    yb = view.h - 18
    d.line([(16, yb), (16 + bar, yb)], fill=INK3, width=2)
    d.line([(16, yb - 5), (16, yb + 5)], fill=INK3, width=2)
    d.line([(16 + bar, yb - 5), (16 + bar, yb + 5)], fill=INK3, width=2)
    d.text((16 + bar / 2 - 12, yb - 20), '10 m', font=small, fill=INK3)
    return im


# ------------------------------------------------------------- animated SVG
def smil_svg(doc, frames, view, footprint, label, dur_s, step) -> str:
    keys = frames[::step]
    if keys[-1] is not frames[-1]:
        keys.append(frames[-1])
    fs = view.w / 26.0

    def fmt(v):
        return f'{v:.1f}'

    tx = ';'.join(f'{fmt(view.p(f["x"], f["y"])[0])},{fmt(view.p(f["x"], f["y"])[1])}'
                  for f in keys)
    # The hull is drawn bow-along-local-+X, and rotate() in SVG turns clockwise
    # because y is down. yaw = 0 has to point up the page, which is rotate(-90).
    rot = ';'.join(fmt(-math.degrees(f['yaw']) - 90) for f in keys)

    hl, hb = footprint
    s = view.s
    body = (f'M{fmt(hl * s)},{fmt(-hb * s)} L{fmt(hl * s)},{fmt(hb * s)} '
            f'L{fmt(-hl * s)},{fmt(hb * s)} L{fmt(-hl * s)},{fmt(-hb * s)} Z')

    track = 'M' + ' L'.join(f'{fmt(view.p(f["x"], f["y"])[0])},'
                            f'{fmt(view.p(f["x"], f["y"])[1])}' for f in frames)
    # length is only needed as an upper bound for the dash trick
    length = sum(math.dist(view.p(a['x'], a['y']), view.p(b['x'], b['y']))
                 for a, b in zip(frames, frames[1:])) + 10

    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{view.w}" '
           f'height="{view.h}" viewBox="0 0 {view.w} {view.h}">',
           f'<rect width="{view.w}" height="{view.h}" fill="rgb{WATER}"/>']

    gx = math.ceil(view.y0 / 5) * 5
    grid = []
    while gx < view.y1:
        grid.append(f'M{fmt(view.p(0, gx)[0])},0 V{view.h}')
        gx += 5
    gy = math.ceil(view.x0 / 5) * 5
    while gy < view.x1:
        grid.append(f'M0,{fmt(view.p(gy, 0)[1])} H{view.w}')
        gy += 5
    out.append(f'<path d="{" ".join(grid)}" stroke="rgb{GRID}" fill="none"/>')

    pts = global_plan(doc)
    if pts:
        out.append('<path d="M' + ' L'.join(
            f'{fmt(view.p(px, py)[0])},{fmt(view.p(px, py)[1])}'
            for px, py in pts) + f'" fill="none" stroke="rgb{PLAN}" '
            'stroke-width="2" stroke-dasharray="7 7"/>')

    for ob in doc.get('obstacles', []):
        cx, cy = view.p(ob['x'], ob['y'])
        r = max(3.0, ob.get('r', 0.2) * s)
        out.append(f'<circle cx="{fmt(cx)}" cy="{fmt(cy)}" r="{fmt(r)}" '
                   f'fill="{ob.get("colour", "#333")}"/>')

    out.append(markers(view, frames, fmt))

    # the track, revealed as the boat gets there
    out.append(f'<path d="{track}" fill="none" stroke="rgb{TRACK}" '
               f'stroke-width="3" stroke-linecap="round" '
               f'stroke-dasharray="{length:.0f}" stroke-dashoffset="{length:.0f}">'
               f'<animate attributeName="stroke-dashoffset" '
               f'values="{length:.0f};0" dur="{dur_s}s" fill="freeze" '
               f'repeatCount="indefinite"/></path>')

    # the hull: translate on the outer group, rotate on the inner one
    out.append(f'<g><animateTransform attributeName="transform" type="translate" '
               f'values="{tx}" dur="{dur_s}s" calcMode="linear" '
               f'repeatCount="indefinite"/>'
               f'<g><animateTransform attributeName="transform" type="rotate" '
               f'values="{rot}" dur="{dur_s}s" calcMode="linear" '
               f'repeatCount="indefinite"/>'
               f'<path d="{body}" fill="rgb{HULL_F}" fill-opacity=".9" '
               f'stroke="rgb{TRACK}" stroke-width="2"/>'
               f'<path d="M{fmt(hl * s)},0 L{fmt(hl * s * 1.5)},0" '
               f'stroke="rgb{TRACK}" stroke-width="2"/>'
               f'</g></g>')

    bar = 10 * s
    yb = view.h - 18
    out.append(f'<path d="M16,{yb:.0f} h{bar:.0f} M16,{yb - 5:.0f} v10 '
               f'M{16 + bar:.0f},{yb - 5:.0f} v10" stroke="rgb{INK3}" '
               f'stroke-width="2" fill="none"/>')
    out.append(f'<text x="{16 + bar / 2:.0f}" y="{yb - 10:.0f}" '
               f'font-family="monospace" font-size="{fs:.0f}" '
               f'text-anchor="middle" fill="rgb{INK3}">10 m</text>')
    out.append(f'<text x="16" y="{fs * 1.6:.0f}" font-family="monospace" '
               f'font-size="{fs:.0f}" fill="rgb{INK}">{label}</text>')
    out.append('</svg>')
    return '\n'.join(out)


# --------------------------------------------------------- CSS-only replay
CSS_PAGE = """<title>{title}</title>
<style>
:root{{--ground:#EDF1F2;--panel:#fff;--ink:#0E2028;--ink3:#6E848D;--rule:#CBD8DC;
 --accent:#D9600A;--pass:#1E7A52}}
@media (prefers-color-scheme:dark){{:root{{--ground:#08151B;--panel:#0E2029;
 --ink:#E4EDEF;--ink3:#6F8892;--rule:#1C3742;--accent:#FF8A2B;--pass:#4FC08D}}}}
*{{box-sizing:border-box}}
body{{margin:0;background:var(--ground);color:var(--ink);
 font:16px/1.6 system-ui,sans-serif}}
.wrap{{max-width:900px;margin:0 auto;padding:22px 16px 48px}}
h1{{font-size:1.4rem;margin:0 0 4px}}
p.sub{{color:var(--ink3);margin:0 0 18px;font-size:.9rem}}
.stage{{background:var(--panel);border:1px solid var(--rule);border-radius:14px;
 padding:10px;overflow:hidden}}
.stage svg{{display:block;width:100%;height:auto}}
.boat{{offset-path:path('{path}');offset-rotate:auto;
 animation:fly {dur}s linear infinite}}
@keyframes fly{{from{{offset-distance:0%}}to{{offset-distance:100%}}}}
.reveal{{stroke-dasharray:{len};stroke-dashoffset:{len};
 animation:draw {dur}s linear infinite}}
@keyframes draw{{from{{stroke-dashoffset:{len}}}to{{stroke-dashoffset:0}}}}
/* Controls are checkboxes and radios: no script anywhere on this page. */
#pause:checked ~ .stage .boat,#pause:checked ~ .stage .reveal
 {{animation-play-state:paused}}
#s2:checked ~ .stage .boat,#s2:checked ~ .stage .reveal{{animation-duration:{half}s}}
#s4:checked ~ .stage .boat,#s4:checked ~ .stage .reveal{{animation-duration:{quarter}s}}
input{{position:absolute;opacity:0;pointer-events:none}}
.bar{{display:flex;gap:8px;flex-wrap:wrap;margin:14px 0 0}}
label{{cursor:pointer;user-select:none;border:1px solid var(--rule);
 background:var(--panel);border-radius:999px;padding:7px 15px;font-size:.82rem;
 font-weight:600;letter-spacing:.02em}}
#pause:checked ~ .bar label[for=pause],#s2:checked ~ .bar label[for=s2],
#s4:checked ~ .bar label[for=s4],#s1:checked ~ .bar label[for=s1]
 {{background:var(--accent);border-color:var(--accent);color:#fff}}
.note{{color:var(--ink3);font-size:.85rem;margin-top:16px}}
table{{border-collapse:collapse;margin-top:14px;font-size:.9rem}}
td{{padding:4px 14px 4px 0}}
td.n{{font-family:ui-monospace,monospace}}
</style>
<div class="wrap">
<h1>{title}</h1>
<p class="sub">{sub}</p>
<input type="checkbox" id="pause">
<input type="radio" name="sp" id="s1" checked><input type="radio" name="sp" id="s2">
<input type="radio" name="sp" id="s4">
<div class="bar">
  <label for="pause">play / pause</label>
  <label for="s1">1x</label><label for="s2">2x</label><label for="s4">4x</label>
</div>
<div class="stage">{svg}</div>
<table><tbody>{rows}</tbody></table>
<p class="note">No JavaScript on this page. The boat is moved by a CSS motion
   path and the buttons are checkbox and radio inputs, so it animates and
   responds anywhere CSS runs.</p>
</div>
"""


def css_replay(doc, frames, view, footprint, title, sub, dur_s, rows) -> str:
    def fmt(v):
        return f'{v:.1f}'

    track = 'M' + ' L'.join(f'{fmt(view.p(f["x"], f["y"])[0])},'
                            f'{fmt(view.p(f["x"], f["y"])[1])}' for f in frames)
    length = sum(math.dist(view.p(a['x'], a['y']), view.p(b['x'], b['y']))
                 for a, b in zip(frames, frames[1:])) + 10

    hl, hb = footprint
    s = view.s
    body = (f'<path d="M{fmt(hl * s)},{fmt(-hb * s)} L{fmt(hl * s)},{fmt(hb * s)} '
            f'L{fmt(-hl * s)},{fmt(hb * s)} L{fmt(-hl * s)},{fmt(-hb * s)} Z" '
            f'fill="rgb{HULL_F}" fill-opacity=".9" stroke="rgb{TRACK}" '
            f'stroke-width="2"/>'
            f'<path d="M{fmt(hl * s)},0 L{fmt(hl * s * 1.5)},0" '
            f'stroke="rgb{TRACK}" stroke-width="2"/>')

    out = [f'<svg viewBox="0 0 {view.w} {view.h}" xmlns="http://www.w3.org/2000/svg">',
           f'<rect width="{view.w}" height="{view.h}" fill="rgb{WATER}"/>']
    grid = []
    gx = math.ceil(view.y0 / 5) * 5
    while gx < view.y1:
        grid.append(f'M{fmt(view.p(0, gx)[0])},0 V{view.h}')
        gx += 5
    gy = math.ceil(view.x0 / 5) * 5
    while gy < view.x1:
        grid.append(f'M0,{fmt(view.p(gy, 0)[1])} H{view.w}')
        gy += 5
    out.append(f'<path d="{" ".join(grid)}" stroke="rgb{GRID}" fill="none"/>')

    pts = global_plan(doc)
    if pts:
        out.append('<path d="M' + ' L'.join(
            f'{fmt(view.p(px, py)[0])},{fmt(view.p(px, py)[1])}'
            for px, py in pts) + f'" fill="none" stroke="rgb{PLAN}" '
            'stroke-width="2" stroke-dasharray="7 7"/>')

    for ob in doc.get('obstacles', []):
        cx, cy = view.p(ob['x'], ob['y'])
        r = max(3.0, ob.get('r', 0.2) * s)
        out.append(f'<circle cx="{fmt(cx)}" cy="{fmt(cy)}" r="{fmt(r)}" '
                   f'fill="{ob.get("colour", "#333")}"/>')

    out.append(markers(view, frames, fmt))
    out.append(f'<path class="reveal" d="{track}" fill="none" '
               f'stroke="rgb{TRACK}" stroke-width="3" stroke-linecap="round"/>')
    out.append(f'<g class="boat">{body}</g>')

    bar = 10 * s
    yb = view.h - 18
    out.append(f'<path d="M16,{yb:.0f} h{bar:.0f} M16,{yb - 5:.0f} v10 '
               f'M{16 + bar:.0f},{yb - 5:.0f} v10" stroke="rgb{INK3}" '
               f'stroke-width="2" fill="none"/>')
    out.append(f'<text x="{16 + bar / 2:.0f}" y="{yb - 10:.0f}" '
               f'font-family="monospace" font-size="{view.w / 26:.0f}" '
               f'text-anchor="middle" fill="rgb{INK3}">10 m</text>')
    out.append('</svg>')

    return CSS_PAGE.format(
        title=title, sub=sub, svg='\n'.join(out), path=track,
        len=f'{length:.0f}', dur=f'{dur_s:.0f}', half=f'{dur_s / 2:.0f}',
        quarter=f'{dur_s / 4:.0f}',
        rows=''.join(f'<tr><td>{k}</td><td class="n">{v}</td></tr>'
                     for k, v in rows))


# ------------------------------------------------------------------- driver
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--record', type=Path, required=True)
    ap.add_argument('--card', type=Path)
    ap.add_argument('--stem', required=True)
    ap.add_argument('--label', default='run')
    ap.add_argument('--width', type=int, default=760)
    ap.add_argument('--gif-width', type=int, default=560)
    ap.add_argument('--playback', type=float, default=30.0,
                    help='seconds of wall clock for the whole run')
    args = ap.parse_args()

    doc, frames = read_trace(args.record)
    footprint = doc.get('footprint', [0.75, 0.55])
    total_t = frames[-1]['t']
    stem = Path(args.stem)
    stem.parent.mkdir(parents=True, exist_ok=True)

    rows = []
    if args.card and args.card.exists():
        info = json.loads(args.card.read_text())
        rows = [('min clearance', f'{info["min_clearance_m"]:.3f} m'),
                ('elapsed', f'{info["elapsed_s"]:.1f} s'),
                ('path length', f'{info["path_length_m"]:.1f} m'),
                ('result', 'PASS' if info.get('passed') else 'FAIL')]
        if 'gates_transited' in info:
            rows.insert(1, ('gates', f'{info["gates_transited"]}/'
                                     f'{info["gates_total"]} in order'))
        if 'laps_completed' in info:
            rows.insert(1, ('laps', f'{info["laps_completed"]:.2f} '
                                    f'{info.get("direction", "")}'))

    # ---- vector formats
    vec = View(frames, args.width)
    (stem.with_name(stem.name + '_smil.svg')).write_text(
        smil_svg(doc, frames, vec, footprint, args.label, args.playback,
                 max(1, len(frames) // 400)))
    sub = (f'{total_t:.1f} s of run, played back over {args.playback:.0f} s. '
           f'Tightest pass {doc.get("min_clearance", 0):.3f} m.')
    (stem.with_name(stem.name + '_css.html')).write_text(
        css_replay(doc, frames, vec, footprint, args.label, sub,
                   args.playback, rows))

    # ---- raster formats
    view = View(frames, args.gif_width)
    plan_pts = [view.p(px, py) for px, py in global_plan(doc)]
    base = base_layer(doc, frames, view, plan_pts)
    font, small = load_font(12), load_font(10)

    step = max(1, round(len(frames) / (args.playback * 20)))
    idx = list(range(0, len(frames), step))
    if idx[-1] != len(frames) - 1:
        idx.append(len(frames) - 1)
    imgs = [draw_frame(base, view, frames, i, footprint, font, small,
                       args.label, total_t) for i in idx]

    imgs[-1].save(stem.with_suffix('.png'))
    hold = 1800  # linger on the finished track before looping
    durs = [int(1000 * args.playback / len(imgs))] * len(imgs)
    durs[-1] = hold
    imgs[0].save(stem.with_suffix('.gif'), save_all=True,
                 append_images=imgs[1:], duration=durs, loop=0, optimize=True)

    if shutil.which('ffmpeg'):
        with tempfile.TemporaryDirectory() as td:
            for n, im in enumerate(imgs):
                im.save(Path(td) / f'f_{n:05d}.png')
            fps = max(1, round(len(imgs) / args.playback))
            subprocess.run(
                ['ffmpeg', '-y', '-loglevel', 'error', '-framerate', str(fps),
                 '-i', str(Path(td) / 'f_%05d.png'),
                 '-vf', 'scale=trunc(iw/2)*2:trunc(ih/2)*2',
                 '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '23',
                 str(stem.with_suffix('.mp4'))], check=True)

    for p in sorted(stem.parent.glob(stem.name + '*')):
        print(f'{p}  {p.stat().st_size / 1024:.0f} KB')


if __name__ == '__main__':
    main()
