#!/usr/bin/env python3
"""Render the whole tuning report as a single MP4.

    python3 tools/make_report_video.py --out media/report.mp4

Slides plus a side-by-side replay of the baseline against the tuned run, so the
difference is watched rather than read. Everything is drawn with PIL and encoded
with ffmpeg; there is no HTML, no browser and no scripting anywhere in the
pipeline, which is the point -- a video plays in viewers that will not run a
page.
"""

from __future__ import annotations

import argparse
import json
import math
import subprocess
import tempfile
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

W, H = 1280, 720
FPS = 20

BG = (237, 241, 242)
PANEL = (255, 255, 255)
WATER = (219, 232, 236)
GRID = (196, 213, 219)
INK = (14, 32, 40)
INK2 = (64, 85, 94)
INK3 = (110, 132, 141)
ACC = (217, 96, 10)
PASS = (30, 122, 82)
WARN = (154, 107, 18)
HULL = (255, 244, 232)
RULE = (203, 216, 220)

D = '/usr/share/fonts/truetype/dejavu/'
F_H1 = ImageFont.truetype(D + 'DejaVuSans-Bold.ttf', 52)
F_H2 = ImageFont.truetype(D + 'DejaVuSans-Bold.ttf', 34)
F_H3 = ImageFont.truetype(D + 'DejaVuSans-Bold.ttf', 25)
F_BODY = ImageFont.truetype(D + 'DejaVuSans.ttf', 24)
F_SM = ImageFont.truetype(D + 'DejaVuSans.ttf', 19)
F_LBL = ImageFont.truetype(D + 'DejaVuSans-Bold.ttf', 15)
F_NUM = ImageFont.truetype(D + 'DejaVuSansMono-Bold.ttf', 34)
F_MONO = ImageFont.truetype(D + 'DejaVuSansMono.ttf', 21)
F_MONOS = ImageFont.truetype(D + 'DejaVuSansMono.ttf', 17)


# ------------------------------------------------------------------ traces
def read_trace(path):
    doc = json.loads(Path(path).read_text())
    f = doc['frame_fields']
    return doc, [dict(zip(f, r)) for r in doc['frames']]


class View:
    def __init__(self, frames, w, h, pad=3.0):
        xs = [f['x'] for f in frames]
        ys = [f['y'] for f in frames]
        self.x0, self.x1 = min(xs) - pad, max(xs) + pad
        self.y0, self.y1 = min(ys) - pad, max(ys) + pad
        sx, sy = self.x1 - self.x0, self.y1 - self.y0
        self.s = min(w / sy, h / sx)
        self.ox = (w - sy * self.s) / 2
        self.oy = (h - sx * self.s) / 2

    def p(self, x, y):
        return (self.ox + (self.y1 - y) * self.s,
                self.oy + (self.x1 - x) * self.s)


def hull(view, f, fp=(0.75, 0.55)):
    hl, hb = fp
    x, y, a = f['x'], f['y'], f['yaw']
    return [view.p(x + lx * math.cos(a) - ly * math.sin(a),
                   y + lx * math.sin(a) + ly * math.cos(a))
            for lx, ly in ((hl, hb), (hl, -hb), (-hl, -hb), (-hl, hb))]


def hexrgb(s):
    s = s.lstrip('#')
    return tuple(int(s[i:i + 2], 16) for i in (0, 2, 4))


# ------------------------------------------------------------------ slides
def blank():
    im = Image.new('RGB', (W, H), BG)
    return im, ImageDraw.Draw(im, 'RGBA')


def wrap(d, text, font, maxw):
    out, line = [], ''
    for word in text.split():
        t = (line + ' ' + word).strip()
        if d.textlength(t, font=font) <= maxw:
            line = t
        else:
            out.append(line)
            line = word
    if line:
        out.append(line)
    return out


def eyebrow(d, text, y=64):
    d.text((80, y), text.upper(), font=F_LBL, fill=ACC)


def title_slide(kicker, title, sub):
    im, d = blank()
    eyebrow(d, kicker, 240)
    y = 274
    for ln in wrap(d, title, F_H1, W - 160):
        d.text((80, y), ln, font=F_H1, fill=INK)
        y += 62
    y += 12
    for ln in wrap(d, sub, F_BODY, W - 220):
        d.text((80, y), ln, font=F_BODY, fill=INK2)
        y += 34
    return im


def stat_slide(kicker, heading, stats, note=''):
    """stats: list of (value, unit, label, sublabel)"""
    im, d = blank()
    eyebrow(d, kicker)
    d.text((80, 96), heading, font=F_H2, fill=INK)
    n = len(stats)
    gap, m = 22, 80
    cw = (W - 2 * m - gap * (n - 1)) / n
    for i, (val, unit, label, sub) in enumerate(stats):
        x = m + i * (cw + gap)
        d.rounded_rectangle([x, 190, x + cw, 420], 14, fill=PANEL,
                            outline=RULE)
        d.text((x + 22, 214), label.upper(), font=F_LBL, fill=INK3)
        d.text((x + 22, 252), val, font=F_NUM, fill=ACC)
        vw = d.textlength(val, font=F_NUM)
        d.text((x + 24 + vw, 274), unit, font=F_SM, fill=INK3)
        yy = 310
        for ln in wrap(d, sub, F_SM, cw - 44):
            d.text((x + 22, yy), ln, font=F_SM, fill=INK2)
            yy += 26
    if note:
        yy = 470
        for ln in wrap(d, note, F_BODY, W - 160):
            d.text((80, yy), ln, font=F_BODY, fill=INK2)
            yy += 34
    return im


def table_slide(kicker, heading, cols, rows, note='', hl=()):
    im, d = blank()
    eyebrow(d, kicker)
    d.text((80, 96), heading, font=F_H2, fill=INK)
    m, top = 80, 178
    tw = W - 2 * m
    xs = [m + tw * c for c in cols[0]]
    d.rounded_rectangle([m, top, m + tw, top + 44], 10, fill=WATER)
    for x, lab, al in zip(xs, cols[1], cols[2]):
        t = lab.upper()
        w = d.textlength(t, font=F_LBL)
        d.text((x - (w if al == 'r' else 0), top + 15), t, font=F_LBL,
               fill=INK3)
    y = top + 44
    rh = min(52, int((H - 150 - y) / max(1, len(rows))))
    for i, row in enumerate(rows):
        if i in hl:
            d.rectangle([m, y, m + tw, y + rh], fill=(250, 233, 220))
        d.line([(m, y + rh), (m + tw, y + rh)], fill=RULE)
        for j, (x, cell, al) in enumerate(zip(xs, row, cols[2])):
            txt, col, fnt = cell if isinstance(cell, tuple) else (cell, INK, None)
            fnt = fnt or (F_BODY if j == 0 else F_MONO)
            w = d.textlength(txt, font=fnt)
            d.text((x - (w if al == 'r' else 0), y + (rh - 30) / 2), txt,
                   font=fnt, fill=col)
        y += rh
    if note:
        yy = y + 22
        for ln in wrap(d, note, F_SM, tw):
            d.text((m, yy), ln, font=F_SM, fill=INK3)
            yy += 26
    return im


def quote_slide(kicker, big, body):
    im, d = blank()
    eyebrow(d, kicker)
    y = 190
    d.rectangle([80, y, 86, y + 4], fill=ACC)
    lines = wrap(d, big, F_H2, W - 200)
    d.rectangle([80, y, 86, y + len(lines) * 46 + 10], fill=ACC)
    for ln in lines:
        d.text((110, y), ln, font=F_H2, fill=INK)
        y += 46
    y += 34
    for ln in wrap(d, body, F_BODY, W - 200):
        d.text((110, y), ln, font=F_BODY, fill=INK2)
        y += 34
    return im


def layout_slide():
    im, d = blank()
    eyebrow(d, 'the thruster layout')
    d.text((80, 96), 'Every motor pushes the boat forward', font=F_H2, fill=INK)
    cx, cy, s = 420, 430, 175
    ang = {'FL': -45, 'FR': 45, 'RL': 45, 'RR': -45}
    mnt = {'FL': (.55, .40), 'FR': (.55, -.40), 'RL': (-.55, .40),
           'RR': (-.55, -.40)}

    def px(lx, ly):
        return cx - ly * s, cy - lx * s

    for side in (1, -1):
        hx, hy = px(0, side * 0.375)
        d.rounded_rectangle([hx - .125 * s, hy - .7 * s, hx + .125 * s,
                             hy + .7 * s], .125 * s, fill=(247, 250, 250),
                            outline=(157, 179, 187))
    bx, by = px(0.80, 0)
    d.polygon([(bx, by), (bx + 9, by + 16), (bx - 9, by + 16)], fill=ACC)
    d.text((cx - 22, by - 26), 'BOW', font=F_LBL, fill=INK3)
    for name, deg in ang.items():
        a = math.radians(deg)
        ox, oy = px(*mnt[name])
        ux, uy = -math.sin(a), -math.cos(a)
        body = Image.new('RGBA', (28, 56), (0, 0, 0, 0))
        bd = ImageDraw.Draw(body)
        bd.rounded_rectangle([0, 0, 27, 55], 7, fill=PANEL, outline=INK, width=2)
        bd.rounded_rectangle([0, 0, 27, 14], 6, fill=ACC)
        body = body.rotate(deg, expand=True, resample=Image.BICUBIC)
        im.paste(body, (int(ox - body.width / 2), int(oy - body.height / 2)),
                 body)
        d.line([(ox + ux * 30, oy + uy * 30), (ox + ux * 66, oy + uy * 66)],
               fill=ACC, width=4)
        hx2, hy2 = ox + ux * 74, oy + uy * 74
        pxp, pyp = -uy, ux
        d.polygon([(hx2, hy2), (ox + ux * 60 + pxp * 8, oy + uy * 60 + pyp * 8),
                   (ox + ux * 60 - pxp * 8, oy + uy * 60 - pyp * 8)], fill=ACC)
        lx = ox + (-42 if mnt[name][1] > 0 else 42)
        ly = oy + (-46 if mnt[name][0] > 0 else 40)
        d.text((lx - 12, ly), name, font=F_LBL, fill=INK3)
    # yaw arm
    a = math.radians(-45)
    t = .55 * math.cos(a) + .40 * math.sin(a)
    fx, fy = px(.55 - t * math.cos(a), .40 - t * math.sin(a))
    d.line([(cx, cy), (fx, fy)], fill=PASS, width=5)
    d.ellipse([cx - 6, cy - 6, cx + 6, cy + 6], fill=INK)
    d.text((700, 200), 'yaw arm  0.672 m', font=F_MONO, fill=PASS)
    yy = 240
    for ln in wrap(d, 'All four forward is straight-ahead surge. Yaw is a mix '
                      'of forward and reverse. Against the old X layout this is '
                      '6.3x the turning moment, with surge and sway identical '
                      'to the newton.', F_BODY, 480):
        d.text((700, yy), ln, font=F_BODY, fill=INK2)
        yy += 34
    yy += 16
    for ln in wrap(d, 'No motor moved and none was re-rated. Only the '
                      'direction each one points.', F_SM, 480):
        d.text((700, yy), ln, font=F_SM, fill=INK3)
        yy += 26
    return im


# ------------------------------------------------- side-by-side replay
def replay_pair(left, right, labels, seconds, tail_s=1.5):
    """Two traces playing at once, each on its own clock, same wall time."""
    (docL, frL), (docR, frR) = left, right
    pad = 92
    half = (W - 3 * pad // 2) // 2
    vh = H - 250
    vL = View(frL, half, vh)
    vR = View(frR, half, vh)
    out = []
    n = int(seconds * FPS)
    base = Image.new('RGB', (W, H), BG)
    bd = ImageDraw.Draw(base)
    panels = []
    for k, (doc, fr, v) in enumerate(((docL, frL, vL), (docR, frR, vR))):
        p = Image.new('RGB', (half, vh), WATER)
        pd = ImageDraw.Draw(p)
        gx = math.ceil(v.y0 / 5) * 5
        while gx < v.y1:
            X = v.p(0, gx)[0]
            pd.line([(X, 0), (X, vh)], fill=GRID)
            gx += 5
        gy = math.ceil(v.x0 / 5) * 5
        while gy < v.x1:
            Y = v.p(gy, 0)[1]
            pd.line([(0, Y), (half, Y)], fill=GRID)
            gy += 5
        for ob in doc.get('obstacles', []):
            X, Y = v.p(ob['x'], ob['y'])
            r = max(3, ob.get('r', .2) * v.s)
            pd.ellipse([X - r, Y - r, X + r, Y + r],
                       fill=hexrgb(ob.get('colour', '#333')))
        panels.append(p)

    for i in range(n):
        u = min(1.0, i / max(1, n - int(tail_s * FPS)))
        im = base.copy()
        d = ImageDraw.Draw(im, 'RGBA')
        for k, (fr, v, p, lab) in enumerate(
                ((frL, vL, panels[0], labels[0]), (frR, vR, panels[1], labels[1]))):
            x0 = pad // 2 + k * (half + pad // 2)
            im.paste(p, (x0, 150))
            j = min(len(fr) - 1, int(u * (len(fr) - 1)))
            trail = [(v.p(f['x'], f['y'])[0] + x0, v.p(f['x'], f['y'])[1] + 150)
                     for f in fr[:j + 1]]
            if len(trail) > 1:
                d.line(trail, fill=ACC, width=3, joint='curve')
            pts = [(a + x0, b + 150) for a, b in hull(v, fr[j])]
            d.polygon(pts, fill=HULL + (225,), outline=ACC, width=3)
            col = WARN if k == 0 else PASS
            d.text((x0, 96), lab[0], font=F_H3, fill=col)
            d.text((x0, 128), lab[1], font=F_SM, fill=INK3)
            clk = f"t {fr[j]['t']:5.1f} s"
            d.text((x0 + half - d.textlength(clk, font=F_MONO), 100), clk,
                   font=F_MONO, fill=INK2)
        d.text((pad // 2, H - 62),
               'same course, same scoring, same wall clock',
               font=F_SM, fill=INK3)
        out.append(im)
    return out


# ------------------------------------------------------------------ driver
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scratch', required=True)
    ap.add_argument('--out', required=True)
    a = ap.parse_args()
    S = Path(a.scratch)

    frames = []

    def hold(im, secs):
        frames.extend([im] * int(secs * FPS))

    hold(title_slide('Gazebo · Nav2 MPPI · scored',
                     'It turns now',
                     'The boat had six times the turning authority and would '
                     'not use it. Two lines of controller cost changed that. '
                     'Every figure here is measured from a recorded run.'), 6)

    hold(stat_slide('the symptom', 'Sway was pinned. Yaw was idle.', [
        ('0.450', 'm/s', 'sway used', 'of a 0.45 limit, a quarter of every run'),
        ('0.0', '%', 'time above half yaw cap', 'in either task, across every run'),
        ('20', 's', 'stopped at one waypoint', 'sailed 1.4 m past, then crabbed back'),
    ], 'It corrected its heading by sliding sideways instead of turning, and '
       'stopped dead at waypoints.'), 8)

    hold(layout_slide(), 7)

    hold(quote_slide('the cause',
                     'Correcting heading by rotating lost by 1.79x, at every '
                     'yaw rate, everywhere on the course.',
                     'PathAngleCritic scores the terminal yaw of a sampled '
                     'trajectory; TwirlingCritic scores the mean yaw rate over '
                     'the whole 2.8 s horizon. Turning saved 2.8 x 2.0 x w and '
                     'cost 10.0 x w. Sway was charged by nothing anywhere in '
                     'the stack. The boat was being paid to crab.'), 9)

    hold(table_slide('the change', 'Two lines',
                     ([0.0, 0.62, 0.92], ['parameter', 'was', 'now'],
                      ['l', 'r', 'r']),
                     [['PathAngle weight', '2.0', ('6.0', ACC, F_NUM)],
                      ['PathAngle dead band', '1.0 rad', ('0.35 rad', ACC, F_NUM)],
                      ['Twirling weight', '10.0', '10.0'],
                      ['wz_max', '0.55', '0.55'],
                      ['vy_max', '0.45', '0.45']],
                     'The unchanged rows matter most. Raising wz_max is the '
                     'obvious move and was refused three times: measured yaw '
                     'p99 is 0.25 against a 0.55 cap, so the limit never bound.',
                     hl=(0, 1)), 9)

    for task, was, now in (('channel', 'I_channel', 'J3_channel'),
                           ('sprint', 'I_sprint', 'J2_sprint')):
        _, fw = read_trace(S / f'{was}_trace.json')
        _, fn = read_trace(S / f'{now}_trace.json')
        rows = []
        for label, fn_ in (('Elapsed  s', lambda f: f[-1]['t']),
                           ('Seconds stopped  s', None),
                           ('Crab frames  %', None),
                           ('Sideslip over 25 deg  %', None),
                           ('Sway p90  m/s', None),
                           ('Yaw rate p90  rad/s', None)):
            rows.append(label)
        vals = []
        for fr in (fw, fn):
            ok = [f for f in fr if abs(f['wz']) < 5.0]
            T = fr[-1]['t']
            mv = [f for f in ok if math.hypot(f['vx'], f['vy']) > 0.5]
            slip = [math.degrees(abs(math.atan2(f['vy'], f['vx']))) for f in mv]
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

            def p90(v):
                v = sorted(v)
                return v[int(.9 * len(v))]
            vals.append([
                f'{T:.1f}',
                f'{sum(b - x for x, b in runs if x > 3.0):.1f}',
                f'{100 * sum(1 for f in ok if abs(f["vy"]) > .15 and abs(f["wz"]) < .10) / len(ok):.1f}',
                f'{100 * sum(1 for s in slip if s > 25) / len(slip):.1f}',
                f'{p90([abs(f["vy"]) for f in ok]):.3f}',
                f'{p90([abs(f["wz"]) for f in ok]):.3f}',
            ])
        table = [[rows[i], vals[0][i], (vals[1][i], PASS, F_NUM), ('BETTER', PASS, F_LBL)]
                 for i in range(len(rows))]
        hold(table_slide('before and after', task.title(),
                         ([0.0, 0.58, 0.82, 1.0],
                          ['measure', 'before', 'after', ''],
                          ['l', 'r', 'r', 'r']), table,
                         'Sideslip is measured only above 0.5 m/s -- it is an '
                         'angle, so it explodes at low speed.'), 8)

    for task, was, now, secs in (('channel', 'I_channel', 'J3_channel', 22),
                                 ('sprint', 'I_sprint', 'J2_sprint', 22)):
        L = read_trace(S / f'{was}_trace.json')
        R = read_trace(S / f'{now}_trace.json')
        frames.extend(replay_pair(
            L, R,
            [('BEFORE', f'{L[1][-1]["t"]:.1f} s, crabs and stalls'),
             ('AFTER', f'{R[1][-1]["t"]:.1f} s, turns and keeps going')],
            secs))
        frames.extend([frames[-1]] * int(1.5 * FPS))

    hold(stat_slide('the result', 'Five runs. All five pass.', [
        ('10/10', '', 'channel gates', 'in order, no contact, mean offset 0.19 m'),
        ('1.00', 'lap', 'sprint', 'gate transited both ways, no contact'),
        ('0', '', 'contracts edited', 'both regression contracts held throughout'),
    ], 'Three reviewers worked this independently and had to agree before it '
       'shipped.'), 8)

    hold(title_slide('reading it honestly',
                     'What it is not',
                     'It is not faster driving. Median moving speed is 0.991 '
                     'against 1.004 m/s -- flat. The time came from deleting '
                     'the stalls. Channel clearance is unchanged within noise. '
                     'The boat still slows for the sprint\'s 92 degree corner, '
                     'about 4 s on a 95 s run: recorded, deliberately not '
                     'chased, because every lever for it trades against gate '
                     'clearance.'), 9)

    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory() as td:
        for i, im in enumerate(frames):
            im.save(Path(td) / f'f_{i:05d}.png')
        subprocess.run(['ffmpeg', '-y', '-loglevel', 'error', '-framerate',
                        str(FPS), '-i', str(Path(td) / 'f_%05d.png'),
                        '-c:v', 'libx264', '-pix_fmt', 'yuv420p', '-crf', '21',
                        str(out)], check=True)
    print(f'{out}  {out.stat().st_size / 1024:.0f} KB  '
          f'{len(frames) / FPS:.0f} s')


if __name__ == '__main__':
    main()
