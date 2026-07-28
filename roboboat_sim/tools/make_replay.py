#!/usr/bin/env python3
"""Turn a recorded run into a single self-contained HTML replay.

    python3 tools/nav_test.py --record run.json      # while the sim is up
    python3 tools/make_replay.py --record run.json --out replay.html

The output has no external requests at all -- fonts are subset and inlined as
data URIs, the trace is embedded as JSON -- so it can be opened from a phone,
attached to a PR, or published as an artifact without a server.

Fonts are subset from whatever is installed locally. If fontTools or the font
files are missing the page still works; it falls back to system faces.
"""

from __future__ import annotations

import argparse
import base64
import json
import sys
import tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
TEMPLATE = HERE / 'replay_template.html'

# (placeholder, candidate font paths). First existing path wins.
FONTS = {
    '__FONT_COND__': [
        '/usr/share/fonts/truetype/open-sans/OpenSans-CondBold.ttf',
        '/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed-Bold.ttf',
    ],
    '__FONT_CONDREG__': [
        '/usr/share/fonts/truetype/open-sans/OpenSans-CondLight.ttf',
        '/usr/share/fonts/truetype/dejavu/DejaVuSansCondensed.ttf',
    ],
    '__FONT_MONO__': [
        '/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf',
        '/usr/share/fonts/truetype/liberation/LiberationMono-Regular.ttf',
    ],
}

# Latin-1 printable plus the handful of symbols the page actually sets.
GLYPHS = ''.join(chr(c) for c in range(32, 127)) + '·—–°±×→↑↓≈'


def subset_font(path: Path) -> str:
    """Subset a TTF to WOFF2 and return it base64-encoded."""
    from fontTools import subset
    from fontTools.ttLib import TTFont

    font = TTFont(str(path))
    options = subset.Options(flavor='woff2', desubroutinize=True,
                             layout_features=['kern', 'liga'])
    subsetter = subset.Subsetter(options=options)
    subsetter.populate(text=GLYPHS)
    subsetter.subset(font)
    with tempfile.NamedTemporaryFile(suffix='.woff2', delete=False) as handle:
        tmp = Path(handle.name)
    try:
        font.save(str(tmp))
        return base64.b64encode(tmp.read_bytes()).decode('ascii')
    finally:
        tmp.unlink(missing_ok=True)


def build(record: Path, out: Path, title: str | None = None) -> None:
    data = json.loads(record.read_text())
    html = TEMPLATE.read_text()

    # The gallery lists artifacts by <title>, so two replays that share one are
    # indistinguishable once published.
    if title:
        html = html.replace(
            '<title>RoboBoat \u2014 Nav2 MPPI run replay</title>',
            f'<title>{title}</title>')

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
                print('fontTools not installed; falling back to system fonts',
                      file=sys.stderr)
                break
            except Exception as exc:                      # noqa: BLE001
                print(f'could not subset {path.name}: {exc}', file=sys.stderr)
        html = html.replace(placeholder, encoded)

    # Guard against the trace closing the script element early.
    payload = json.dumps(data, separators=(',', ':')).replace('</', '<\\/')
    html = html.replace('__DATA__', payload)

    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(html)
    print(f'{out}  ({out.stat().st_size / 1024:.0f} KB, '
          f'{len(data["frames"])} frames, {len(data["obstacles"])} obstacles)')


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', type=Path, required=True,
                        help='JSON trace from nav_test.py --record')
    parser.add_argument('--out', type=Path, default=Path('replay.html'))
    parser.add_argument('--title', default=None,
                        help='page title; also names the published artifact')
    args = parser.parse_args()

    if not args.record.exists():
        sys.exit(f'no such recording: {args.record}')
    build(args.record, args.out, args.title)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
