#!/usr/bin/env bash
# Encode a captured frame directory into an mp4 and a poster still.
#
#   tools/make_video.sh /tmp/frames_showcase_channel media/channel.mp4 [fps]
#
# The frames come out of tools/capture_frames.py at the camera's own rate, so
# playing them back at that rate gives real-time motion. Pass a higher fps to
# speed the playback up; the default keeps it honest.
#
# H.264 in yuv420p with even dimensions, because that is what plays everywhere
# without a codec argument -- phones included, which is the whole point of
# making these.
set -euo pipefail

FRAMES="${1:?usage: make_video.sh <frames-dir> <out.mp4> [fps]}"
OUT="${2:?usage: make_video.sh <frames-dir> <out.mp4> [fps]}"
FPS="${3:-6}"

shopt -s nullglob
first=("$FRAMES"/*_0000.png)
if [[ ${#first[@]} -eq 0 ]]; then
  echo "no frames in $FRAMES" >&2; exit 1
fi
PREFIX="$(basename "${first[0]}" _0000.png)"
COUNT=$(find "$FRAMES" -name "${PREFIX}_*.png" | wc -l)
mkdir -p "$(dirname "$OUT")"

ffmpeg -y -loglevel error -framerate "$FPS" \
    -i "$FRAMES/${PREFIX}_%04d.png" \
    -vf "scale=trunc(iw/2)*2:trunc(ih/2)*2" \
    -c:v libx264 -preset slow -crf 20 -pix_fmt yuv420p \
    -movflags +faststart "$OUT"

# A poster from a third of the way in: far enough that the boat is moving,
# early enough that it is still recognisably the start of the run.
POSTER="${OUT%.*}.jpg"
MID=$(( COUNT / 3 ))
ffmpeg -y -loglevel error -i "$(printf '%s/%s_%04d.png' "$FRAMES" "$PREFIX" "$MID")" \
    -q:v 3 "$POSTER"

SIZE=$(du -h "$OUT" | cut -f1)
DUR=$(python3 -c "print(f'{$COUNT / $FPS:.1f}')")
echo "wrote $OUT  ($COUNT frames, ${DUR}s at ${FPS} fps, $SIZE)"
echo "wrote $POSTER"
