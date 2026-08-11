#!/usr/bin/env bash
# Render a directory of SLDPRT parts into a numbered series of isometric
# contact sheets, several sheets at a time.
#
#   tools/contact_sheets.sh SRC_DIR OUT_DIR [PER_SHEET] [TILE_PX] [JOBS]
#
# Parts are sorted by name so revisions of the same part land together, then
# chunked PER_SHEET at a time (default 64 = an 8x8 grid). Each chunk is one
# `solid-diff iso` process, JOBS of them running at once.
set -euo pipefail

SRC=${1:?usage: contact_sheets.sh SRC_DIR OUT_DIR [PER_SHEET] [TILE_PX] [JOBS]}
OUT=${2:?missing OUT_DIR}
PER=${3:-64}
TILE=${4:-300}
JOBS=${5:-12}

BIN=${SOLID_DIFF_BIN:-$(dirname "$0")/../rust/target/release/solid-diff}
[ -x "$BIN" ] || { echo "build first: (cd rust && cargo build --release)"; exit 1; }
COLS=$(python3 -c "import math;print(math.ceil(math.sqrt($PER)))")

mkdir -p "$OUT"
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

find "$SRC" -name '*.SLDPRT' | sort > "$work/all.txt"
total=$(wc -l < "$work/all.txt")
[ "$total" -gt 0 ] || { echo "no .SLDPRT files under $SRC"; exit 1; }
split -l "$PER" -d -a 3 "$work/all.txt" "$work/batch_"
echo "$total parts -> $(ls "$work"/batch_* | wc -l) sheets of ${COLS}x${COLS} @ ${TILE}px"

export BIN OUT COLS TILE
ls "$work"/batch_* | xargs -P "$JOBS" -I{} bash -c '
  n=$(basename {}); n=${n#batch_}
  mapfile -t files < {}
  "$BIN" iso "${files[@]}" -o "$OUT/sheet_$n.png" --size "$TILE" --cols "$COLS" --sheet \
    > "$OUT/sheet_$n.log" 2>&1 && echo "sheet_$n.png  (${#files[@]} parts)"
'
echo "wrote $(ls "$OUT"/sheet_*.png | wc -l) sheets to $OUT"
