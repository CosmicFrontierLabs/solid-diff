"""Extract embedded Parasolid transmit files (.x_b) from SolidWorks 2015+ files.

The B-rep geometry in a modern SLDPRT lives in per-configuration container
streams (`Contents/Config-N-Partition`, plus `-GhostPartition` and sometimes
`ResolvedFeatures`). Each Partition stream is a small header followed by
back-to-back zlib blocks; each block decompresses to a Parasolid binary
transmit ('PS' magic, big-endian length-prefixed banner). See
docs/PARASOLID.md for the layout.

Usage: python -m solid_diff.extract part.SLDPRT [-o OUTDIR] [--all-streams]
"""

from __future__ import annotations

import argparse
import re
import zlib
from pathlib import Path

from .container import parse_file

BANNER_RE = re.compile(
    rb"TRANSMIT FILE (?:\((?P<kind>\w+)\) )?created by modeller version (?P<ver>\d+)"
)
GEOMETRY_STREAM_RE = re.compile(
    r"Contents/Config-\d+-(Partition|GhostPartition|ResolvedFeatures)$"
)
ZLIB_HEADERS = (b"\x78\x01", b"\x78\x9c", b"\x78\xda")


def carve_zlib(data: bytes, min_size: int = 64) -> list[tuple[int, bytes]]:
    """Decompress every zlib block found in data, in offset order."""
    out = []
    pos = 0
    while pos < len(data) - 2:
        if data[pos : pos + 2] in ZLIB_HEADERS:
            try:
                d = zlib.decompressobj()
                blob = d.decompress(data[pos:])
                if len(blob) >= min_size:
                    out.append((pos, blob))
                    pos = len(data) - len(d.unused_data)
                    continue
            except zlib.error:
                pass
        pos += 1
    return out


def describe_transmit(blob: bytes) -> tuple[str, str] | None:
    """Return (kind, modeller_version) if blob is a Parasolid transmit."""
    if not blob.startswith(b"PS"):
        return None
    m = BANNER_RE.search(blob[:256])
    if not m:
        return None
    kind = (m.group("kind") or b"part").decode()
    return kind, m.group("ver").decode()


def extract(path: str, outdir: str, all_streams: bool = False) -> int:
    swx = parse_file(path)
    out = Path(outdir)
    out.mkdir(parents=True, exist_ok=True)
    stem = Path(path).stem

    n = 0
    for name, data in swx.streams.items():
        if not all_streams and not GEOMETRY_STREAM_RE.match(name):
            continue
        for offset, blob in carve_zlib(data):
            info = describe_transmit(blob)
            if info is None:
                continue
            kind, ver = info
            safe = name.replace("/", ".")
            dest = out / f"{stem}.{safe}.{kind}.x_b"
            dest.write_bytes(blob)
            print(f"{dest}  ({len(blob)} bytes, {kind} transmit, "
                  f"modeller {ver}, from {name}@{offset})")
            n += 1
    if n == 0:
        print(f"{path}: no embedded Parasolid transmits found")
    return n


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("-o", "--outdir", default=".")
    ap.add_argument("--all-streams", action="store_true",
                    help="carve every stream, not just known geometry streams")
    args = ap.parse_args()
    total = 0
    for f in args.files:
        total += extract(f, args.outdir, all_streams=args.all_streams)
    raise SystemExit(0 if total else 1)


if __name__ == "__main__":
    main()
