"""Scan SolidWorks 2015+ files for embedded Parasolid B-rep data.

The SolidWorks kernel is Parasolid, and the Document Manager API proves the
B-rep is physically stored in the file. This tool decodes the chunked
container (see container.py), then hunts every decompressed stream — and the
raw file — for Parasolid transmit-format signatures:

  text transmit (.x_t):   '**ABCDEFGHIJKLMNOPQRSTUVWXYZ...' banner,
                          '**PARASOLID', 'TRANSMIT FILE'
  binary transmit (.x_b): optional '**...**END_OF_HEADER**\\n' text block,
                          then 'PS' + i32-length modeler-version string +
                          i32-length schema name ('SCH_<digits>_<digits>')
  either:                 'SCH_' schema-name strings

It also attempts nested decompression (zlib and raw-deflate carving) inside
streams, since the geometry partitions may be independently compressed, and
reports per-stream entropy so undecoded high-entropy blobs stand out.

Usage: python -m solid_diff.psscan [--brute] file.SLDPRT [...]
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import zlib
from dataclasses import dataclass

from .container import parse_file

TEXT_MARKERS = [
    (b"**ABCDEFGHIJKLMNOPQRSTUVWXYZ", "xt-banner"),
    (b"**PARASOLID", "xt-parasolid"),
    (b"**END_OF_HEADER**", "xt-end-of-header"),
    (b"TRANSMIT FILE", "xt-transmit"),
]
SCH_RE = re.compile(rb"SCH_\d{4,8}_\d{4,6}")


@dataclass
class Hit:
    stream: str
    kind: str
    offset: int
    detail: str


def shannon_entropy(data: bytes, cap: int = 1 << 20) -> float:
    if not data:
        return 0.0
    sample = data[:cap]
    counts = [0] * 256
    for b in sample:
        counts[b] += 1
    n = len(sample)
    return -sum(c / n * math.log2(c / n) for c in counts if c)


def check_ps_binary_header(data: bytes, pos: int) -> str | None:
    """Validate a 'PS' occurrence as a plausible .x_b binary header.

    Layout after 'PS': i32 len + modeler-version string, i32 len + schema
    name. Returns a description string if it parses cleanly, else None.
    """
    try:
        p = pos + 2
        vlen = int.from_bytes(data[p : p + 4], "little")
        if not 4 <= vlen <= 128:
            return None
        version = data[p + 4 : p + 4 + vlen]
        if len(version) != vlen or not all(0x20 <= b < 0x7F for b in version):
            return None
        p = p + 4 + vlen
        slen = int.from_bytes(data[p : p + 4], "little")
        if not 4 <= slen <= 64:
            return None
        schema = data[p + 4 : p + 4 + slen]
        if len(schema) != slen or not schema.startswith(b"SCH_"):
            return None
        return f"version={version.decode()!r} schema={schema.decode()!r}"
    except Exception:
        return None


def scan_blob(name: str, data: bytes) -> list[Hit]:
    hits: list[Hit] = []
    for marker, kind in TEXT_MARKERS:
        start = 0
        while (pos := data.find(marker, start)) != -1:
            context = data[pos : pos + 60].decode("latin-1")
            hits.append(Hit(name, kind, pos, context))
            start = pos + 1
    for m in SCH_RE.finditer(data):
        hits.append(Hit(name, "schema-name", m.start(), m.group().decode()))
    start = 0
    while (pos := data.find(b"PS", start)) != -1:
        detail = check_ps_binary_header(data, pos)
        if detail:
            hits.append(Hit(name, "xb-binary-header", pos, detail))
        start = pos + 1
    return hits


def nested_zlib_carve(data: bytes) -> list[tuple[int, bytes]]:
    """Find zlib-headed sub-streams (0x78 0x01/0x9C/0xDA) inside a blob."""
    found = []
    for hdr in (b"\x78\x9c", b"\x78\xda", b"\x78\x01"):
        start = 0
        while (pos := data.find(hdr, start)) != -1:
            try:
                d = zlib.decompressobj()
                out = d.decompress(data[pos:], 1 << 24)
                if len(out) >= 64:
                    found.append((pos, out))
            except zlib.error:
                pass
            start = pos + 1
    return found


def brute_deflate_carve(data: bytes, min_out: int = 256) -> list[tuple[int, bytes]]:
    """Try raw-deflate decompression at every offset. Slow; use --brute."""
    found = []
    pos = 0
    while pos < len(data) - 4:
        try:
            d = zlib.decompressobj(wbits=-15)
            out = d.decompress(data[pos:], 1 << 24)
            if len(out) >= min_out:
                found.append((pos, out))
                pos += max(1, len(data[pos:]) - len(d.unconsumed_tail) - 1)
        except zlib.error:
            pass
        pos += 1
    return found


def scan_file(path: str, brute: bool = False) -> int:
    swx = parse_file(path)
    streams = swx.streams
    print(f"\n=== {path}")
    print(f"ROL key: 0x{swx.rol_key:02x}; {len(swx.chunks)} chunks, "
          f"{len(streams)} named streams")

    all_hits: list[Hit] = []
    version = next((n for n in streams if n.startswith("_MO_VERSION_")), None)
    if version:
        print(f"version stream: {version}")

    print(f"\n{'stream':52} {'size':>9} {'entropy':>7}")
    for name, data in sorted(streams.items(), key=lambda kv: -len(kv[1])):
        ent = shannon_entropy(data)
        flag = "  <-- high entropy" if ent > 7.5 and len(data) > 4096 else ""
        print(f"{name[:52]:52} {len(data):>9} {ent:>7.2f}{flag}")
        all_hits += scan_blob(name, data)

        for off, sub in nested_zlib_carve(data):
            sub_name = f"{name}[zlib@{off}]"
            print(f"{sub_name[:52]:52} {len(sub):>9} "
                  f"{shannon_entropy(sub):>7.2f}  (nested zlib)")
            all_hits += scan_blob(sub_name, sub)
        if brute:
            for off, sub in brute_deflate_carve(data):
                sub_name = f"{name}[deflate@{off}]"
                print(f"{sub_name[:52]:52} {len(sub):>9} "
                      f"{shannon_entropy(sub):>7.2f}  (carved deflate)")
                all_hits += scan_blob(sub_name, sub)

    with open(path, "rb") as f:
        raw = f.read()
    all_hits += scan_blob("<raw file>", raw)

    if all_hits:
        print(f"\n{len(all_hits)} Parasolid signature hit(s):")
        for h in all_hits:
            print(f"  [{h.kind}] {h.stream} @ 0x{h.offset:x}: {h.detail}")
    else:
        print("\nNo Parasolid signatures found.")
    return len(all_hits)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("files", nargs="+")
    ap.add_argument("--brute", action="store_true",
                    help="raw-deflate carve at every offset of every stream")
    args = ap.parse_args()
    total = 0
    for path in args.files:
        try:
            total += scan_file(path, brute=args.brute)
        except ValueError as e:
            print(f"skip: {e}", file=sys.stderr)
    sys.exit(0 if total else 1)


if __name__ == "__main__":
    main()
