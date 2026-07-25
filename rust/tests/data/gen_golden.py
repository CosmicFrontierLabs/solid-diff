# Regenerates tests/data/golden.txt from the Python reference pipeline.
# Run from the repo root:  .venv/bin/python rust/tests/data/gen_golden.py
import io, glob, os, struct, sys
sys.path.insert(0, "/home/meawoppl/repos/solid-diff")
from collections import Counter
from solid_diff.container import parse, is_modern_swx
from solid_diff.extract import carve_zlib, describe_transmit
from solid_diff.xt import _schema
from psparser.parser import read_document

ROOT = "/home/meawoppl/repos/solid-diff"


def fmt(v):
    if v is None:
        return "N"
    if isinstance(v, bool):
        return "T" if v else "F"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return "f" + struct.pack(">d", v).hex()
    if isinstance(v, str):
        return "s" + v
    if isinstance(v, tuple):
        return "(" + ",".join(fmt(x) for x in v) + ")"
    if isinstance(v, list):
        return "[" + ",".join(fmt(x) for x in v) + "]"
    raise TypeError(repr(v))


def fnv1a(data: bytes) -> int:
    h = 0xCBF29CE484222325
    for b in data:
        h ^= b
        h = (h * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def value_hash(doc) -> str:
    parts = []
    for n in doc.nodes:
        layout = doc.layouts[n["node_type"]]
        head = f"{n['id']}|{n['node_type']}|{n.get('count', '')}|"
        body = ";".join(f"{f.name}={fmt(n[f.name])}" for f in layout)
        parts.append(head + body)
    return f"{fnv1a('\n'.join(parts).encode('utf-8', 'surrogatepass')):016x}"


lines = []
paths = sorted(glob.glob(f"{ROOT}/samples/*.SLDPRT")) + sorted(glob.glob(f"{ROOT}/vault/*.SLDPRT"))
for path in paths:
    rel = os.path.relpath(path, ROOT)
    data = open(path, "rb").read()
    if not is_modern_swx(data):
        lines.append(f"{rel} NOT_MODERN")
        continue
    swx = parse(data, path)
    streams = swx.streams
    lines.append(f"{rel} streams={len(streams)}")
    for name, sdata in streams.items():
        for off, blob in carve_zlib(sdata):
            info = describe_transmit(blob)
            if info is None:
                continue
            kind = info[0]
            try:
                doc = read_document(io.BytesIO(blob), _schema())
            except Exception:
                lines.append(f"  {name}@{off} {kind} size={len(blob)} ERR")
                continue
            hist = Counter(n["node_name"] for n in doc.nodes)
            h = ",".join(f"{k}:{v}" for k, v in sorted(hist.items()))
            lines.append(
                f"  {name}@{off} {kind} size={len(blob)} nodes={len(doc.nodes)} "
                f"vals={value_hash(doc)} {h}"
            )

open(f"{ROOT}/rust/tests/data/golden.txt", "w").write("\n".join(lines) + "\n")
print(len(lines), "lines")
