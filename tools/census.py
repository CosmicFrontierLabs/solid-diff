"""Structural census of a directory of SLDPRT files.

For each file: container format, embedded transmit parse, FACE surface types
and EDGE curve types (flagging ones our evaluators don't cover), plus node
types ps-parser chokes on. Writes JSONL as it goes so a partial run is usable.

Usage: python -m tools.census DIR OUT.jsonl [--tessellate N]
"""

from __future__ import annotations

import json
import sys
import time
import traceback
from collections import Counter
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from solid_diff.brep2mesh import graphs_from_sldprt  # noqa: E402
from solid_diff.container import parse_file, is_modern_swx, OLE2_MAGIC  # noqa: E402
from solid_diff.geom import make_curve, make_surface  # noqa: E402


def census_file(path: str) -> dict:
    rec = {"file": Path(path).name, "status": "ok"}
    data = open(path, "rb").read(8)
    if data.startswith(OLE2_MAGIC):
        rec["status"] = "ole2-legacy"
        return rec
    try:
        graphs = graphs_from_sldprt(path)
    except Exception as e:
        rec["status"] = f"container-error: {e}"
        return rec
    if not graphs:
        rec["status"] = "no-geometry-transmit"
        # distinguish: streams present but nothing parseable?
        try:
            swx = parse_file(path)
            rec["streams"] = len(swx.streams)
        except Exception:
            pass
        return rec
    name, g = graphs[0]
    rec["stream"] = name
    rec["nodes"] = len(g.nodes)
    surfs, curves = Counter(), Counter()
    unsup_s, unsup_c = Counter(), Counter()
    for f in g.by_type("FACE"):
        s = g.deref(f.get("surface"))
        kind = s["node_name"] if s else "NONE"
        surfs[kind] += 1
        if s is not None and make_surface(g, s) is None:
            unsup_s[kind] += 1
    for e in g.by_type("EDGE"):
        c = g.deref(e.get("curve"))
        kind = c["node_name"] if c else "NONE"
        curves[kind] += 1
        if c is not None and make_curve(g, c) is None:
            unsup_c[kind] += 1
    rec["faces"] = sum(surfs.values())
    rec["surfaces"] = dict(surfs)
    rec["curves"] = dict(curves)
    if unsup_s:
        rec["unsupported_surfaces"] = dict(unsup_s)
    if unsup_c:
        rec["unsupported_curves"] = dict(unsup_c)
    rec["bodies"] = len(g.by_type("BODY"))
    rec["extra_graphs"] = len(graphs) - 1
    return rec


def _child(path: str, mem_gb: int, conn):
    """Census one file under a hard address-space cap, in a forked child.

    Some corpus files drive the parser into runaway allocation (see the
    OOM-killed sweep on 2026-07-25), so each file is isolated: a blown cap
    kills only its own child.
    """
    import resource

    limit = mem_gb * 1024**3
    resource.setrlimit(resource.RLIMIT_AS, (limit, limit))
    try:
        conn.send(census_file(path))
    except MemoryError:
        conn.send({"file": Path(path).name, "status": f"oom>{mem_gb}GB"})
    except Exception as e:
        conn.send({"file": Path(path).name, "status": f"crash: {str(e)[:120]}"})
    finally:
        conn.close()


def census_isolated(path: str, mem_gb: int = 6, timeout: int = 180) -> dict:
    import multiprocessing as mp

    parent, child = mp.Pipe(duplex=False)
    proc = mp.Process(target=_child, args=(path, mem_gb, child), daemon=True)
    proc.start()
    child.close()
    rec = None
    if parent.poll(timeout):
        try:
            rec = parent.recv()
        except EOFError:
            rec = None
    proc.join(5)
    if proc.is_alive():
        proc.terminate()
        proc.join(5)
    if rec is None:
        status = f"timeout>{timeout}s" if proc.exitcode is None else f"killed:{proc.exitcode}"
        rec = {"file": Path(path).name, "status": status}
    return rec


def main():
    src = Path(sys.argv[1])
    done = set()
    if Path(sys.argv[2]).exists():  # resume: skip files already recorded
        for line in open(sys.argv[2]):
            try:
                done.add(json.loads(line)["file"])
            except Exception:
                pass
    out = open(sys.argv[2], "a")
    files = [f for f in sorted(src.glob("*.SLDPRT")) + sorted(src.glob("*.sldprt"))
             if f.name not in done]
    t0 = time.time()
    for i, p in enumerate(files):
        try:
            rec = census_isolated(str(p))
        except Exception as e:
            rec = {"file": p.name, "status": f"crash: {e}",
                   "trace": traceback.format_exc()[-500:]}
        out.write(json.dumps(rec) + "\n")
        out.flush()
        if i % 50 == 0:
            print(f"{i}/{len(files)} ({time.time()-t0:.0f}s)", flush=True)
    print(f"done: {len(files)} files in {time.time()-t0:.0f}s")


if __name__ == "__main__":
    main()
