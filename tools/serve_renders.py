#!/usr/bin/env python3
"""Browsable gallery for rendered contact sheets.

Serves a directory of renders as a small website: a thumbnail grid, a
full-screen viewer with keyboard paging and zoom, and download links. Sheets
that exist at several resolutions are grouped so you can switch between them
without leaving the viewer.

    python3 tools/serve_renders.py [--root renders] [--port 8791]
    python3 tools/serve_renders.py --parts /path/to/parts   # part-by-part review

Pair with `agent-portal forward <port>` to reach it from a browser. Re-running
`forward` issues a NEW hostname and retires the old one, so re-register it only
when you mean to; a stale URL presents as a connection timeout, not a 404.

Thumbnails are generated once and cached in memory (Pillow if available), so a
page of 4K sheets loads in kilobytes rather than tens of megabytes. Range
requests are supported so large downloads resume.

With --parts, /review adds a one-part-at-a-time reviewer: prev/next buttons and
arrow keys, F to flag a part as problematic, and a report of everything flagged
with full filenames. Renders are produced on demand and cached on disk, with the
next few parts prefetched in the background, so paging stays responsive over a
corpus of thousands.
"""

from __future__ import annotations

import argparse
import html
import io
import json
import mimetypes
import os
import re
import socket
import subprocess
import sys
import threading
import time
import urllib.parse
from concurrent.futures import ThreadPoolExecutor
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

IMAGE_EXT = {".png", ".jpg", ".jpeg", ".gif", ".webp"}
THUMB_PX = 340

ROOT = Path(".")
THUMBS: dict[str, bytes] = {}

# ── part review state ───────────────────────────────────────────────────────
PARTS: list[Path] = []
PART_PX = 900
BIN = Path("rust/target/release/solid-diff")
CACHE = Path(".review-cache")
FLAGS: dict[str, dict] = {}
FLAGS_LOCK = threading.Lock()
STATS: dict[str, str] = {}
POOL: ThreadPoolExecutor | None = None
INFLIGHT: set[int] = set()
PREFETCH = 5


def human(n: float) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if n < 1024 or unit == "GB":
            return f"{n:.0f} {unit}" if unit == "B" else f"{n:.1f} {unit}"
        n /= 1024
    return f"{n:.1f} GB"


def png_size(path: Path) -> tuple[int, int] | None:
    """Read a PNG's dimensions from its header, without decoding it."""
    try:
        with open(path, "rb") as f:
            head = f.read(24)
        if head[:8] != b"\x89PNG\r\n\x1a\n":
            return None
        return int.from_bytes(head[16:20], "big"), int.from_bytes(head[20:24], "big")
    except OSError:
        return None


def thumbnail(path: Path) -> bytes | None:
    key = str(path)
    if key in THUMBS:
        return THUMBS[key]
    try:
        from PIL import Image
    except ImportError:
        return None
    try:
        with Image.open(path) as im:
            im.thumbnail((THUMB_PX, THUMB_PX))
            if im.mode not in ("RGB", "RGBA"):
                im = im.convert("RGBA")
            buf = io.BytesIO()
            im.save(buf, "PNG")
    except Exception:
        return None
    THUMBS[key] = buf.getvalue()
    return THUMBS[key]


CSS = """
*{box-sizing:border-box}
body{background:#1a1b26;color:#c0caf5;font:14px/1.55 system-ui,-apple-system,sans-serif;
     margin:0;padding:26px 26px 60px}
a{color:#7aa2f7;text-decoration:none} a:hover{text-decoration:underline}
header{display:flex;align-items:baseline;gap:16px;flex-wrap:wrap;margin-bottom:4px}
h1{font-size:20px;font-weight:650;margin:0}
.sub{color:#565f89;font-size:13px}
h2{font-size:13px;font-weight:650;color:#9ece6a;margin:30px 0 12px;text-transform:uppercase;
   letter-spacing:.06em;border-bottom:1px solid #2f3550;padding-bottom:7px}
.bar{display:flex;gap:10px;flex-wrap:wrap;margin:14px 0 0}
.chip{background:#20222f;border:1px solid #2f3550;border-radius:20px;padding:5px 13px;font-size:12.5px}
.chip b{color:#e0af68;font-weight:600}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(210px,1fr));gap:15px}
.card{background:#20222f;border:1px solid #2f3550;border-radius:8px;padding:9px;
      transition:border-color .12s,transform .12s}
.card:hover{border-color:#7aa2f7;transform:translateY(-2px)}
.card img{width:100%;display:block;border-radius:5px;background:#15161f;cursor:zoom-in}
.meta{display:flex;justify-content:space-between;gap:8px;margin-top:7px;font-size:12px}
.meta .nm{word-break:break-all}
.meta .sz{color:#565f89;white-space:nowrap;font-variant-numeric:tabular-nums}
table{border-collapse:collapse;font-size:13px} td{padding:3px 20px 3px 0}
.sz{color:#565f89;font-variant-numeric:tabular-nums}
/* full-screen viewer */
#v{position:fixed;inset:0;background:#0e0f16f2;display:none;z-index:9;
   align-items:center;justify-content:center;flex-direction:column}
#v.on{display:flex}
#vi{max-width:96vw;max-height:84vh;object-fit:contain;cursor:zoom-in}
#vi.zoom{max-width:none;max-height:none;cursor:zoom-out}
#vw{overflow:auto;max-width:100vw;max-height:86vh;display:flex;align-items:center}
#vb{padding:10px 16px;display:flex;gap:14px;align-items:center;font-size:13px;flex-wrap:wrap;
    justify-content:center}
#vb .t{color:#c0caf5;font-weight:600}
#vb a,#vb button{color:#7aa2f7;background:none;border:1px solid #2f3550;border-radius:6px;
   padding:4px 11px;font:inherit;cursor:pointer}
#vb button:hover{border-color:#7aa2f7}
kbd{background:#20222f;border:1px solid #2f3550;border-radius:4px;padding:1px 5px;font-size:11px;
    color:#565f89}
"""

VIEWER_JS = """
const items = %s;
let idx = -1, zoomed = false;
const v = document.getElementById('v'), vi = document.getElementById('vi'),
      vt = document.getElementById('vt'), vlinks = document.getElementById('vlinks');
function open_(i){
  idx = (i + items.length) %% items.length;
  const it = items[idx];
  vi.src = it.url; vi.className = ''; zoomed = false;
  vt.textContent = `${it.name}  —  ${it.dim}  ${it.size}  (${idx+1}/${items.length})`;
  vlinks.innerHTML = it.alts.map(a =>
      `<a href="${a.url}" download>${a.label}</a>`).join(' ');
  v.classList.add('on'); document.body.style.overflow='hidden';
}
function close_(){ v.classList.remove('on'); document.body.style.overflow=''; }
document.querySelectorAll('[data-i]').forEach(el =>
  el.addEventListener('click', e => { e.preventDefault(); open_(+el.dataset.i); }));
vi.addEventListener('click', () => { zoomed=!zoomed; vi.className = zoomed?'zoom':''; });
document.getElementById('vx').addEventListener('click', close_);
v.addEventListener('click', e => { if(e.target === v) close_(); });
addEventListener('keydown', e => {
  if(!v.classList.contains('on')) return;
  if(e.key==='Escape') close_();
  if(e.key==='ArrowRight'||e.key===' ') { e.preventDefault(); open_(idx+1); }
  if(e.key==='ArrowLeft') open_(idx-1);
});
"""


def page(title: str, sub: str, body: str, items: list[dict]) -> bytes:
    viewer = ""
    if items:
        viewer = f"""
<div id=v>
  <div id=vw><img id=vi alt=""></div>
  <div id=vb><span class=t id=vt></span><span id=vlinks></span>
    <button id=vx>close</button>
    <span><kbd>←</kbd><kbd>→</kbd> page · <kbd>esc</kbd> close · click to zoom</span></div>
</div>
<script>{VIEWER_JS % json.dumps(items)}</script>"""
    return f"""<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>{html.escape(title)}</title><style>{CSS}</style>
<header><h1>{html.escape(title)}</h1><span class=sub>{sub}</span></header>
{body}{viewer}
""".encode()




# ── part-by-part review ─────────────────────────────────────────────────────


def flags_path() -> Path:
    return CACHE / "flags.json"


def load_flags() -> None:
    global FLAGS
    try:
        FLAGS = json.loads(flags_path().read_text())
    except (OSError, ValueError):
        FLAGS = {}


def save_flags() -> None:
    CACHE.mkdir(parents=True, exist_ok=True)
    tmp = flags_path().with_suffix(".tmp")
    tmp.write_text(json.dumps(FLAGS, indent=1, sort_keys=True))
    tmp.replace(flags_path())


def part_png(i: int) -> Path:
    """Cache path for a part's render. Keyed by name + mtime, so re-exported
    parts re-render instead of serving a stale picture."""
    p = PARTS[i]
    try:
        stamp = int(p.stat().st_mtime)
    except OSError:
        stamp = 0
    safe = re.sub(r"[^A-Za-z0-9_.-]", "_", p.stem)[:80]
    return CACHE / f"{safe}.{stamp}.{PART_PX}.png"


def render_part(i: int) -> Path | None:
    """Render part i if not already cached. Also captures --stats output."""
    out = part_png(i)
    name = PARTS[i].name
    if out.exists() and name in STATS:
        return out
    CACHE.mkdir(parents=True, exist_ok=True)
    if not out.exists():
        r = subprocess.run(
            [str(BIN), "iso", str(PARTS[i]), "-o", str(out), "--size", str(PART_PX)],
            capture_output=True,
            text=True,
            timeout=600,
        )
        if r.returncode != 0 or not out.exists():
            STATS[name] = f"render failed: {(r.stderr or r.stdout).strip()[:400]}"
            return None
    if name not in STATS:
        try:
            r = subprocess.run(
                [str(BIN), "mesh", str(PARTS[i]), "-o", os.devnull, "--stats"],
                capture_output=True,
                text=True,
                timeout=900,
            )
            STATS[name] = (r.stdout + r.stderr).strip() or "(no stats)"
        except (subprocess.TimeoutExpired, OSError) as e:
            STATS[name] = f"stats failed: {e}"
    return out


def prefetch(around: int, n: int | None = None) -> None:
    """Warm the next few parts so paging does not wait on a render."""
    if POOL is None:
        return
    n = PREFETCH if n is None else n
    for j in range(around + 1, min(around + 1 + n, len(PARTS))):
        if j in INFLIGHT or part_png(j).exists():
            continue
        INFLIGHT.add(j)
        POOL.submit(lambda k=j: (render_part(k), INFLIGHT.discard(k)))


def stat_line(name: str) -> str:
    """One-line summary pulled out of `mesh --stats` output."""
    txt = STATS.get(name, "")
    for ln in txt.splitlines():
        if "watertight" in ln or "boundary" in ln:
            return ln.strip()
    return txt.splitlines()[-1].strip() if txt else "(not measured yet)"


REVIEW_CSS = """
#stage{display:flex;flex-direction:column;align-items:center;gap:10px;padding:8px 12px 30px}
#img{max-width:min(92vw,900px);max-height:70vh;object-fit:contain;
     background:#12131c;border:1px solid #2f3550;border-radius:10px}
#img.flagged{border-color:#f7768e;box-shadow:0 0 0 3px #f7768e33}
#nm{font-size:15px;color:#c0caf5;word-break:break-all;text-align:center;max-width:92vw}
#st{font-size:12px;color:#565f89;font-variant-numeric:tabular-nums;text-align:center}
#bar{display:flex;gap:10px;align-items:center;flex-wrap:wrap;justify-content:center}
#bar button,#bar a{color:#7aa2f7;background:none;border:1px solid #2f3550;border-radius:6px;
  padding:7px 16px;font:inherit;cursor:pointer;text-decoration:none}
#bar button:hover{border-color:#7aa2f7}
#flagbtn.on{color:#f7768e;border-color:#f7768e;background:#f7768e18}
#note{background:#12131c;border:1px solid #2f3550;border-radius:6px;color:#c0caf5;
  padding:7px 10px;font:inherit;width:min(90vw,420px)}
#count{color:#e0af68;font-variant-numeric:tabular-nums}
#pos{color:#565f89;font-variant-numeric:tabular-nums}
.spin{color:#565f89;font-size:13px}
"""

REVIEW_JS = """
const N = %(n)d;
let i = %(i)d, flags = %(flags)s;
// Unsaved note text, keyed by part name. A note on an unflagged part has
// nowhere to live server-side, so it is held here until the part is flagged --
// and either way the box is only ever rewritten when we move to a new part.
const drafts = {};
let shown = null;
const img=document.getElementById('img'), nm=document.getElementById('nm'),
      st=document.getElementById('st'), pos=document.getElementById('pos'),
      fb=document.getElementById('flagbtn'), note=document.getElementById('note'),
      cnt=document.getElementById('count');

function paint(meta, navigated){
  nm.textContent = meta.name;
  st.textContent = meta.stat;
  pos.textContent = `${i+1} / ${N}`;
  const on = !!flags[meta.name];
  fb.classList.toggle('on', on);
  fb.textContent = on ? 'flagged (F)' : 'flag (F)';
  img.classList.toggle('flagged', on);
  if(navigated){
    shown = meta.name;
    const saved = on ? (flags[meta.name].note || '') : '';
    note.value = saved || drafts[meta.name] || '';
  }
  cnt.textContent = Object.keys(flags).length + ' flagged';
  history.replaceState(null,'','/review/'+i);
}
async function go(d){
  const t = i + d;
  if(t < 0 || t >= N) return;
  if(shown) drafts[shown] = note.value;   // keep the draft when leaving
  i = t;
  st.textContent = 'rendering...'; st.className='spin';
  img.src = '/review/img/' + i + '.png';
  const meta = await (await fetch('/review/meta/' + i)).json();
  st.className='';
  paint(meta, true);
}
async function post(action){
  const r = await fetch('/review/flag/' + i, {method:'POST',
    headers:{'Content-Type':'application/json'},
    body: JSON.stringify({action: action, note: note.value})});
  flags = (await r.json()).flags;
  // navigated=false: repainting must never touch the box under the cursor.
  paint(await (await fetch('/review/meta/' + i)).json(), false);
}
const toggleFlag = () => post('toggle');
let noteTimer;
note.addEventListener('input', () => {
  if(shown) drafts[shown] = note.value;
  clearTimeout(noteTimer);
  // Only worth a round trip once the part is flagged; otherwise the draft
  // waits here and rides along when F is pressed.
  if(shown && flags[shown]) noteTimer = setTimeout(() => post('note'), 400);
});
document.getElementById('prev').onclick = () => go(-1);
document.getElementById('next').onclick = () => go(1);
fb.onclick = toggleFlag;
addEventListener('keydown', e => {
  if(e.target.tagName === 'INPUT' || e.target.tagName === 'TEXTAREA') return;
  if(e.key === 'ArrowLeft')  { e.preventDefault(); go(-1); }
  if(e.key === 'ArrowRight' || e.key === ' ') { e.preventDefault(); go(1); }
  if(e.key === 'f' || e.key === 'F') { e.preventDefault(); toggleFlag(); }
});
go(0);
"""


def review_page(i: int) -> bytes:
    with FLAGS_LOCK:
        fl = json.dumps(FLAGS)
    body = f"""
<div id=stage>
  <div id=nm></div>
  <img id=img alt="part render">
  <div id=st></div>
  <div id=bar>
    <button id=prev>&larr; prev</button>
    <span id=pos></span>
    <button id=next>next &rarr;</button>
    <button id=flagbtn>flag (F)</button>
    <input id=note placeholder="what looks wrong? (press F to flag with this note)">
    <a href="/review/report">report</a>
    <span id=count></span>
  </div>
  <div class=spin><kbd>&larr;</kbd><kbd>&rarr;</kbd> page &middot;
     <kbd>F</kbd> flag &middot; notes are saved with the flag</div>
</div>
<script>{REVIEW_JS % {"n": len(PARTS), "i": i, "flags": fl}}</script>"""
    return f"""<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>part review</title><style>{CSS}{REVIEW_CSS}</style>
<header><h1>part review</h1><span class=sub>{len(PARTS)} parts &middot;
  <a href="/" style="color:#7aa2f7">sheets</a></span></header>
{body}
""".encode()


def report_markdown() -> str:
    with FLAGS_LOCK:
        items = sorted(FLAGS.items())
    by_name = {p.name: p for p in PARTS}
    out = [
        "# Parts flagged as problematic",
        "",
        f"{len(items)} of {len(PARTS)} parts flagged.",
        "",
    ]
    for name, meta in items:
        p = by_name.get(name)
        out.append(f"## {name}")
        if p:
            out.append(f"- path: `{p}`")
        if meta.get("note"):
            out.append(f"- note: {meta['note']}")
        stat = STATS.get(name)
        if stat:
            out.append("- stats:")
            out.append("")
            out.append("```")
            out.append(stat)
            out.append("```")
        out.append("")
    return "\n".join(out)


def write_report() -> Path:
    CACHE.mkdir(parents=True, exist_ok=True)
    path = CACHE / "flagged_report.md"
    path.write_text(report_markdown())
    return path


def report_page() -> bytes:
    path = write_report()
    with FLAGS_LOCK:
        items = sorted(FLAGS.items())
    rows = []
    for name, meta in items:
        rows.append(
            f"<tr><td class=nm>{html.escape(name)}</td>"
            f"<td>{html.escape(meta.get('note') or '')}</td>"
            f"<td class=sz>{html.escape(stat_line(name))}</td></tr>"
        )
    table = (
        "<table><tr><th>part</th><th>note</th><th>stats</th></tr>"
        + "".join(rows)
        + "</table>"
        if rows
        else "<p>Nothing flagged yet. Press <kbd>F</kbd> in the reviewer.</p>"
    )
    body = f"""
<p style="padding:0 12px">Written to <code>{html.escape(str(path))}</code> —
   point the agent at that file.
   <a href="/review/report.md" download style="color:#7aa2f7">download .md</a> &middot;
   <a href="/review" style="color:#7aa2f7">back to review</a></p>
<div style="padding:0 12px">{table}</div>
<pre style="margin:16px 12px;padding:12px;background:#12131c;border:1px solid #2f3550;
    border-radius:8px;white-space:pre-wrap;color:#9aa5ce;font-size:12px">{
    html.escape(report_markdown())}</pre>"""
    return f"""<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>flagged parts</title><style>{CSS}{REVIEW_CSS}</style>
<header><h1>flagged parts</h1><span class=sub>{len(items)} flagged</span></header>
{body}
""".encode()


class Server(ThreadingHTTPServer):
    daemon_threads = True
    # A browser opens several connections per page and a render can hold a
    # thread for seconds; the stdlib default backlog of 5 drops the rest, which
    # a proxy in front reports as a connection timeout rather than a refusal.
    request_queue_size = 128
    allow_reuse_address = True


class Handler(BaseHTTPRequestHandler):
    server_version = "solid-diff-renders"
    # Keep-alive: without it every asset is a fresh connection, which is what
    # exhausts the backlog above. Every response here sets Content-Length.
    protocol_version = "HTTP/1.1"
    # ...but an idle keep-alive connection must not live forever. Each one
    # pins a thread, and anything upstream that reads to EOF rather than
    # honouring Content-Length hangs until *its* timeout instead of ours --
    # which a proxy reports as the forward timing out. This is *idle* time
    # between requests, so it never interrupts a slow render; three seconds is
    # far longer than a page's burst needs and short enough to stay under any
    # upstream probe timeout.
    timeout = 3

    def log_message(self, fmt, *args):
        sys.stderr.write("  %s\n" % (fmt % args))

    def _resolve(self, urlpath: str) -> Path | None:
        rel = urllib.parse.unquote(urlpath.split("?", 1)[0]).lstrip("/")
        target = (ROOT / rel).resolve()
        if target != ROOT.resolve() and ROOT.resolve() not in target.parents:
            return None
        return target

    def do_GET(self):
        self._serve(False)

    def do_POST(self):
        raw = self.path.split("?", 1)[0]
        if raw.startswith("/review/flag/"):
            return self._flag(raw[len("/review/flag/") :])
        self.send_error(HTTPStatus.NOT_FOUND)

    def _json(self, obj):
        data = json.dumps(obj).encode()
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def _flag(self, tail: str):
        try:
            i = int(tail)
            part = PARTS[i]
        except (ValueError, IndexError):
            return self.send_error(HTTPStatus.NOT_FOUND)
        n = int(self.headers.get("Content-Length") or 0)
        body = {}
        if n:
            try:
                body = json.loads(self.rfile.read(n)) or {}
            except ValueError:
                body = {}
        note = body.get("note", "")
        # "toggle" flips the flag; "note" only edits the note of an existing
        # flag, so typing in the box never silently unflags a part.
        action = body.get("action", "toggle")
        with FLAGS_LOCK:
            was = part.name in FLAGS
            if action == "note":
                state = was
                if was:
                    FLAGS[part.name]["note"] = note
            elif was:
                FLAGS.pop(part.name)
                state = False
            else:
                FLAGS[part.name] = {
                    "note": note,
                    "path": str(part),
                    "at": time.strftime("%Y-%m-%d %H:%M:%S"),
                }
                state = True
            save_flags()
            snapshot = dict(FLAGS)
        if action != "note":
            sys.stderr.write(
                f"  {'FLAG  ' if state else 'unflag'} {part.name}"
                f"{'  - ' + note if note and state else ''}\n"
            )
        return self._json({"flagged": state, "flags": snapshot})

    def do_HEAD(self):
        self._serve(True)

    def _serve(self, head_only: bool):
        raw = self.path.split("?", 1)[0]
        if raw.startswith("/_thumb/"):
            return self._send_thumb(raw[len("/_thumb/") :], head_only)
        if raw == "/review" or raw.startswith("/review/"):
            return self._review(raw, head_only)
        target = self._resolve(raw)
        if target is None or not target.exists():
            self.send_error(HTTPStatus.NOT_FOUND, "not found")
            return
        if target.is_dir():
            return self._send_index(target, head_only)
        return self._send_file(target, head_only)

    def _review(self, raw: str, head_only: bool):
        if not PARTS:
            self.send_error(HTTPStatus.NOT_FOUND, "server started without --parts")
            return
        tail = raw[len("/review") :].strip("/")
        if tail == "report":
            return self._send_bytes(report_page(), "text/html; charset=utf-8", head_only)
        if tail == "report.md":
            write_report()
            return self._send_bytes(
                report_markdown().encode(), "text/markdown; charset=utf-8", head_only
            )
        if tail.startswith("img/"):
            try:
                i = int(tail[4:].removesuffix(".png"))
                PARTS[i]
            except (ValueError, IndexError):
                return self.send_error(HTTPStatus.NOT_FOUND)
            out = render_part(i)
            prefetch(i)
            if out is None:
                return self.send_error(HTTPStatus.INTERNAL_SERVER_ERROR, "render failed")
            return self._send_file(out, head_only)
        if tail.startswith("meta/"):
            try:
                i = int(tail[5:])
                part = PARTS[i]
            except (ValueError, IndexError):
                return self.send_error(HTTPStatus.NOT_FOUND)
            with FLAGS_LOCK:
                flagged = part.name in FLAGS
            return self._json(
                {
                    "i": i,
                    "name": part.name,
                    "path": str(part),
                    "stat": stat_line(part.name),
                    "flagged": flagged,
                }
            )
        i = 0
        if tail.isdigit():
            i = max(0, min(int(tail), len(PARTS) - 1))
        prefetch(i)
        return self._send_bytes(review_page(i), "text/html; charset=utf-8", head_only)

    def _send_bytes(self, data: bytes, ctype: str, head_only: bool):
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        if not head_only:
            self.wfile.write(data)

    def _send_thumb(self, rel: str, head_only: bool):
        target = self._resolve("/" + rel)
        if target is None or not target.is_file():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        data = thumbnail(target)
        if data is None:
            return self._send_file(target, head_only)
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "image/png")
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "max-age=3600")
        self.end_headers()
        if not head_only:
            self.wfile.write(data)

    def _variants(self, name: str) -> list[dict]:
        """Same sheet at other resolutions, for the viewer's switcher."""
        out = []
        for d in sorted(ROOT.iterdir()):
            if not d.is_dir():
                continue
            p = d / name
            if p.is_file():
                dim = png_size(p)
                label = f"{d.name.replace('sheets_', '')} {dim[0]}px" if dim else d.name
                out.append({"url": f"/{d.name}/{urllib.parse.quote(name)}", "label": label})
        return out

    def _send_index(self, d: Path, head_only: bool):
        entries = sorted(d.iterdir(), key=lambda p: (not p.is_dir(), p.name.lower()))
        dirs = [p for p in entries if p.is_dir()]
        images = [p for p in entries if p.is_file() and p.suffix.lower() in IMAGE_EXT]
        others = [p for p in entries if p.is_file() and p.suffix.lower() not in IMAGE_EXT]

        rel_root = d.resolve().relative_to(ROOT.resolve())
        at_root = str(rel_root) == "."
        prefix = "/" + ("" if at_root else str(rel_root) + "/")
        parts, items = [], []

        if not at_root:
            parts.append('<p><a href="/">&larr; all renders</a></p>')

        if dirs:
            parts.append("<h2>collections</h2><div class=bar>")
            for p in dirs:
                imgs = [q for q in p.iterdir() if q.suffix.lower() in IMAGE_EXT]
                dim = png_size(imgs[0]) if imgs else None
                d_s = f"{dim[0]}&times;{dim[1]}" if dim else ""
                tot = human(sum(q.stat().st_size for q in imgs))
                parts.append(
                    f'<a class=chip href="{html.escape(prefix + p.name)}/">'
                    f"<b>{html.escape(p.name)}</b> &middot; {len(imgs)} sheets &middot; "
                    f"{d_s} &middot; {tot}</a>"
                )
            parts.append("</div>")

        if others:
            parts.append("<h2>downloads</h2><table>")
            for p in others:
                parts.append(
                    f'<tr><td><a href="{html.escape(prefix + p.name)}" download>'
                    f"{html.escape(p.name)}</a></td>"
                    f"<td class=sz>{human(p.stat().st_size)}</td></tr>"
                )
            parts.append("</table>")

        if images:
            total = human(sum(p.stat().st_size for p in images))
            parts.append(f"<h2>{len(images)} sheets &middot; {total}</h2><div class=grid>")
            for i, p in enumerate(images):
                href = prefix + p.name
                dim = png_size(p)
                dim_s = f"{dim[0]}&times;{dim[1]}" if dim else ""
                items.append(
                    {
                        "url": href,
                        "name": p.name,
                        "dim": dim_s.replace("&times;", "x"),
                        "size": human(p.stat().st_size),
                        "alts": self._variants(p.name),
                    }
                )
                parts.append(
                    f'<div class=card><img loading=lazy data-i="{i}" '
                    f'src="/_thumb{html.escape(href)}" alt="{html.escape(p.name)}">'
                    f'<div class=meta><span class=nm><a href="{html.escape(href)}" download>'
                    f"{html.escape(p.stem)}</a></span>"
                    f'<span class=sz>{human(p.stat().st_size)}</span></div></div>'
                )
            parts.append("</div>")

        n_files = sum(1 for _ in ROOT.rglob("*") if _.is_file())
        body = page(
            "solid-diff renders" if at_root else f"renders / {rel_root}",
            f"{n_files} files &middot; click a sheet to open the viewer",
            "\n".join(parts) or "<p>empty</p>",
            items,
        )
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        if not head_only:
            self.wfile.write(body)

    def _send_file(self, path: Path, head_only: bool):
        size = path.stat().st_size
        ctype = mimetypes.guess_type(path.name)[0] or "application/octet-stream"
        if path.suffix.lower() in {".zip", ".obj", ".stl"}:
            ctype = "application/octet-stream"

        start, end, status = 0, size - 1, HTTPStatus.OK
        rng = self.headers.get("Range")
        if rng:
            m = re.match(r"bytes=(\d*)-(\d*)", rng.strip())
            if m and (m.group(1) or m.group(2)):
                if m.group(1):
                    start = int(m.group(1))
                    if m.group(2):
                        end = min(int(m.group(2)), size - 1)
                else:
                    start = max(0, size - int(m.group(2)))
                if start > end or start >= size:
                    self.send_response(HTTPStatus.REQUESTED_RANGE_NOT_SATISFIABLE)
                    self.send_header("Content-Range", f"bytes */{size}")
                    self.end_headers()
                    return
                status = HTTPStatus.PARTIAL_CONTENT

        length = end - start + 1
        self.send_response(status)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        if status == HTTPStatus.PARTIAL_CONTENT:
            self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.end_headers()
        if head_only:
            return
        with open(path, "rb") as f:
            f.seek(start)
            remaining = length
            while remaining > 0:
                chunk = f.read(min(256 * 1024, remaining))
                if not chunk:
                    break
                try:
                    self.wfile.write(chunk)
                except (BrokenPipeError, ConnectionResetError):
                    return
                remaining -= len(chunk)


def main():
    global ROOT, PARTS, BIN, CACHE, POOL, PART_PX, PREFETCH
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default="renders", type=Path)
    ap.add_argument("--port", type=int, default=8791)
    ap.add_argument("--bind", default="0.0.0.0")
    ap.add_argument(
        "--parts", type=Path, help="directory of .SLDPRT to review one at a time"
    )
    ap.add_argument("--bin", type=Path, default=BIN, help="solid-diff binary")
    ap.add_argument("--cache", type=Path, default=None, help="render cache dir")
    ap.add_argument("--size", type=int, default=PART_PX, help="review render px")
    ap.add_argument("--jobs", type=int, default=4, help="background prefetch workers")
    ap.add_argument(
        "--prefetch", type=int, default=PREFETCH, help="parts to render ahead"
    )
    args = ap.parse_args()

    ROOT = args.root.resolve()
    if not ROOT.is_dir():
        sys.exit(f"no such directory: {ROOT}")

    if args.parts:
        if not args.parts.is_dir():
            sys.exit(f"no such directory: {args.parts}")
        BIN = args.bin.resolve()
        if not BIN.is_file():
            sys.exit(f"solid-diff binary not found: {BIN}  (pass --bin)")
        PART_PX = args.size
        PREFETCH = args.prefetch
        CACHE = (args.cache or ROOT / ".review-cache").resolve()
        PARTS = sorted(args.parts.rglob("*.SLDPRT"), key=lambda p: p.name.lower())
        if not PARTS:
            sys.exit(f"no .SLDPRT under {args.parts}")
        load_flags()
        POOL = ThreadPoolExecutor(max_workers=args.jobs)
        print(
            f"review: {len(PARTS)} parts from {args.parts} "
            f"({len(FLAGS)} already flagged), {PREFETCH} rendered ahead "
            f"on {args.jobs} workers, cache {CACHE}"
        )

    srv = Server((args.bind, args.port), Handler)
    n = sum(1 for _ in ROOT.rglob("*") if _.is_file())
    print(f"serving {ROOT} ({n} files) on http://{socket.gethostname()}:{args.port}/")
    if PARTS:
        print(f"  part reviewer:  /review     flags -> {CACHE / 'flags.json'}")
        print(f"  flagged report: /review/report  ->  {CACHE / 'flagged_report.md'}")
    print("expose it with:  agent-portal forward", args.port)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
