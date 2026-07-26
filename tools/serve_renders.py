#!/usr/bin/env python3
"""Browsable gallery for rendered contact sheets.

Serves a directory of renders as a small website: a thumbnail grid, a
full-screen viewer with keyboard paging and zoom, and download links. Sheets
that exist at several resolutions are grouped so you can switch between them
without leaving the viewer.

    python3 tools/serve_renders.py [--root renders] [--port 8791]

Pair with `agent-portal forward <port>` to reach it from a browser.

Thumbnails are generated once and cached in memory (Pillow if available), so a
page of 4K sheets loads in kilobytes rather than tens of megabytes. Range
requests are supported so large downloads resume.
"""

from __future__ import annotations

import argparse
import html
import io
import json
import mimetypes
import re
import socket
import sys
import urllib.parse
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

IMAGE_EXT = {".png", ".jpg", ".jpeg", ".gif", ".webp"}
THUMB_PX = 340

ROOT = Path(".")
THUMBS: dict[str, bytes] = {}


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


class Handler(BaseHTTPRequestHandler):
    server_version = "solid-diff-renders"

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

    def do_HEAD(self):
        self._serve(True)

    def _serve(self, head_only: bool):
        raw = self.path.split("?", 1)[0]
        if raw.startswith("/_thumb/"):
            return self._send_thumb(raw[len("/_thumb/") :], head_only)
        target = self._resolve(raw)
        if target is None or not target.exists():
            self.send_error(HTTPStatus.NOT_FOUND, "not found")
            return
        if target.is_dir():
            return self._send_index(target, head_only)
        return self._send_file(target, head_only)

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
    global ROOT
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--root", default="renders", type=Path)
    ap.add_argument("--port", type=int, default=8791)
    ap.add_argument("--bind", default="0.0.0.0")
    args = ap.parse_args()

    ROOT = args.root.resolve()
    if not ROOT.is_dir():
        sys.exit(f"no such directory: {ROOT}")

    srv = ThreadingHTTPServer((args.bind, args.port), Handler)
    srv.daemon_threads = True
    n = sum(1 for _ in ROOT.rglob("*") if _.is_file())
    print(f"serving {ROOT} ({n} files) on http://{socket.gethostname()}:{args.port}/")
    print("expose it with:  agent-portal forward", args.port)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\nstopped")


if __name__ == "__main__":
    main()
