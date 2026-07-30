//! 3×3-grid review over a directory of rendered PNGs.
//!
//! Clicking a tile flags that part bad (appending a triage task to
//! `--out/triage_queue.jsonl` for the agent) and clicking again un-flags it;
//! "All good" (Enter/Space) marks the rest of the page ok and advances to the
//! next nine. Verdicts persist to `--out/verdicts.json`, so a restart resumes
//! at the first unreviewed page.
//!
//!     frame-review --images renders/.review-cache --out renders/review-verdicts

use std::collections::BTreeMap;
use std::io::Write as _;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::time::{SystemTime, UNIX_EPOCH};

use axum::extract::{Path, State};
use axum::http::{header, StatusCode};
use axum::response::{Html, IntoResponse};
use axum::routing::{get, post};
use axum::{Json, Router};
use clap::Parser;
use serde::{Deserialize, Serialize};

#[derive(Parser)]
struct Args {
    /// Directory of PNGs to review (sorted by filename)
    #[arg(long)]
    images: PathBuf,
    /// Where verdicts.json and triage_queue.jsonl are written
    #[arg(long)]
    out: PathBuf,
    #[arg(long, default_value_t = 8792)]
    port: u16,
}

#[derive(Clone, Serialize)]
struct Item {
    /// Part name recovered from the cache filename (`NAME.<mtime>.<px>.png`).
    name: String,
    file: String,
}

#[derive(Clone, Serialize, Deserialize)]
struct Verdict {
    status: String, // "ok" | "bad"
    note: String,
    at: u64,
}

struct App {
    items: Vec<Item>,
    verdicts: Mutex<BTreeMap<String, Verdict>>,
    out: PathBuf,
}

/// `Foo_Bar.1784948667.900` -> `Foo_Bar` (trailing all-numeric segments are
/// the render cache's mtime/size key, not part of the name).
fn part_name(stem: &str) -> String {
    let mut segs: Vec<&str> = stem.split('.').collect();
    while segs.len() > 1 && segs.last().unwrap().bytes().all(|b| b.is_ascii_digit()) {
        segs.pop();
    }
    segs.join(".")
}

fn now() -> u64 {
    SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_secs()).unwrap_or(0)
}

fn load_verdicts(out: &std::path::Path) -> BTreeMap<String, Verdict> {
    std::fs::read_to_string(out.join("verdicts.json"))
        .ok()
        .and_then(|s| serde_json::from_str(&s).ok())
        .unwrap_or_default()
}

fn save_verdicts(out: &std::path::Path, v: &BTreeMap<String, Verdict>) {
    let tmp = out.join("verdicts.json.tmp");
    if std::fs::write(&tmp, serde_json::to_string_pretty(v).unwrap()).is_ok() {
        let _ = std::fs::rename(&tmp, out.join("verdicts.json"));
    }
}

#[tokio::main]
async fn main() {
    let args = Args::parse();
    let mut items: Vec<Item> = std::fs::read_dir(&args.images)
        .expect("cannot read --images dir")
        .filter_map(|e| e.ok())
        .map(|e| e.path())
        .filter(|p| p.extension().is_some_and(|x| x == "png"))
        .map(|p| Item {
            name: part_name(p.file_stem().unwrap().to_string_lossy().as_ref()),
            file: p.to_string_lossy().into_owned(),
        })
        .collect();
    items.sort_by(|a, b| a.name.cmp(&b.name));
    assert!(!items.is_empty(), "no .png files under --images");
    std::fs::create_dir_all(&args.out).expect("cannot create --out dir");

    let app = Arc::new(App {
        verdicts: Mutex::new(load_verdicts(&args.out)),
        items,
        out: args.out,
    });
    let n = app.items.len();
    let done = app.verdicts.lock().unwrap().len();
    eprintln!("{n} frames, {done} already reviewed");

    let router = Router::new()
        .route("/", get(page))
        .route("/api/state", get(state))
        .route("/img/{i}", get(image))
        .route("/api/verdict", post(verdict))
        .route("/api/verdicts", post(verdicts_batch))
        .with_state(app);

    // [::] is dual-stack on Linux: `localhost` resolves to ::1 first in many
    // clients, and a v4-only bind looks like a hang to them, not a refusal.
    let listener = tokio::net::TcpListener::bind(("::", args.port)).await.unwrap();
    eprintln!("listening on http://localhost:{}", args.port);
    axum::serve(listener, router).await.unwrap();
}

async fn state(State(app): State<Arc<App>>) -> Json<serde_json::Value> {
    let verdicts = app.verdicts.lock().unwrap().clone();
    Json(serde_json::json!({ "items": app.items, "verdicts": verdicts }))
}

async fn image(State(app): State<Arc<App>>, Path(i): Path<usize>) -> impl IntoResponse {
    let Some(item) = app.items.get(i) else {
        return (StatusCode::NOT_FOUND, "no such frame").into_response();
    };
    match tokio::fs::read(&item.file).await {
        Ok(bytes) => (
            [
                (header::CONTENT_TYPE, "image/png"),
                // Cache filenames embed the render mtime, so bytes never change.
                (header::CACHE_CONTROL, "max-age=86400, immutable"),
            ],
            bytes,
        )
            .into_response(),
        Err(_) => (StatusCode::NOT_FOUND, "image unreadable").into_response(),
    }
}

#[derive(Deserialize)]
struct VerdictReq {
    i: usize,
    status: String,
    #[serde(default)]
    note: String,
}

async fn verdict(
    State(app): State<Arc<App>>,
    Json(req): Json<VerdictReq>,
) -> Result<Json<serde_json::Value>, StatusCode> {
    if req.status != "ok" && req.status != "bad" {
        return Err(StatusCode::BAD_REQUEST);
    }
    let item = app.items.get(req.i).ok_or(StatusCode::NOT_FOUND)?.clone();
    let v = Verdict { status: req.status.clone(), note: req.note.clone(), at: now() };
    let (total_done, bad);
    {
        let mut verdicts = app.verdicts.lock().unwrap();
        verdicts.insert(item.name.clone(), v);
        save_verdicts(&app.out, &verdicts);
        total_done = verdicts.len();
        bad = verdicts.values().filter(|v| v.status == "bad").count();
    }
    if req.status == "bad" {
        append_task(&app, &item, &req.note);
    }
    Ok(Json(serde_json::json!({ "done": total_done, "bad": bad })))
}

/// Every ✗ is a task for the agent: append-only, one JSON object per line.
/// A later "ok" verdict retracts it — `verdicts.json` is the source of truth,
/// the queue is just the doorbell.
fn append_task(app: &App, item: &Item, note: &str) {
    let line = serde_json::json!({
        "task": "triage this render: decide whether the artifact is a parse, tessellation, or render bug, and file/fix accordingly",
        "part": item.name,
        "image": item.file,
        "note": note,
        "at": now(),
    });
    if let Ok(mut f) = std::fs::OpenOptions::new()
        .create(true)
        .append(true)
        .open(app.out.join("triage_queue.jsonl"))
    {
        let _ = writeln!(f, "{line}");
    }
}

#[derive(Deserialize)]
struct BatchReq {
    items: Vec<VerdictReq>,
}

/// The "all good" commit: several verdicts, one lock, one save. Bad entries
/// still dump triage tasks, though the UI sends those individually on click.
async fn verdicts_batch(
    State(app): State<Arc<App>>,
    Json(req): Json<BatchReq>,
) -> Result<Json<serde_json::Value>, StatusCode> {
    let (total_done, bad);
    {
        let mut verdicts = app.verdicts.lock().unwrap();
        for r in &req.items {
            if r.status != "ok" && r.status != "bad" {
                return Err(StatusCode::BAD_REQUEST);
            }
            let item = app.items.get(r.i).ok_or(StatusCode::NOT_FOUND)?;
            verdicts.insert(
                item.name.clone(),
                Verdict { status: r.status.clone(), note: r.note.clone(), at: now() },
            );
        }
        save_verdicts(&app.out, &verdicts);
        total_done = verdicts.len();
        bad = verdicts.values().filter(|v| v.status == "bad").count();
    }
    for r in &req.items {
        if r.status == "bad" {
            append_task(&app, &app.items[r.i], &r.note);
        }
    }
    Ok(Json(serde_json::json!({ "done": total_done, "bad": bad })))
}

async fn page() -> Html<&'static str> {
    Html(PAGE)
}

const PAGE: &str = r#"<!doctype html><meta charset=utf-8>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>frame review</title>
<style>
*{box-sizing:border-box}
body{background:#1a1b26;color:#c0caf5;font:14px/1.5 system-ui,sans-serif;margin:0;
     display:flex;flex-direction:column;align-items:center;gap:12px;padding:14px}
h1{font-size:17px;font-weight:650;margin:0}
#grid{display:grid;grid-template-columns:repeat(3,1fr);gap:10px;width:min(96vw,1100px)}
.cell{position:relative;background:#12131c;border:2px solid #2f3550;border-radius:10px;
      cursor:pointer;overflow:hidden}
.cell img{width:100%;aspect-ratio:1;object-fit:contain;display:block}
.cell .cap{font-size:11px;color:#565f89;padding:2px 6px 5px;word-break:break-all;
      text-align:center}
.cell:hover{border-color:#7aa2f7}
.cell.bad{border-color:#f7768e;box-shadow:0 0 0 2px #f7768e55}
.cell.bad::after{content:"\2717";position:absolute;top:6px;right:10px;color:#f7768e;
      font-size:28px;font-weight:700;text-shadow:0 0 6px #000}
.cell.ok{border-color:#9ece6a55}
.cell.empty{visibility:hidden}
#bar{display:flex;gap:12px;align-items:center;flex-wrap:wrap;justify-content:center}
button{color:#c0caf5;background:#20222f;border:1px solid #2f3550;border-radius:8px;
       padding:10px 26px;font:inherit;font-size:16px;cursor:pointer}
button:hover{border-color:#7aa2f7}
#good{color:#9ece6a;font-weight:650}
.mut{color:#565f89;font-variant-numeric:tabular-nums}
kbd{background:#20222f;border:1px solid #2f3550;border-radius:4px;padding:1px 5px;
    font-size:11px;color:#565f89}
</style>
<h1>frame review</h1>
<div id=grid></div>
<div id=bar>
  <button id=prev>&larr; prev 9</button>
  <span id=pos class=mut></span>
  <button id=good>&#10003; all good (Enter)</button>
  <button id=next>next 9 &rarr;</button>
</div>
<div class=mut><span id=stats></span> &middot; click a tile to flag it bad (click
  again to undo) &middot; <kbd>1</kbd>&ndash;<kbd>9</kbd> toggle &middot;
  <kbd>Enter</kbd>/<kbd>Space</kbd> all good &middot; <kbd>&larr;</kbd><kbd>&rarr;</kbd> pages</div>
<script>
let items=[], verdicts={}, page=0;
const N=9, grid=document.getElementById('grid'),
      pos=document.getElementById('pos'), stats=document.getElementById('stats');
const pages=()=>Math.ceil(items.length/N);
function firstUnreviewedPage(){
  for(let k=0;k<items.length;k++) if(!verdicts[items[k].name]) return Math.floor(k/N);
  return pages()-1;
}
function idxs(){
  const out=[];
  for(let k=page*N;k<Math.min((page+1)*N,items.length);k++) out.push(k);
  return out;
}
function paint(){
  grid.innerHTML='';
  for(const k of idxs()){
    const it=items[k], v=verdicts[it.name];
    const cell=document.createElement('div');
    cell.className='cell'+(v?' '+v.status:'');
    cell.innerHTML=`<img src="/img/${k}" loading=eager alt="">`+
                   `<div class=cap>${it.name}</div>`;
    cell.onclick=()=>toggle(k);
    grid.appendChild(cell);
  }
  pos.textContent=`page ${page+1} / ${pages()}`;
  const done=Object.keys(verdicts).length,
        bad=Object.values(verdicts).filter(v=>v.status==='bad').length;
  stats.textContent=`${done} / ${items.length} reviewed, ${bad} flagged for triage`;
  for(let k=(page+1)*N;k<Math.min((page+2)*N,items.length);k++)
    (new Image()).src='/img/'+k;   // preload the next page
}
async function send(one){
  await fetch('/api/verdict',{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(one)});
}
function toggle(k){
  const it=items[k], cur=verdicts[it.name];
  // click: flag bad; click again: back to ok (an ok verdict retracts the task)
  const status=(cur&&cur.status==='bad')?'ok':'bad';
  verdicts[it.name]={status,note:''};
  send({i:k,status,note:''});
  paint();
}
async function allGood(){
  const commit=idxs().filter(k=>{
    const v=verdicts[items[k].name];
    return !(v&&v.status==='bad');
  }).map(k=>({i:k,status:'ok',note:''}));
  for(const c of commit) verdicts[items[c.i].name]={status:'ok',note:''};
  if(commit.length)
    await fetch('/api/verdicts',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({items:commit})});
  if(page+1<pages()){ page++; paint(); } else paint();
}
function go(d){ page=Math.min(Math.max(page+d,0),pages()-1); paint(); }
document.getElementById('prev').onclick=()=>go(-1);
document.getElementById('next').onclick=()=>go(1);
document.getElementById('good').onclick=allGood;
addEventListener('keydown',e=>{
  if(e.key==='ArrowLeft'){e.preventDefault();go(-1);}
  if(e.key==='ArrowRight'){e.preventDefault();go(1);}
  if(e.key==='Enter'||e.key===' '){e.preventDefault();allGood();}
  if(e.key>='1'&&e.key<='9'){
    const k=page*N+(+e.key-1);
    if(k<items.length){e.preventDefault();toggle(k);}
  }
});
fetch('/api/state').then(r=>r.json()).then(s=>{
  items=s.items; verdicts=s.verdicts; page=firstUnreviewedPage(); paint();
});
</script>
"#;
