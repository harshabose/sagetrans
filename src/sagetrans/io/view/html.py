"""The page shell: tokens, CSS, the one inline script, and the table and chapter renderers.

One self-contained file: no network, no CDN, system fonts only. The colour tokens are the ones the
palette validator passed (docs/OUTPUTS_spec.md 7): s1 orange (C), s2 blue (B1 attached), s3 aqua (B1
wing-to-30), checked in light and dark against the panel surface; the light aqua is below 3:1 against
white, which the spec allows only with visible labels and a table view, and every chart here has both.
Grey ink (`ref`) is reserved for reference series, and reference series are dashed.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass, field
from typing import Any

from sagetrans.io.view.charts import esc
from sagetrans.io.view.units import fmt_value, header_label, unit_of

_LIGHT = (
    "--bg:#f3f5f8;--panel:#ffffff;--ink:#13171c;--ink2:#464f5b;--mute:#5f6a77;--rule:#dce1e8;"
    "--grid:#e8ebf0;--axis:#9aa4b0;--accent:#1f4e8c;--accent-soft:#e4edf8;--note:#fff6df;"
    "--note-ink:#6b4a00;--warn:#b45309;--s1:#eb6834;--s2:#2a78d6;--s3:#1baf7a;--refc:#7d8794;"
    "--seq-hi:#1f4e8c;--seq-lo:#ffffff;"
)
_DARK = (
    "--bg:#0f1316;--panel:#171c20;--ink:#eceff3;--ink2:#b6bec9;--mute:#8b96a3;--rule:#2a3138;"
    "--grid:#232a30;--axis:#5d6874;--accent:#86b6ef;--accent-soft:#1c2a3b;--note:#2a2412;"
    "--note-ink:#f1d48a;--warn:#f0a35a;--s1:#d95926;--s2:#3987e5;--s3:#199e70;--refc:#8b96a3;"
    "--seq-hi:#86b6ef;--seq-lo:#171c20;color-scheme:dark;"
)

CSS = (
    (
        ":root{" + _LIGHT + "--sans:system-ui,-apple-system,'Segoe UI',Roboto,sans-serif;"
        "--mono:ui-monospace,'SFMono-Regular',Menlo,Consolas,monospace}"
        "@media (prefers-color-scheme:dark){:root:not([data-theme=light]){" + _DARK + "}}"
        ":root[data-theme=dark]{" + _DARK + "}"
    )
    + """
*{box-sizing:border-box}
body{background:var(--bg);color:var(--ink);font:15px/1.55 var(--sans);margin:0;
 padding:20px max(16px,env(safe-area-inset-right,0px)) 56px max(16px,env(safe-area-inset-left,0px))}
.shell{max-width:1240px;margin:0 auto;display:grid;grid-template-columns:210px minmax(0,1fr);gap:40px}
nav.side{position:sticky;top:20px;align-self:start;font-size:13px;max-height:calc(100vh - 40px);
 overflow:auto}
nav.side .brand{font-weight:600;font-size:14px;margin:0 0 2px}
nav.side .sub{color:var(--mute);margin:0 0 14px;font-size:12px}
nav.side ol{list-style:none;margin:0;padding:0;display:grid;gap:2px}
nav.side a{display:block;padding:5px 8px;margin-inline:-8px;color:var(--ink2);text-decoration:none;
 border-radius:4px}
nav.side a:hover,nav.side a:focus-visible,nav.side a.cur{background:var(--accent-soft);
 color:var(--ink);outline:none}
nav.side a.off{color:var(--mute)}
nav.side .grp{margin:14px 0 4px;color:var(--mute);font-size:11px;letter-spacing:.08em;
 text-transform:uppercase}
main{min-width:0;display:grid;grid-template-columns:minmax(0,1fr);gap:44px}
h1{font-size:30px;line-height:1.15;margin:0 0 6px;font-weight:600;text-wrap:balance}
h2{font-size:21px;margin:0 0 4px;font-weight:600;text-wrap:balance}
h3{font-size:15px;margin:0 0 6px;font-weight:600}
h4{font-size:13px;margin:0 0 4px;font-weight:600;color:var(--ink2)}
p{margin:0 0 10px;max-width:72ch}
.lede{color:var(--ink2);max-width:66ch}
.meta{font:12px/1.5 var(--mono);color:var(--mute);margin:10px 0 0;overflow-wrap:anywhere}
.chips{display:flex;flex-wrap:wrap;gap:6px;margin:12px 0 0}
.chip{display:inline-flex;align-items:center;gap:5px;font-size:12px;line-height:1;padding:5px 8px;
 border-radius:999px;border:1px solid var(--rule);background:var(--panel);color:var(--ink2);
 max-width:100%;overflow-wrap:anywhere}
.chip.warn{border-color:var(--warn);color:var(--warn)}
.chip.info{border-color:var(--accent);color:var(--accent)}
.chip.mute{color:var(--mute)}
.banner{background:var(--note);color:var(--note-ink);border:1px solid
 color-mix(in srgb,var(--note-ink) 30%,transparent);padding:10px 12px;border-radius:6px;
 font-size:14px;max-width:none}
section.chap{display:grid;grid-template-columns:minmax(0,1fr);gap:16px;min-width:0;
 scroll-margin-top:16px}
.eyebrow{font-size:11px;letter-spacing:.09em;text-transform:uppercase;color:var(--mute);margin:0 0 4px}
.facts{display:grid;grid-template-columns:repeat(auto-fit,minmax(210px,1fr));
 border-block:1px solid var(--rule)}
.fact{padding:14px 16px 14px 0;margin-right:16px;border-right:1px solid var(--rule);min-width:0}
.fact:last-child{border-right:0}
.fact .k{font-size:12px;color:var(--mute);margin:0 0 4px}
.fact .v{font:500 21px/1.2 var(--mono);margin:0 0 4px;font-variant-numeric:tabular-nums}
.fact .v.long{font:13px/1.45 var(--sans)}
.fact .v small{font:12px var(--sans);color:var(--ink2)}
.fact .n{font:12.5px var(--mono);color:var(--mute);margin:0;max-width:none;overflow-wrap:anywhere}
.flags{display:flex;flex-wrap:wrap;gap:6px}
.panelbox{background:var(--panel);border:1px solid var(--rule);border-radius:8px;
 padding:16px 16px 12px;min-width:0}
.panelbox .cap{color:var(--ink2);font-size:13.5px;margin:0 0 10px;max-width:none}
.absent{color:var(--mute);font-size:13.5px;border-left:3px solid var(--rule);padding:4px 10px;
 margin:0;max-width:none}
.legend{display:flex;flex-wrap:wrap;gap:6px 18px;font-size:13px;color:var(--ink2);margin:0 0 8px}
.legend span{display:inline-flex;align-items:center;gap:7px}
.sw{display:inline-block;width:18px;height:0;border-top:3px solid var(--s1)}
.sw.s2{border-color:var(--s2)}.sw.s3{border-color:var(--s3)}.sw.ink{border-color:var(--ink2)}
.sw.ref{border-color:var(--refc);border-top-style:dashed}
.sw.dash{border-top-style:dashed}
.sw.dot,.sw.sq,.sw.dia,.sw.hollow{width:10px;height:10px;border:0;background:var(--s1)}
.sw.dot,.sw.hollow{border-radius:50%}
.sw.dia{transform:rotate(45deg);width:9px;height:9px}
.sw.s2.dot,.sw.s2.sq,.sw.s2.dia{background:var(--s2)}
.sw.s3.dot,.sw.s3.sq,.sw.s3.dia{background:var(--s3)}
.sw.ink.dot,.sw.ink.sq,.sw.ink.dia{background:var(--ink2)}
.sw.ref.dot,.sw.ref.sq,.sw.ref.dia{background:var(--refc)}
.sw.hollow{background:var(--panel);border:2px solid var(--s1)}
.sw.cellno{width:12px;height:12px;border:1px solid var(--axis);border-radius:2px;background:var(--panel)}
.sw.cellok{width:12px;height:12px;border-radius:2px;background:var(--accent-soft);
 border:1px solid var(--accent)}
svg.viz{width:100%;height:auto;display:block;overflow:visible}
.grid{stroke:var(--grid);stroke-width:1}.axis{stroke:var(--axis);stroke-width:1}
.zero{stroke:var(--axis);stroke-width:1;stroke-dasharray:3 3}
.rule{stroke:var(--ink2);stroke-width:1;stroke-dasharray:2 4}
.tick{fill:var(--mute);font:11px var(--mono)}.axlab{fill:var(--ink2);font:12px var(--sans)}
.ptitle{fill:var(--ink2);font:500 12px var(--sans)}.dl{fill:var(--ink2);font:12px var(--sans)}
.ln{fill:none;stroke-width:2;stroke-linejoin:round;stroke-linecap:round;stroke:var(--s1)}
.ln.thick{stroke-width:3}.ln.s2{stroke:var(--s2)}.ln.s3{stroke:var(--s3)}
.ln.ink{stroke:var(--ink2);stroke-width:1.5}.ln.ref{stroke:var(--refc)}
.ln.dash{stroke-dasharray:6 5}
.mk{stroke:var(--panel);stroke-width:2;fill:var(--s1)}
.mk.s2{fill:var(--s2)}.mk.s3{fill:var(--s3)}.mk.ink{fill:var(--ink2)}.mk.ref{fill:var(--refc)}
.mk.hollow{fill:var(--panel);stroke:var(--s1);stroke-width:2.2}
.mk.hollow.s2{stroke:var(--s2)}.mk.hollow.s3{stroke:var(--s3)}.mk.hollow.ref{stroke:var(--refc)}
.mk:focus-visible,.mk:hover,.bar:hover,.bar:focus-visible,.cell:hover,.cell:focus-visible{
 outline:none;stroke:var(--ink);stroke-width:2.5}
.bar{fill:var(--s1)}.bar.s2{fill:var(--s2)}.bar.s3{fill:var(--s3)}.bar.ink{fill:var(--ink2)}
.bar.ref{fill:var(--refc)}
.refmk{stroke:var(--refc);stroke-width:3}
.band{fill:var(--accent-soft);opacity:.7}
.cell{stroke:var(--panel)}.cell.ok{fill:var(--accent-soft)}.cell.no{fill:var(--panel);
 stroke:var(--axis);stroke-dasharray:2 2}
.cell.na{fill:none;stroke:var(--rule)}.cell.flag{stroke:var(--warn);stroke-width:2}
.cellv{fill:var(--ink);font:11.5px var(--mono)}
.xh{stroke:var(--ink2);stroke-width:1}
.pans{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px 18px}
.pan{min-width:0}
.mult{display:grid;gap:14px 18px}.mult.c2{grid-template-columns:repeat(2,minmax(0,1fr))}
.mult.c3{grid-template-columns:repeat(3,minmax(0,1fr))}.sm{min-width:0}
details{margin-top:6px;font-size:13.5px}
summary{cursor:pointer;color:var(--accent);padding:4px 0}
summary:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
.tbl.full{max-height:none}
.tbl{overflow:auto;max-height:520px;border:1px solid var(--rule);border-radius:8px;
 background:var(--panel);margin-top:6px}
table{border-collapse:separate;border-spacing:0;width:100%;font-size:13px;min-width:520px}
th,td{text-align:left;padding:7px 12px;border-bottom:1px solid var(--rule);vertical-align:top}
th{position:sticky;top:0;z-index:1;font-weight:600;font-size:12px;color:var(--ink2);
 background:color-mix(in srgb,var(--bg) 60%,var(--panel));white-space:nowrap}
tr:last-child td{border-bottom:0}
td.num,th.num{text-align:right;font-family:var(--mono);font-variant-numeric:tabular-nums;
 white-space:nowrap}
tr.flagged td:first-child{box-shadow:inset 3px 0 0 var(--warn)}
tr.dim td{color:var(--mute)}
td.dflt{color:var(--mute);font-style:italic}
td code,.mono{font-family:var(--mono);font-size:12.5px}
td.wrap{min-width:16ch;max-width:46ch;overflow-wrap:anywhere}
pre.verbatim{font:12px/1.5 var(--mono);background:var(--panel);border:1px solid var(--rule);
 border-radius:8px;padding:12px;overflow:auto;max-height:420px;margin:6px 0 0;white-space:pre-wrap}
.how{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:16px}
#tip{position:fixed;z-index:10;pointer-events:none;background:var(--ink);color:var(--bg);
 font:12px/1.45 var(--sans);padding:7px 9px;border-radius:6px;max-width:320px;
 box-shadow:0 2px 10px rgba(0,0,0,.25)}
#tip .r{display:flex;gap:7px;align-items:center}#tip i{display:inline-block;width:9px;height:9px;
 border-radius:2px}
#tip .r span.mono{font-family:var(--mono)}
.s1bg{background:var(--s1)}.s2bg{background:var(--s2)}.s3bg{background:var(--s3)}
.inkbg{background:var(--ink2)}.refbg{background:var(--refc)}
@media (max-width:860px){.shell{grid-template-columns:minmax(0,1fr)}
 nav.side{position:static;max-height:none}nav.side ol{display:flex;flex-wrap:wrap;gap:4px}
 .pans,.mult.c2,.mult.c3{grid-template-columns:minmax(0,1fr)}h1{font-size:25px}}
@media print{nav.side{display:none}.shell{display:block}}
"""
)

JS = r"""
const tip=document.getElementById('tip');
function showTip(h,x,y){tip.innerHTML=h;tip.hidden=false;const r=tip.getBoundingClientRect();
 let l=Math.min(x+14,innerWidth-r.width-8),t=y+14;if(t+r.height>innerHeight-8)t=y-r.height-14;
 tip.style.left=Math.max(8,l)+'px';tip.style.top=Math.max(8,t)+'px';}
function hideTip(){tip.hidden=true;}
document.querySelectorAll('[data-tip]').forEach(el=>{
 const t=()=>el.getAttribute('data-tip');
 el.addEventListener('pointerenter',e=>showTip(t(),e.clientX,e.clientY));
 el.addEventListener('pointermove',e=>showTip(t(),e.clientX,e.clientY));
 el.addEventListener('pointerleave',hideTip);
 el.addEventListener('focus',()=>{const b=el.getBoundingClientRect();showTip(t(),b.left+b.width/2,b.top)});
 el.addEventListener('blur',hideTip);});
const HIST={};
document.querySelectorAll('script[data-hist]').forEach(s=>{HIST[s.dataset.hist]=JSON.parse(s.textContent);});
function nearest(ts,t){const n=ts.length;if(!n)return -1;const dt=(ts[n-1]-ts[0])/Math.max(n-1,1);
 if(t>ts[n-1]+dt)return -1;let lo=0,hi=n-1;while(lo<hi){const m=(lo+hi)>>1;if(ts[m]<t)lo=m+1;else hi=m;}
 if(lo>0&&t-ts[lo-1]<=ts[lo]-t)lo--;return lo;}
const SW={s1:'s1bg',s2:'s2bg',s3:'s3bg',ink:'inkbg',ref:'refbg'};
document.querySelectorAll('svg.panel').forEach(svg=>{
 const H=HIST[svg.dataset.fig];if(!H)return;
 const pl=+svg.dataset.pl,pr=+svg.dataset.pr,w=+svg.dataset.w,P=H.panels[+svg.dataset.pi];
 const xh=svg.querySelector('.xh'),hit=svg.querySelector('.hit');
 const fmt=v=>(P.sign&&v>=0?'+':'')+v.toFixed(P.dec)+(P.unit?' '+P.unit:'');
 hit.addEventListener('pointermove',e=>{
  const r=svg.getBoundingClientRect(),sx=(e.clientX-r.left)/r.width*w;
  const t=Math.max(0,Math.min(H.t1,(sx-pl)/(w-pl-pr)*H.t1));
  svg.closest('.pans').querySelectorAll('svg.panel').forEach(o=>{const a=o.querySelector('.xh');
   const x=pl+t/H.t1*(w-pl-pr);a.setAttribute('x1',x);a.setAttribute('x2',x);a.setAttribute('visibility','visible');});
  let h='<b>t = '+t.toFixed(1)+' s</b>';
  H.series.forEach(s=>{const v=s.v[P.key];if(!v)return;const k=nearest(s.t,t);
   h+='<div class="r"><i class="'+(SW[s.cls]||'s1bg')+'"></i><span>'+s.short+'</span><span class="mono" style="margin-left:auto">'
    +(k<0?'run ended':fmt(v[k]))+'</span></div>';});
  showTip(h,e.clientX,e.clientY);});
 hit.addEventListener('pointerleave',()=>{svg.closest('.pans').querySelectorAll('.xh').forEach(a=>a.setAttribute('visibility','hidden'));hideTip();});
});
const links=[...document.querySelectorAll('nav.side a[href^="#"]')];
if('IntersectionObserver' in window){const io=new IntersectionObserver(es=>{es.forEach(e=>{
 if(e.isIntersecting){links.forEach(a=>a.classList.toggle('cur',a.getAttribute('href')==='#'+e.target.id));}});},
 {rootMargin:'-10% 0px -80% 0px'});
 document.querySelectorAll('section.chap').forEach(s=>io.observe(s));}
"""


@dataclass
class Item:
    """One number of a result box: its text, unit and the table cell it is read from."""

    label: str
    value: str
    unit: str = ""
    src: str = ""  # "Table.column[row]" for a cell, empty for a text or an expression
    expr: str = ""  # the arithmetic, shown in the tooltip, when the number is not one cell
    note: str = ""


@dataclass
class Out:
    """What a chapter builder hands to the renderer."""

    items: list[Item] = field(default_factory=list)
    flags: list[str] = field(default_factory=list)
    figures: list[str] = field(default_factory=list)  # panelbox HTML
    tables: list[tuple[str, list[str] | None]] = field(default_factory=list)  # (table, columns)
    absent: list[str] = field(default_factory=list)  # lines "not in this run: ..."
    notes: list[str] = field(default_factory=list)


def chip(text: str, kind: str = "") -> str:
    return f'<span class="chip {kind}">{esc(text)}</span>'


def panelbox(title: str, caption: str, body: str, legend_html: str = "") -> str:
    cap = f'<p class="cap">{esc(caption)}</p>' if caption else ""
    return f'<div class="panelbox"><h3>{esc(title)}</h3>{cap}{legend_html}{body}</div>'


def absent(msg: str) -> str:
    return f'<p class="absent">{esc(msg)}</p>'


def result_box(items: Sequence[Item]) -> str:
    cells = []
    for it in items:
        tip = f' title="{esc(it.expr)}"' if it.expr else ""
        src = f' data-src="{esc(it.src)}"' if it.src else ""
        unit = f" <small>{esc(it.unit)}</small>" if it.unit else ""
        sub = it.src or it.expr or "layout only"
        note = f'<p class="n">{esc(it.note)}</p>' if it.note else ""
        cells.append(
            f'<div class="fact"><p class="k">{esc(it.label)}</p>'
            f'<p class="v{" long" if len(it.value) > 40 else ""}"{src}{tip}>{esc(it.value)}{unit}</p>'
            f'<p class="n">{esc(sub)}</p>{note}</div>'
        )
    return f'<div class="facts">{"".join(cells)}</div>' if cells else ""


def table_html(
    name: str, columns: Sequence[str], rows: Sequence[dict[str, Any]], *, limit: int | None = None,
    extra_class: dict[str, str] | None = None,
) -> str:  # fmt: skip
    """A table in a scrolling block: numbers right-aligned with units in the header (spec 3.2)."""
    shown = rows if limit is None else rows[:limit]
    head = []
    for c in columns:
        num = any(
            isinstance(r.get(c), int | float) and not isinstance(r.get(c), bool) for r in rows
        )
        head.append(f'<th class="{"num" if num else ""}">{esc(header_label(c))}</th>')
    body = []
    for r in shown:
        flagged = bool(str(r.get("flags", "") or "")) or r.get("below_vs") is True
        cells = []
        for c in columns:
            v = r.get(c)
            num = isinstance(v, int | float) and not isinstance(v, bool)
            txt = fmt_value(v, c)
            mark = ""
            if c in ("flags", "below_vs", "overshoot") and txt not in ("", "no"):
                mark = "▲ "  # a marker as well as the word
            cls = ("num " if num else "wrap ") + (extra_class or {}).get(c, "")
            cells.append(f'<td class="{cls.strip()}">{mark}{esc(txt)}</td>')
        body.append(f"<tr{' class=flagged' if flagged else ''}>{''.join(cells)}</tr>")
    more = ""
    if limit is not None and len(rows) > limit:
        more = absent(
            f"showing {limit} of {len(rows)} rows; every row is in {name}.csv and the workbook"
        )
    return (f'<div class="tbl"><table><thead><tr>{"".join(head)}</tr></thead>'
            f'<tbody>{"".join(body)}</tbody></table></div>{more}')  # fmt: skip


def details(summary: str, inner: str, open_: bool = False) -> str:
    return f"<details{' open' if open_ else ''}><summary>{esc(summary)}</summary>{inner}</details>"


def verbatim(text: str) -> str:
    return f'<pre class="verbatim">{esc(text)}</pre>'


def column_unit_note(columns: Sequence[str]) -> str:
    """The units the header shows, for a table whose columns are not all unit-suffixed."""
    return ", ".join(header_label(c) for c in columns if unit_of(c)[0])


def page(title: str, nav: str, main: str) -> str:
    return (
        '<!doctype html><html lang="en"><head><meta charset="utf-8">'
        '<meta name="viewport" content="width=device-width,initial-scale=1">'
        f"<title>{esc(title)}</title><style>{CSS}</style></head><body>"
        f'<div class="shell">{nav}<main>{main}</main></div><div id="tip" role="tooltip" hidden></div>'
        f"<script>{JS}</script></body></html>"
    )
