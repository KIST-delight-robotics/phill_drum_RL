#!/usr/bin/env python3
"""motors CSV 를 matplotlib 창으로 띄운다. 확대·이동은 툴바, 점 위에 마우스를 올리면 값이 뜬다.

    python3 plot_motors_live.py <motors.csv> --ids 0 1 3 4 --from 113 --deg

  위치(desired/actual) · 오차 · 토크 3단, 관절별 열. 모든 x 축이 연동된다.
  키:  g  격자 토글    m  마커(점) 토글    q  닫기
"""
import argparse, os, sys
import numpy as np, pandas as pd
import matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt, matplotlib.font_manager as fm
for c in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic"):
    if any(f.name == c for f in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = c; break
plt.rcParams["axes.unicode_minus"] = False

NAMES = {0:"waist",1:"right_shoulder_1",2:"left_shoulder_1",3:"right_shoulder_2",4:"right_elbow",
         5:"left_shoulder_2",6:"left_elbow",7:"right_wrist",8:"left_wrist",9:"right_pedal",10:"left_pedal"}
MODE = {0:"CST",1:"POS/CSP",2:"VEL",3:"MIT"}

p = argparse.ArgumentParser()
p.add_argument("csv"); p.add_argument("--ids", type=int, nargs="+")
p.add_argument("--from", dest="t0", type=float); p.add_argument("--to", dest="t1", type=float)
p.add_argument("--deg", action="store_true")
p.add_argument("--vel", action="store_true", help="속도 행 추가 (4단: 위치/속도/오차/토크). actual 은 로그 velocity 열, desired 는 desired 의 차분")
p.add_argument("--overlay", help="같은 형식의 두 번째 CSV (예: 캔덤프 변환본). 점선으로 겹쳐 그린다")
a = p.parse_args()

def load(path):
    df = pd.read_csv(path)
    df.columns = ["t","id","mode","desired","actual","err","torque","input","velocity","tick"][:len(df.columns)]
    df = df.dropna(subset=["id", "t"])            # 급종료로 잘린 마지막 행 제거
    df["id"] = df["id"].round().astype(int)
    if a.t0 is not None: df = df[df.t >= a.t0]
    if a.t1 is not None: df = df[df.t <= a.t1]
    return df
df = load(a.csv)
ov = load(a.overlay) if a.overlay else None
ids = a.ids or sorted(df.id.unique())
k = 180/np.pi if a.deg else 1.0; unit = "deg" if a.deg else "rad"

NR = 4 if a.vel else 3
fig, axes = plt.subplots(NR, len(ids), figsize=(5.5*len(ids), 2.6*NR+1), squeeze=False, sharex=True)
fig.canvas.manager.set_window_title(os.path.basename(a.csv))
lines = []                                   # (line, label) — 호버 대상
for c, jid in enumerate(ids):
    d = df[df.id == jid].drop_duplicates('t'); t = d.t.to_numpy()   # 1kHz 로그의 중복 t 제거 (차분 0 나눗셈 방지)
    mode = MODE.get(int(d["mode"].mode().iloc[0]), "?")
    ap, ae, at = axes[0][c], axes[-2][c], axes[-1][c]
    if a.vel:
        av = axes[1][c]
        # desired 속도는 로그에 없어 desired 위치를 차분한다 (5ms 격자, 3점 이동평균)
        dv = np.gradient(d.desired.to_numpy(), t) if len(t) > 2 else np.zeros_like(t)
        dv = np.convolve(dv, np.ones(3)/3, mode="same")
        vact = d.velocity.to_numpy() if "velocity" in d and d.velocity.abs().sum() > 0 else np.convolve(np.gradient(d.actual.to_numpy(), t), np.ones(3)/3, mode="same")
        v1, = av.plot(t, dv*k, lw=1.2, color="#1D4E89", label="desired (차분)")
        v2, = av.plot(t, vact*k, lw=1.0, color="#A34B26", label="actual" + ("" if d.velocity.abs().sum() > 0 else " (차분)"))
        lines += [(v1, f"id{jid} vel desired"), (v2, f"id{jid} vel actual")]
        av.set_ylabel(f"속도 [{unit}/s]"); av.axhline(0, lw=0.8, color="0.7"); av.grid(alpha=0.25, lw=0.5)
        av.legend(loc="upper left", fontsize=8)
    l1, = ap.plot(t, d.desired*k, lw=1.4, color="#1D4E89", label="desired")
    l2, = ap.plot(t, d.actual*k,  lw=1.1, color="#A34B26", label="actual")
    l3, = ae.plot(t, d.err*k, lw=1.1, color="#8E6009")
    l4, = at.plot(t, d.torque, lw=1.1, color="#A62F1F")
    for l, lab in ((l1,"desired"),(l2,"actual"),(l3,"err"),(l4,"torque/current")):
        lines.append((l, f"id{jid} {lab}"))
    if ov is not None:
        o = ov[ov.id == jid]; to = o.t.to_numpy()
        o1, = ap.plot(to, o.desired*k, lw=1.0, ls="--", color="#5B9BD5", label="desired (overlay)")
        o2, = ap.plot(to, o.actual*k,  lw=1.0, ls="--", color="#E08A5B", label="actual (overlay)")
        o3, = ae.plot(to, o.err*k, lw=1.0, ls="--", color="#C9A227")
        o4, = at.plot(to, o.torque, lw=1.0, ls="--", color="#E06C5B")
        for l, lab in ((o1,"desired"),(o2,"actual"),(o3,"err"),(o4,"torque/current")):
            lines.append((l, f"id{jid} {lab} [overlay]"))
    ap.set_title(f"id {jid}  {NAMES.get(jid, '')}   ({mode})", loc="left", fontsize=10)
    ap.set_ylabel(f"위치 [{unit}]"); ae.set_ylabel(f"오차 [{unit}]")
    at.set_ylabel("전류 [A]" if mode == "VEL" else "토크 [N·m]"); at.set_xlabel("t [s]")
    ae.axhline(0, lw=0.8, color="0.7"); at.axhline(0, lw=0.8, color="0.7")
    ap.legend(loc="upper left", fontsize=8)
    for ax in (ap, ae, at): ax.grid(alpha=0.25, lw=0.5)

# ── 호버: 가장 가까운 점의 (t, 값) 을 주석으로
ann = fig.text(0.5, 0.005, "", ha="center", fontsize=9, color="0.25")
dot, = axes[0][0].plot([], [], "o", ms=5, color="k", zorder=5)
def on_move(ev):
    if ev.inaxes is None or ev.xdata is None: return
    best = None
    for l, lab in lines:
        if l.axes is not ev.inaxes: continue
        x, y = l.get_xdata(), l.get_ydata()
        i = np.searchsorted(x, ev.xdata); i = min(max(i, 0), len(x)-1)
        if i > 0 and abs(x[i-1]-ev.xdata) < abs(x[i]-ev.xdata): i -= 1
        # 화면 픽셀 거리로 가장 가까운 선을 고른다
        px, py = ev.inaxes.transData.transform((x[i], y[i]))
        dist = np.hypot(px-ev.x, py-ev.y)
        if best is None or dist < best[0]: best = (dist, lab, x[i], y[i], l.axes)
    if best is None: return
    _, lab, x, y, ax = best
    ann.set_text(f"{lab}    t = {x:.4f} s    y = {y:.4f}")
    dot.set_data([x], [y])
    if dot.axes is not ax:
        dot.remove(); ax.add_line(dot)
    fig.canvas.draw_idle()
def on_key(ev):
    if ev.key == "m":
        for l, _ in lines: l.set_marker("" if l.get_marker() == "." else "."); l.set_markersize(3)
        fig.canvas.draw_idle()
    elif ev.key == "q": plt.close(fig)
fig.canvas.mpl_connect("motion_notify_event", on_move)
fig.canvas.mpl_connect("key_press_event", on_key)
fig.suptitle(os.path.basename(a.csv) + (f"   실선   |   점선 = {os.path.basename(a.overlay)}" if a.overlay else ""), fontsize=11)
fig.tight_layout(rect=[0, 0.02, 1, 0.97])
plt.show()
