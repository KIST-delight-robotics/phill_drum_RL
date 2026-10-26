#!/usr/bin/env python3
"""policy CSV 를 matplotlib 창으로 띄운다.

    python3 plot_policy_live.py <policy.csv> joints   # 관절 9개: 정책이 본 joint_pos, 발행한 q_cmd, 액션 원값
    python3 plot_policy_live.py <policy.csv> hits     # 타격: 팁 높이, 재장전 상태, 다음 타격, 팔 역할

  x 축은 tick×5ms (컨트롤러 시각, motors 로그와 같은 축). 마우스를 올리면 하단에 값이 뜬다.
"""
import sys, os, numpy as np, pandas as pd, matplotlib
matplotlib.use("QtAgg")
import matplotlib.pyplot as plt, matplotlib.font_manager as fm
for c in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic"):
    if any(f.name == c for f in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = c; break
plt.rcParams["axes.unicode_minus"] = False

OBS = ["waist","L_sh1","R_sh1","L_sh2","R_sh2","L_elbow","R_elbow","L_wrist","R_wrist"]
MOTOR_BY_OBS = [0,2,1,5,3,6,4,8,7]
DRUMS = ["snare","floor","mid","top","hihat","ride","Rcrash","Lcrash"]
k = 180/np.pi

path, mode = sys.argv[1], (sys.argv[2] if len(sys.argv) > 2 else "joints")
p = pd.read_csv(path).dropna(subset=["tick"])
t = p.tick.to_numpy()*0.005
lines = []
if mode == "joints":
    fig, axes = plt.subplots(3, 3, figsize=(16, 9.5), sharex=True)
    for i in range(9):
        ax = axes[i//3][i%3]; m = MOTOR_BY_OBS[i]
        l1, = ax.plot(t, p[f"q_cmd_{m}"]*k, lw=1.3, color="#1D4E89", label="q_cmd (목표)")
        l2, = ax.plot(t, p[f"joint_pos_{i}"]*k, lw=1.1, color="#A34B26", label="joint_pos (실측)")
        ax2 = ax.twinx(); l3, = ax2.plot(t, p[f"act_{i}"], lw=0.8, color="#5B8E3E", alpha=0.8, label="act (원값)")
        ax2.set_ylim(-1.5, 1.5); ax2.axhline(1, lw=0.6, ls="--", color="#5B8E3E"); ax2.axhline(-1, lw=0.6, ls="--", color="#5B8E3E")
        ax2.set_ylabel("act", color="#5B8E3E", fontsize=8)
        ax.set_title(f"obs[{i}] {OBS[i]}  (motor id {m})", loc="left", fontsize=10); ax.set_ylabel("deg"); ax.grid(alpha=0.25, lw=0.5)
        if i == 0: ax.legend(handles=[l1,l2,l3], loc="upper left", fontsize=8)
        lines += [(l1, f"{OBS[i]} q_cmd"), (l2, f"{OBS[i]} joint_pos"), (l3, f"{OBS[i]} act")]
    for ax in axes[-1]: ax.set_xlabel("t [s] (tick×5ms)")
    fig.suptitle(f"{os.path.basename(path)}  관절: 목표 q_cmd · 실측 joint_pos · 액션 원값(±1 점선)", fontsize=11)
else:
    fig, axes = plt.subplots(4, 1, figsize=(15, 10), sharex=True)
    # 1) 팁 높이 + 드럼 z
    ax = axes[0]
    l1, = ax.plot(t, p.tip_pos_2, lw=1.2, color="#1D4E89", label="L tip z"); l2, = ax.plot(t, p.tip_pos_5, lw=1.2, color="#A34B26", label="R tip z")
    dz = [p[f"drum_pos_{d*3+2}"].iloc[0] for d in range(8)]
    for d, z in enumerate(dz): ax.axhline(z, lw=0.6, ls=":", color="0.5"); ax.text(t[0], z, f" {DRUMS[d]}", fontsize=7, va="bottom", color="0.4")
    ax.set_ylabel("tip z [m]"); ax.legend(loc="upper left", fontsize=8); ax.grid(alpha=0.25, lw=0.5)
    lines += [(l1, "L tip z"), (l2, "R tip z")]
    # 2) hit_armed 상태 (팔×드럼) — 1 이면 재장전됨. 해제 순간(1→0)에 마커
    ax = axes[1]
    ha = p[[f"hit_armed_{i}" for i in range(16)]].to_numpy()
    for a, (arm, col) in enumerate((("L", "#1D4E89"), ("R", "#A34B26"))):
        for d in range(8):
            s = ha[:, a*8+d]; y = d + (0.15 if a else -0.15)
            on = s > 0.5
            ax.plot(t[on], np.full(on.sum(), y), "|", ms=4, color=col, alpha=0.5)
            drop = np.where((s[:-1] > 0.5) & (s[1:] < 0.5))[0] + 1
            if len(drop):
                l, = ax.plot(t[drop], np.full(len(drop), y), "v", ms=8, color=col, label=f"{arm} 타격 판정" if d == 0 else None)
                lines.append((l, f"{arm} hit"))
    ax.set_yticks(range(8)); ax.set_yticklabels(DRUMS); ax.set_ylabel("hit_armed"); ax.legend(loc="upper left", fontsize=8); ax.grid(alpha=0.25, lw=0.5)
    # 3) next_hits[0]: 다음 타격까지 시간(정규화) 와 드럼
    ax = axes[2]
    nh = p[[f"next_hits_{i}" for i in range(66)]].to_numpy().reshape(len(p), 6, 11)
    tn = np.where(nh[:,0,9] > 0.5, nh[:,0,8]*1.98, np.nan)
    l, = ax.plot(t, tn, lw=1.2, color="#8E6009", label="next_hits[0] 까지 남은 시간 [s]"); lines.append((l, "t_to_hit"))
    drum0 = np.where(nh[:,0,9] > 0.5, nh[:,0,:8].argmax(1), np.nan)
    ax2 = ax.twinx(); l2, = ax2.plot(t, drum0, ".", ms=3, color="#5B8E3E", label="next_hits[0] 드럼"); ax2.set_yticks(range(8)); ax2.set_yticklabels(DRUMS, fontsize=7)
    lines.append((l2, "next drum"))
    ax.set_ylabel("[s]"); ax.legend(loc="upper left", fontsize=8); ax.grid(alpha=0.25, lw=0.5)
    # 4) arm_role, t_score, valid 개수
    ax = axes[3]
    l1, = ax.plot(t, p.arm_role_0, lw=1.2, color="#1D4E89", label="arm_role L"); l2, = ax.plot(t, p.arm_role_1 + 1.2, lw=1.2, color="#A34B26", label="arm_role R (+1.2)")
    l3, = ax.plot(t, nh[:,:,9].sum(1)/6*0.9 + 2.5, lw=1.0, color="0.4", label="valid 개수/6 (+2.5)")
    ax.set_yticks([0,1,1.2,2.2,2.5,3.4]); ax.set_yticklabels(["L 0","L 1","R 0","R 1","valid 0","valid 6"], fontsize=7)
    ax.legend(loc="upper left", fontsize=8); ax.grid(alpha=0.25, lw=0.5); ax.set_xlabel("t [s] (tick×5ms)")
    lines += [(l1, "arm_role L"), (l2, "arm_role R"), (l3, "valid")]
    fig.suptitle(f"{os.path.basename(path)}  타격: 팁 높이 · 재장전 · 다음 타격 · 팔 역할   t_score {p.t_score.min():.1f}~{p.t_score.max():.1f}s", fontsize=11)

ann = fig.text(0.5, 0.005, "", ha="center", fontsize=9, color="0.25")
def on_move(ev):
    if ev.inaxes is None or ev.xdata is None: return
    best = None
    for l, lab in lines:
        if l.axes is not ev.inaxes and getattr(l.axes, "_sharex", None) is not ev.inaxes and not (hasattr(ev.inaxes, "get_shared_x_axes") and l.axes.bbox.bounds == ev.inaxes.bbox.bounds): continue
        x, y = np.asarray(l.get_xdata(), float), np.asarray(l.get_ydata(), float)
        if len(x) == 0: continue
        i = int(np.clip(np.searchsorted(x, ev.xdata), 0, len(x)-1))
        if i > 0 and abs(x[i-1]-ev.xdata) < abs(x[i]-ev.xdata): i -= 1
        if np.isnan(y[i]): continue
        px, py = l.axes.transData.transform((x[i], y[i])); dist = np.hypot(px-ev.x, py-ev.y)
        if best is None or dist < best[0]: best = (dist, lab, x[i], y[i])
    if best: ann.set_text(f"{best[1]}    t = {best[2]:.3f} s    y = {best[3]:.4f}"); fig.canvas.draw_idle()
fig.canvas.mpl_connect("motion_notify_event", on_move)
fig.canvas.manager.set_window_title(f"policy {mode}")
fig.tight_layout(rect=[0, 0.02, 1, 0.96]); plt.show()
