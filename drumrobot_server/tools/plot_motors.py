#!/usr/bin/env python3
"""motors 로그를 관절별로 그린다. 목표/실측 · 오차 · 토크 세 단.

로그 컬럼 (Controller::tmotor_send_task / maxon_motor_send_task)
    t, id, mode, desired, actual, err, current/torque, input

  mode      1=POS  2=VEL  3=MIT  0=CST  (Maxon CSP 는 1)
  desired   TMotor: 모터위치[rad]        Maxon: 모터위치[rad]
  actual    피드백 위치[rad]
  err       desired - actual [rad]
  7번째     MIT: 피드백 토크[N·m]        서보: 전류[A]        CST: 피드백 토크[N·m]
  input     MIT: 걸린 mit_kp             VEL: control_input   CST: 계산 토크[mN·m]

가드가 트립한 틱은 record 를 하지 않으므로 그 구간은 로그에 공백으로 남는다.
공백은 회색 띠로 표시한다.

사용법
    python3 tools/plot_motors.py log/log_0904_1554_motors.csv
    python3 tools/plot_motors.py <csv> --ids 2 1
    python3 tools/plot_motors.py <csv> --ids 2 --from 51.8 --to 52.1
    python3 tools/plot_motors.py <csv> --ids 2 --deg          # 각도를 도로
    python3 tools/plot_motors.py <csv> --out plot.png         # 저장만
"""
import argparse, sys, os
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")                       # 헤드리스에서도 저장된다
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 한글 라벨용 폰트. 없으면 기본 폰트로 두고 경고만 난다.
for _cand in ("Noto Sans CJK KR", "Noto Sans CJK JP", "NanumGothic",
              "Malgun Gothic", "AppleGothic"):
    if any(f.name == _cand for f in fm.fontManager.ttflist):
        plt.rcParams["font.family"] = _cand
        break
plt.rcParams["axes.unicode_minus"] = False  # 마이너스 기호가 두부가 되지 않게

COLS = ["t", "id", "mode", "desired", "actual", "err", "torque", "input"]
MODE = {0: "CST", 1: "POS/CSP", 2: "VEL", 3: "MIT"}

NAMES = {0: "waist", 1: "right_shoulder_1", 2: "left_shoulder_1",
         3: "right_shoulder_2", 4: "right_elbow", 5: "left_shoulder_2",
         6: "left_elbow", 7: "right_wrist", 8: "left_wrist",
         9: "right_pedal", 10: "left_pedal"}

# motors.json 의 토크 한계 (참고선)
T_LIMIT = {0: 65.0}          # waist AK10-9
T_SAFETY = {0: 48.0}
T_LIMIT_DEFAULT, T_SAFETY_DEFAULT = 25.0, 24.0

DT = 0.005                   # 제어 주기. 이보다 큰 간격은 기록 공백


def load(path):
    df = pd.read_csv(path)
    if list(df.columns[:len(COLS)]) != COLS:
        # 헤더가 다르면 이름을 강제로 붙인다 (구버전 로그)
        df = pd.read_csv(path, header=0, names=COLS[:len(df.columns)])
    df["id"] = df["id"].round().astype(int)
    df["mode"] = df["mode"].round().astype(int)
    return df


def gaps(t, thresh=DT * 1.5):
    """기록 공백 구간 [(시작, 끝), ...]. 가드 트립 흔적."""
    out = []
    for a, b in zip(t[:-1], t[1:]):
        if b - a > thresh:
            out.append((a, b))
    return out


def plot_joint(df, jid, t0, t1, use_deg, ax_pos, ax_err, ax_tau):
    d = df[df["id"] == jid]
    if t0 is not None:
        d = d[d["t"] >= t0]
    if t1 is not None:
        d = d[d["t"] <= t1]
    if d.empty:
        for a in (ax_pos, ax_err, ax_tau):
            a.text(0.5, 0.5, f"id {jid}: 데이터 없음", ha="center", va="center",
                   transform=a.transAxes, color="0.5")
        return None

    t = d["t"].to_numpy()
    k = 180.0 / np.pi if use_deg else 1.0
    unit = "deg" if use_deg else "rad"
    mode = MODE.get(int(d["mode"].mode().iloc[0]), "?")
    name = NAMES.get(jid, f"id {jid}")

    # ── 1단: 목표 vs 실측
    ax_pos.plot(t, d["desired"] * k, lw=1.6, label="desired", color="#1D4E89")
    ax_pos.plot(t, d["actual"] * k, lw=1.2, label="actual", color="#A34B26")
    ax_pos.set_ylabel(f"위치 [{unit}]")
    ax_pos.set_title(f"id {jid}  {name}   ({mode})", loc="left", fontsize=11)
    ax_pos.legend(loc="upper left", fontsize=8, framealpha=0.9)

    # ── 2단: 오차
    ax_err.plot(t, d["err"] * k, lw=1.3, color="#8E6009")
    ax_err.axhline(0, lw=0.8, color="0.7")
    if mode == "MIT":                      # 송신 전 급변 가드 30도
        lim = 30.0 if use_deg else np.deg2rad(30.0)
        for s in (+1, -1):
            ax_err.axhline(s * lim, lw=1.0, ls="--", color="#A62F1F")
        ax_err.text(t[0], lim, " POS_DIFF_LIMIT 30deg", va="bottom",
                    fontsize=7.5, color="#A62F1F")
    ax_err.set_ylabel(f"오차 [{unit}]")

    # ── 3단: 토크(또는 전류)
    is_cur = mode == "VEL"                 # 서보는 전류[A]
    ax_tau.plot(t, d["torque"], lw=1.3, color="#A62F1F")
    ax_tau.axhline(0, lw=0.8, color="0.7")
    ax_tau.set_ylabel("전류 [A]" if is_cur else "토크 [N·m]")
    if not is_cur:
        tl = T_LIMIT.get(jid, T_LIMIT_DEFAULT)
        ts = T_SAFETY.get(jid, T_SAFETY_DEFAULT)
        for s in (+1, -1):
            ax_tau.axhline(s * tl, lw=1.0, ls="-", color="#A62F1F", alpha=0.5)
            ax_tau.axhline(s * ts, lw=1.0, ls=":", color="#8E6009")
        ax_tau.text(t[0], tl, f" mit_t_limit {tl:g}", va="bottom",
                    fontsize=7.5, color="#A62F1F")
        ax_tau.text(t[0], ts, f" torque_safety {ts:g}", va="top",
                    fontsize=7.5, color="#8E6009")
    ax_tau.set_xlabel("t [s]")

    # ── 기록 공백 = 가드 트립 구간
    g = gaps(t)
    for a, b in g:
        for ax in (ax_pos, ax_err, ax_tau):
            ax.axvspan(a, b, color="0.85", zorder=0)
    if g:
        ax_pos.text(g[0][0], ax_pos.get_ylim()[1],
                    " 회색 = 기록 공백 (가드 트립)", va="top",
                    fontsize=7.5, color="0.4")

    for ax in (ax_pos, ax_err, ax_tau):
        ax.grid(alpha=0.25, lw=0.5)

    return {
        "id": jid, "name": name, "mode": mode, "n": len(d),
        "t0": t[0], "t1": t[-1],
        "err_max": float(d["err"].abs().max()) * k,
        "tau_max": float(d["torque"].abs().max()),
        "kp": float(d["input"].iloc[0]) if mode == "MIT" else float("nan"),
        "gaps": g,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("csv")
    p.add_argument("--ids", type=int, nargs="+", default=None,
                   help="그릴 관절 id. 기본은 로그에 있는 전부")
    p.add_argument("--from", dest="t0", type=float, default=None)
    p.add_argument("--to", dest="t1", type=float, default=None)
    p.add_argument("--deg", action="store_true", help="각도를 도로 표시")
    p.add_argument("--out", default=None, help="저장 경로 (기본: <csv>_plot.png)")
    a = p.parse_args()

    df = load(a.csv)
    ids = a.ids if a.ids else sorted(df["id"].unique())

    fig, axes = plt.subplots(3, len(ids), figsize=(6.2 * len(ids), 9.5),
                             squeeze=False, sharex="col")
    stats = []
    for c, jid in enumerate(ids):
        s = plot_joint(df, jid, a.t0, a.t1, a.deg,
                       axes[0][c], axes[1][c], axes[2][c])
        if s:
            stats.append(s)

    win = ""
    if a.t0 is not None or a.t1 is not None:
        win = f"   t {a.t0 if a.t0 is not None else '시작'} ~ {a.t1 if a.t1 is not None else '끝'}"
    fig.suptitle(f"{os.path.basename(a.csv)}{win}", fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.97])

    out = a.out or (os.path.splitext(a.csv)[0] + "_plot.png")
    fig.savefig(out, dpi=130)
    print(f"저장: {out}")

    # 요약을 표준출력에
    print()
    print(f"{'id':>3} {'이름':<20} {'모드':<8} {'행':>7} {'t 범위':>16} "
          f"{'최대|오차|':>10} {'최대|토크|':>10} {'Kp':>6} {'공백':>5}")
    for s in stats:
        u = "deg" if a.deg else "rad"
        print(f"{s['id']:>3} {s['name']:<20} {s['mode']:<8} {s['n']:>7} "
              f"{s['t0']:>7.2f}~{s['t1']:<8.2f} "
              f"{s['err_max']:>7.4f}{u:<3} {s['tau_max']:>10.2f} "
              f"{s['kp']:>6.0f} {len(s['gaps']):>5}")


if __name__ == "__main__":
    sys.exit(main())
