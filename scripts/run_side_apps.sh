#!/usr/bin/env bash
#
# make run 이 서버를 띄운 직후, 라이브 플로터와 TCP 클라이언트를
# 각각 별도 터미널 창으로 띄운다. (make run 에서 백그라운드로 호출됨)
#
# 환경변수:
#   CLIENT=0    클라이언트 안 띄움
#   PLOTTER=0   플로터 안 띄움
#   TIMEOUT=15  서버 포트(1951) 기다릴 최대 초
#   TERM_CMD    터미널 에뮬레이터 (기본 gnome-terminal)

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${PORT:-1951}"
TIMEOUT="${TIMEOUT:-15}"
TERM_CMD="${TERM_CMD:-gnome-terminal}"

# GUI 없는 환경(SSH 등)이면 조용히 빠진다
if [[ -z "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
    echo "[side] GUI 세션이 아님 → 클라이언트/플로터는 수동으로 띄우세요."
    exit 0
fi
if ! command -v "$TERM_CMD" >/dev/null; then
    echo "[side] $TERM_CMD 를 찾을 수 없음 → 클라이언트/플로터는 수동으로 띄우세요."
    exit 0
fi

# ---------------------------------------------------------------- 플로터
# CAN 만 엿보는 물건이라 서버를 기다릴 필요가 없다. 바로 띄운다.
if [[ "${PLOTTER:-1}" == 1 ]]; then
    echo "[side] 라이브 플로터 실행"
    "$TERM_CMD" --title="Phil Live Plotter" \
        -- "$ROOT/Phil_Live_plotter/run.sh" --keep-open
fi

# -------------------------------------------------------------- 클라이언트
# 서버가 listen 을 시작해야 접속이 되므로 포트가 열릴 때까지 기다린다.
[[ "${CLIENT:-1}" == 1 ]] || exit 0

port_in_use() {
    ss -ltn "sport = :$1" 2>/dev/null | tail -n +2 | grep -q .
}

if command -v ss >/dev/null; then
    echo "[side] 서버 포트 $PORT 대기 중 (최대 ${TIMEOUT}초)..."
    tries=$(( TIMEOUT * 5 ))
    until port_in_use "$PORT"; do
        if (( tries-- <= 0 )); then
            echo "[side] 포트 $PORT 가 안 열림. 클라이언트는 그냥 띄웁니다."
            break
        fi
        sleep 0.2
    done
else
    sleep 3   # ss 가 없으면 그냥 몇 초 기다린다
fi

echo "[side] TCP 클라이언트 실행"
"$TERM_CMD" --title="Drum Client" --working-directory="$ROOT" \
    -- bash -c 'python3 drumrobot_client/main.py; echo; read -rp "[client] 종료됨. Enter 를 누르면 창이 닫힙니다. "'
