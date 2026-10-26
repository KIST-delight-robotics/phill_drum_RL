#!/usr/bin/env bash
#
# 라이브 플로터를 새 터미널 창에서 띄운다. DrumRobot 의 `make run` 이 호출한다.
# 이미 떠 있으면 (포트가 잡혀 있으면) 새로 띄우지 않는다.
#
#   scripts/launch_terminal.sh [플로터_경로] [run.sh 에 넘길 옵션...]

set -uo pipefail

PLOTTER_DIR="${1:-$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)}"
shift || true

if [[ ! -x "$PLOTTER_DIR/run.sh" ]]; then
    echo "[launch] $PLOTTER_DIR/run.sh 를 찾을 수 없습니다."
    exit 1
fi

# 이미 떠 있으면 중복 실행하지 않는다
if ss -ltn 'sport = :8765' 2>/dev/null | tail -n +2 | grep -q .; then
    echo "[launch] 플로터가 이미 실행 중입니다 (포트 8765). 새로 띄우지 않습니다."
    exit 0
fi

CMD="cd '$PLOTTER_DIR' && ./run.sh --keep-open $*"

if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] && command -v gnome-terminal >/dev/null; then
    gnome-terminal --title="Live CAN Plotter" -- bash -lc "$CMD"
elif [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] && command -v x-terminal-emulator >/dev/null; then
    x-terminal-emulator -e bash -lc "$CMD"
elif [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]] && command -v xterm >/dev/null; then
    xterm -T "Live CAN Plotter" -e bash -lc "$CMD" &
else
    # GUI 세션이 아니면 (ssh 등) 백그라운드로 돌리고 로그만 남긴다
    echo "[launch] 새 터미널을 열 수 없어 백그라운드로 실행합니다. 로그: $PLOTTER_DIR/logs/plotter.out"
    mkdir -p "$PLOTTER_DIR/logs"
    ( cd "$PLOTTER_DIR" && setsid ./run.sh --no-browser "$@" > logs/plotter.out 2>&1 & )
fi
