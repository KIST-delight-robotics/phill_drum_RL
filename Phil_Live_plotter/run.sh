#!/usr/bin/env bash
#
# 라이브 플로터(백엔드 + GUI)를 띄운다. 로봇 제어와는 무관하며,
# DrumRobot 쪽 `make run` 이 이 스크립트를 별도 터미널로 띄우도록 되어 있다.
#
#   ./run.sh                        플로터 실행
#   ./run.sh --replay logs/xxx.log  기록 재생 모드
#   ./run.sh --help
#
# Ctrl+C 로 백엔드와 GUI 서버가 함께 정리된다.

set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

WS_PORT=8765
GUI_PORT=8099
IFACE=""
REPLAY=""
OPEN_BROWSER=1
KEEP_OPEN=0

usage() {
    sed -n '3,10p' "${BASH_SOURCE[0]}" | sed 's/^# \?//'
    cat <<'EOF'

옵션:
  -i, --iface a,b      CAN 인터페이스 지정 (기본: 자동 탐색 + 실행 중 추가 감지)
      --replay FILE    candump 로그 재생 모드
      --ws-port N      백엔드 WebSocket 포트 (기본 8765)
      --gui-port N     GUI 웹서버 포트 (기본 8099)
      --no-browser     브라우저 자동 실행 안 함
      --keep-open      종료 후 창을 닫지 않고 대기 (별도 터미널로 띄울 때 유용)
  -h, --help           이 도움말
EOF
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -i|--iface)   IFACE="$2"; shift 2 ;;
        --replay)     REPLAY="$2"; shift 2 ;;
        --ws-port)    WS_PORT="$2"; shift 2 ;;
        --gui-port)   GUI_PORT="$2"; shift 2 ;;
        --no-browser) OPEN_BROWSER=0; shift ;;
        --keep-open)  KEEP_OPEN=1; shift ;;
        -h|--help)    usage; exit 0 ;;
        *) echo "알 수 없는 옵션: $1"; echo; usage; exit 1 ;;
    esac
done

PIDS=()

finish() {
    if (( KEEP_OPEN )); then
        echo
        read -r -p "[run] 종료되었습니다. Enter 를 누르면 창이 닫힙니다. " _ || true
    fi
}

cleanup() {
    trap - INT TERM EXIT
    echo
    echo "[run] 정리 중..."
    for pid in "${PIDS[@]:-}"; do
        [[ -n "$pid" ]] || continue
        pkill -TERM -P "$pid" 2>/dev/null
        kill -TERM "$pid" 2>/dev/null
    done
    wait 2>/dev/null
    echo "[run] 종료"
    finish
}
trap cleanup INT TERM EXIT

# 로그 앞에 [이름] 을 붙여서 어느 프로세스가 찍은 건지 구분되게 한다
start_bg() {
    local name="$1"; shift
    { "$@" 2>&1 | while IFS= read -r line; do printf '[%s] %s\n' "$name" "$line"; done; } &
    PIDS+=("$!")
}

port_in_use() {
    ss -ltn "sport = :$1" 2>/dev/null | tail -n +2 | grep -q .
}

wait_for_port() {
    local port="$1" tries=50
    while (( tries-- > 0 )); do
        port_in_use "$port" && return 0
        sleep 0.1
    done
    return 1
}

# ---------------------------------------------------------------- 사전 점검

if ! python3 -c "import websockets" 2>/dev/null; then
    echo "[run] websockets 모듈이 없습니다. 설치:  pip3 install -r requirements.txt"
    exit 1
fi

for port in "$WS_PORT" "$GUI_PORT"; do
    if port_in_use "$port"; then
        echo "[run] 포트 $port 가 이미 사용 중입니다."
        echo "      플로터가 이미 떠 있는지 확인하거나 --ws-port/--gui-port 로 바꾸세요."
        exit 1
    fi
done

# CAN 상태 안내 (인터페이스를 올리는 건 이 도구의 역할이 아니다)
if [[ -z "$REPLAY" ]]; then
    can_list=$(ip -br link show 2>/dev/null | awk '$1 ~ /^v?can/')
    can_up=$(awk '$2=="UP"||$2=="UNKNOWN"{print $1}' <<<"$can_list" | tr '\n' ' ')
    can_down=$(awk '$2=="DOWN"{print $1}' <<<"$can_list" | tr '\n' ' ')
    [[ -n "$can_up" ]] && echo "[run] CAN 사용 가능: $can_up"
    if [[ -n "$can_down" ]]; then
        echo "[run] CAN 링크가 내려가 있음: $can_down"
        for dev in $can_down; do
            echo "      sudo ip link set $dev up type can bitrate 1000000"
        done
    fi
    if [[ -z "$can_up$can_down" ]]; then
        echo "[run] CAN 인터페이스가 아직 없습니다. 나중에 올라오면 알아서 붙습니다."
    fi
fi

# ------------------------------------------------------------------ 실행

BACKEND_ARGS=(python3 -u backend/can_bridge.py --port "$WS_PORT")
[[ -n "$IFACE" ]] && BACKEND_ARGS+=(-i "$IFACE")
[[ -n "$REPLAY" ]] && BACKEND_ARGS+=(--replay "$REPLAY")

echo "[run] 백엔드 시작 (ws://127.0.0.1:$WS_PORT)"
start_bg backend "${BACKEND_ARGS[@]}"
if ! wait_for_port "$WS_PORT"; then
    echo "[run] 백엔드가 뜨지 않았습니다. 위 [backend] 로그를 확인하세요."
    exit 1
fi

echo "[run] GUI 서버 시작 (http://localhost:$GUI_PORT)"
start_bg gui python3 -m http.server "$GUI_PORT" --bind 127.0.0.1
wait_for_port "$GUI_PORT" || true

GUI_URL="http://localhost:$GUI_PORT/index.html"
if (( OPEN_BROWSER )) && command -v xdg-open >/dev/null; then
    xdg-open "$GUI_URL" >/dev/null 2>&1 &
fi

echo
echo "  GUI : $GUI_URL"
echo "  WS  : ws://127.0.0.1:$WS_PORT"
echo
echo "[run] Ctrl+C 로 종료합니다."
wait
