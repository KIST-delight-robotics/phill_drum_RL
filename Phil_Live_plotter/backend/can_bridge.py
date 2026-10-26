#!/usr/bin/env python3
"""SocketCAN -> 디코딩 -> WebSocket 브리지.

  python3 backend/can_bridge.py                     # can* / vcan* 자동 탐색
  python3 backend/can_bridge.py -i can0,can1        # 인터페이스 지정
  python3 backend/can_bridge.py --replay logs/candump-....log   # 기록 재생

GUI(static/app.js)는 ws://localhost:8765 로 붙는다.
CAN 인터페이스를 올리고 내리는 건 이 도구가 하지 않는다 (제어 프로세스/스크립트 담당).
"""

import argparse
import asyncio
import ctypes
import json
import re
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

import websockets

import protocol
from motor_config import CONFIG_SOURCE, DEFAULT_MODE, MOTORS, motors_by_node

# linux/can.h :: struct can_frame
CAN_FRAME_FMT = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)

PROJECT_ROOT = Path(__file__).resolve().parent.parent
LOGS_DIR = PROJECT_ROOT / "logs"

# 이 시간 동안 프레임이 없으면 해당 모터를 "신호 없음"으로 본다
MOTOR_TIMEOUT_SEC = 1.0
# 브라우저로 보낼 때 묶는 간격 (제어 주기 5ms x 12모터면 초당 수천 프레임이라 batching 필수)
BROADCAST_INTERVAL_SEC = 0.02
MAX_BATCH = 500
# 인터페이스가 내려가 있을 때 다시 열어보는 주기
RECONNECT_INTERVAL_SEC = 2.0


class BridgeState:
    def __init__(self):
        self.global_mode = DEFAULT_MODE
        self.overrides = {m["id"]: "global" for m in MOTORS}
        self.last_seen = {}
        self.connected = {m["id"]: False for m in MOTORS}
        self.clients = set()
        self.log_proc = None
        self.log_file = None
        self.log_error = None
        # 인터페이스가 살아있으면 candump 로깅은 기본적으로 항상 돈다.
        # GUI에서 명시적으로 끈 경우에만 자동 재시작을 멈춘다.
        self.auto_log = True
        self.log_stopped_by_user = False
        self.log_ifaces = []      # 현재 candump 가 물고 있는 인터페이스
        # 브링업 진단용: 버스에 흐르지만 config/motors.json 어느 모터에도 안 붙는 프레임
        self.interfaces = []      # [(이름, 링크 상태)]
        self.link_ok = {}         # 인터페이스별 실제 수신 가능 여부
        self.frames_seen = 0
        self.frames_decoded = 0
        self.unmapped = {}        # "can0 0x123" -> 개수

    def mode_of(self, motor):
        if motor["type"] == "Maxon":
            return "maxon"
        override = self.overrides.get(motor["id"], "global")
        return self.global_mode if override == "global" else override


# ------------------------------------------------------------------ 프레임 처리


def format_can_id(can_id_raw):
    is_eff = bool(can_id_raw & protocol.CAN_EFF_FLAG)
    can_id = can_id_raw & (protocol.CAN_EFF_MASK if is_eff else protocol.CAN_SFF_MASK)
    return ("0x%08X" % can_id) if is_eff else ("0x%03X" % can_id)


def build_frame_message(can_id_raw, data, ifname, ts, state, node_map):
    state.frames_seen += 1
    result = protocol.classify(can_id_raw, data, node_map, state.mode_of)
    if result is None:
        # 어느 모터에도 매핑이 안 된 프레임. nodeId 설정이 틀렸거나 모드가 안 맞으면
        # 여기로 다 빠지므로, 무엇이 흐르고 있는지 보여줘야 원인을 찾을 수 있다.
        key = "%s %s" % (ifname, format_can_id(can_id_raw))
        state.unmapped[key] = state.unmapped.get(key, 0) + 1
        return None

    motor, direction, decoded = result
    state.frames_decoded += 1
    is_eff = bool(can_id_raw & protocol.CAN_EFF_FLAG)
    can_id = can_id_raw & (protocol.CAN_EFF_MASK if is_eff else protocol.CAN_SFF_MASK)
    state.last_seen[motor["id"]] = ts

    return {
        "type": "frame",
        "motor": motor["id"],
        "dir": direction,
        "mode": state.mode_of(motor),
        "ts": ts,
        "iface": ifname,
        "decoded": decoded,
        "raw": {
            "id": ("0x%08X" % can_id) if is_eff else ("0x%03X" % can_id),
            "dlc": len(data),
            "data": list(data),
        },
    }


def enqueue(queue, msg):
    try:
        queue.put_nowait(msg)
    except asyncio.QueueFull:
        pass  # 소비가 밀리면 최신 프레임을 버린다 (플로팅용이라 무방)


# ------------------------------------------------------------------ CAN 수신


def can_reader_thread(ifname, state, loop, queue, stop_event):
    """인터페이스가 내려가 있어도 죽지 않고, 올라올 때까지 계속 재시도한다.

    플로터를 먼저 띄우고 나중에 로봇/CAN 을 올리는 순서가 흔하기 때문에,
    한 번 실패했다고 스레드를 끝내면 재시작해야 하는 불편이 생긴다.
    """
    node_map = motors_by_node()
    receiving = False   # 현재 정상 수신 중인지 (로그 도배 방지용)

    while not stop_event.is_set():
        sock = None
        try:
            sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            sock.bind((ifname,))
            sock.settimeout(0.5)
        except OSError as err:
            if receiving or state.link_ok.get(ifname) is None:
                print("[can] %s 열기 실패(%s). 올라올 때까지 재시도합니다." % (ifname, err))
            receiving = False
            state.link_ok[ifname] = False
            if sock:
                sock.close()
            stop_event.wait(RECONNECT_INTERVAL_SEC)
            continue

        try:
            while not stop_event.is_set():
                try:
                    raw = sock.recv(CAN_FRAME_SIZE)
                except socket.timeout:
                    # 링크가 내려가 있으면 timeout 이 아니라 ENETDOWN 이 난다.
                    # 즉 timeout 은 "링크는 살아있고 트래픽만 없음" 이라는 뜻.
                    if not receiving:
                        print("[can] %s 수신 시작 (아직 트래픽 없음)" % ifname)
                        receiving = True
                        state.link_ok[ifname] = True
                    continue

                if not receiving:
                    print("[can] %s 수신 시작" % ifname)
                    receiving = True
                    state.link_ok[ifname] = True

                if len(raw) < CAN_FRAME_SIZE:
                    continue

                can_id, dlc, payload = struct.unpack(CAN_FRAME_FMT, raw)
                msg = build_frame_message(can_id, payload[:dlc], ifname, time.time(), state, node_map)
                if msg:
                    loop.call_soon_threadsafe(enqueue, queue, msg)
        except OSError as err:
            if receiving or state.link_ok.get(ifname) is None:
                print("[can] %s 수신 불가(%s). 올라올 때까지 재시도합니다." % (ifname, err))
            receiving = False
            state.link_ok[ifname] = False
            stop_event.wait(RECONNECT_INTERVAL_SEC)
        finally:
            sock.close()

    print("[can] %s 수신 종료" % ifname)


# ------------------------------------------------------------------ 로그 재생

LOG_LINE_RE = re.compile(r"^\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)#([0-9A-Fa-f]*)")


def replay_thread(path, speed, state, loop, queue, stop_event):
    node_map = motors_by_node()
    with open(path) as f:
        lines = [line.strip() for line in f if line.strip()]
    print("[replay] %s (%d줄) 재생 시작" % (path, len(lines)))

    first_ts = None
    started = time.time()

    for line in lines:
        if stop_event.is_set():
            return
        match = LOG_LINE_RE.match(line)
        if not match:
            continue

        ts = float(match.group(1))
        ifname = match.group(2)
        id_hex = match.group(3)
        data = bytes.fromhex(match.group(4))

        if first_ts is None:
            first_ts = ts
        if speed > 0:
            wait = (ts - first_ts) / speed - (time.time() - started)
            if wait > 0:
                time.sleep(min(wait, 1.0))

        can_id = int(id_hex, 16)
        if len(id_hex) > 3:  # candump 는 확장 ID 를 8자리로 쓴다
            can_id |= protocol.CAN_EFF_FLAG

        msg = build_frame_message(can_id, data, ifname, time.time(), state, node_map)
        if msg:
            loop.call_soon_threadsafe(enqueue, queue, msg)

    print("[replay] 재생 완료")


# --------------------------------------------------- 로그 파일 전체 구간 디코딩

SERIES_KEYS = ("posCmd", "posFb", "velCmd", "velFb", "torCmd", "torFb")


class BucketReducer:
    """긴 로그를 그래프에 올릴 수 있게 줄인다.

    구간(bucket)마다 최솟값/최댓값 두 점만 남기므로 점 개수는 줄어도
    타격 순간의 스파이크 같은 건 뭉개지지 않는다.
    """

    def __init__(self, t0, width):
        self.t0 = t0
        self.width = width
        self.bucket = None
        self.lo = None
        self.hi = None
        self.points = []

    def add(self, t, y):
        bucket = int((t - self.t0) / self.width)
        if self.bucket is None:
            self.bucket = bucket
        elif bucket != self.bucket:
            self.flush()
            self.bucket = bucket
        if self.lo is None or y < self.lo[1]:
            self.lo = (t, y)
        if self.hi is None or y > self.hi[1]:
            self.hi = (t, y)

    def flush(self):
        if self.lo is None:
            return
        pair = [self.lo] if self.lo == self.hi else sorted([self.lo, self.hi])
        self.points.extend([[round(t, 4), round(y, 6)] for t, y in pair])
        self.lo = None
        self.hi = None


def log_time_range(path):
    """앞뒤 줄만 읽어서 로그의 시작/끝 시각을 구한다 (파일 전체를 안 읽는다)."""
    first = last = None
    with open(path, "rb") as f:
        for raw in f:
            match = LOG_LINE_RE.match(raw.decode("utf-8", "ignore").strip())
            if match:
                first = float(match.group(1))
                break
        f.seek(0, 2)
        size = f.tell()
        f.seek(max(0, size - 65536))
        tail = f.read().decode("utf-8", "ignore").splitlines()
        for raw in reversed(tail):
            match = LOG_LINE_RE.match(raw.strip())
            if match:
                last = float(match.group(1))
                break
    return first, last


def decode_log_file(path, state, buckets):
    """candump -l 로그 전체를 디코딩해서 모터별/필드별 시계열로 만든다."""
    t_start, t_end = log_time_range(path)
    if t_start is None:
        return {"type": "file_error", "message": "읽을 수 있는 CAN 프레임이 없습니다"}

    span = max(t_end - t_start, 1e-6)
    width = span / max(buckets, 1)

    node_map = motors_by_node()
    reducers = {m["id"]: {key: BucketReducer(t_start, width) for key in SERIES_KEYS} for m in MOTORS}
    stats = {m["id"]: {"tx": 0, "rx": 0} for m in MOTORS}
    total = 0
    decoded = 0
    ifaces = set()

    with open(path, encoding="utf-8", errors="ignore") as f:
        for line in f:
            match = LOG_LINE_RE.match(line.strip())
            if not match:
                continue
            total += 1
            ts = float(match.group(1))
            ifaces.add(match.group(2))
            id_hex = match.group(3)
            try:
                data = bytes.fromhex(match.group(4))
            except ValueError:
                continue

            can_id = int(id_hex, 16)
            if len(id_hex) > 3:
                can_id |= protocol.CAN_EFF_FLAG

            result = protocol.classify(can_id, data, node_map, state.mode_of)
            if result is None:
                continue

            motor, direction, values = result
            decoded += 1
            stats[motor["id"]]["tx" if direction == "tx" else "rx"] += 1

            suffix = "Cmd" if direction == "tx" else "Fb"
            bucket_set = reducers[motor["id"]]
            for field, key in (("pos", "pos"), ("vel", "vel"), ("tor", "tor")):
                value = values.get(field)
                if value is not None:
                    bucket_set[key + suffix].add(ts, value)

    series = {}
    for motor_id, bucket_set in reducers.items():
        motor_series = {}
        for key, reducer in bucket_set.items():
            reducer.flush()
            if reducer.points:
                motor_series[key] = reducer.points
        if motor_series:
            series[motor_id] = motor_series

    return {
        "type": "file_series",
        "file": str(Path(path).relative_to(PROJECT_ROOT)) if str(path).startswith(str(PROJECT_ROOT)) else str(path),
        "tStart": t_start,
        "tEnd": t_end,
        "totalLines": total,
        "decodedFrames": decoded,
        "interfaces": sorted(ifaces),
        "stats": stats,
        "series": series,
    }


def list_log_files():
    if not LOGS_DIR.exists():
        return []
    files = []
    for path in sorted(LOGS_DIR.glob("*.log"), reverse=True):
        stat = path.stat()
        files.append({
            "path": str(path.relative_to(PROJECT_ROOT)),
            "name": path.name,
            "size": stat.st_size,
            "mtime": stat.st_mtime,
        })
    return files


def resolve_log_path(raw_path):
    """프로젝트 폴더 밖은 열지 않는다 (WS 가 네트워크에 열려 있을 수 있으므로)."""
    candidate = Path(raw_path)
    if not candidate.is_absolute():
        candidate = PROJECT_ROOT / candidate
    candidate = candidate.resolve()
    if PROJECT_ROOT not in candidate.parents and candidate != PROJECT_ROOT:
        return None
    if not candidate.is_file():
        return None
    return candidate


# --------------------------------------------------------------- candump 로깅


_PR_SET_PDEATHSIG = 1


def _die_with_parent():
    """부모(백엔드)가 죽으면 candump 도 같이 죽게 한다.

    이게 없으면 백엔드를 SIGKILL 로 죽였을 때 candump 가 고아로 남아
    재시작할 때마다 프로세스와 로그 파일이 겹쳐서 쌓인다.

    SIGTERM 이 아니라 SIGKILL 인 이유: 버스에 트래픽이 없으면 candump 가
    select() 에 블록된 채 종료 플래그를 확인하지 못해서 SIGTERM 을 무시한다.
    (트래픽이 있으면 정상 종료되므로, 평상시 종료는 stop_logging 의
     SIGTERM -> 대기 -> SIGKILL 순서를 그대로 탄다.)
    """
    try:
        ctypes.CDLL("libc.so.6", use_errno=True).prctl(_PR_SET_PDEATHSIG, signal.SIGKILL)
    except Exception:
        pass


def cleanup_empty_logs():
    """이전 실행이 남긴 빈 로그를 지운다.

    candump 는 트래픽이 없어도 파일부터 만들기 때문에, 자동 로깅을 켜두면
    백엔드를 재시작할 때마다 0바이트 파일이 쌓인다. 지금 기록 중인 파일을
    건드리지 않도록 10분 이상 지난 것만 지운다.
    """
    if not LOGS_DIR.exists():
        return
    cutoff = time.time() - 600
    removed = 0
    for path in LOGS_DIR.glob("candump-*.log"):
        try:
            stat = path.stat()
            if stat.st_size == 0 and stat.st_mtime < cutoff:
                path.unlink()
                removed += 1
        except OSError:
            pass
    if removed:
        print("[log] 비어 있는 예전 로그 %d개를 정리했습니다" % removed)


def loggable_interfaces(state, interfaces):
    """candump 에 넘길 수 있는 인터페이스만 고른다.

    없는 인터페이스가 하나라도 섞이면 candump 가 통째로 실패한다.
    """
    usable = []
    for name in interfaces:
        ok = state.link_ok.get(name)
        if ok is False:
            continue
        if ok is None:
            # 아직 수신 스레드가 판정 전 - 시작 시점 링크 상태로 판단
            startup = dict(state.interfaces).get(name)
            if startup in ("없음", "DOWN"):
                continue
        usable.append(name)
    return usable


def start_logging(state, interfaces):
    if state.log_proc and state.log_proc.poll() is None:
        return

    targets = loggable_interfaces(state, interfaces)
    if not targets:
        state.log_error = "로깅할 수 있는 CAN 인터페이스가 없습니다"
        return

    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    before = set(LOGS_DIR.glob("candump-*.log"))

    try:
        state.log_proc = subprocess.Popen(
            ["candump", "-l"] + targets,
            cwd=str(LOGS_DIR),
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            preexec_fn=_die_with_parent,
        )
    except FileNotFoundError:
        state.log_proc = None
        state.log_error = "candump 를 찾을 수 없습니다 (can-utils 설치 필요)"
        return

    state.log_error = None
    state.log_file = None
    state.log_ifaces = list(targets)
    for _ in range(10):  # candump 가 파일을 만들 때까지 잠깐 기다린다
        created = set(LOGS_DIR.glob("candump-*.log")) - before
        if created:
            newest = sorted(created)[-1]
            state.log_file = str(newest.relative_to(PROJECT_ROOT))
            break
        time.sleep(0.02)

    print("[log] candump 시작: %s (%s)" % (state.log_file or "(파일명 확인 실패)", " ".join(targets)))


def stop_logging(state):
    if state.log_proc and state.log_proc.poll() is None:
        state.log_proc.terminate()
        try:
            # 트래픽이 있으면 SIGTERM 으로 바로 정리되고,
            # 없으면 (버퍼에 남은 것도 없으므로) 강제 종료해도 잃을 게 없다
            state.log_proc.wait(timeout=1)
        except subprocess.TimeoutExpired:
            state.log_proc.kill()
            state.log_proc.wait(timeout=2)
        print("[log] candump 중지: %s" % state.log_file)
    state.log_proc = None


async def log_supervisor(state, interfaces):
    """인터페이스가 살아있는 한 candump 로깅이 계속 돌아가도록 지킨다.

    - 시작할 때 자동으로 켠다
    - CAN 이 나중에 올라와도 그때 켠다
    - candump 가 죽으면(링크 끊김, USB 재연결 등) 다시 띄운다
    - 단, 사용자가 GUI에서 명시적으로 끈 경우에는 건드리지 않는다
    """
    cleanup_empty_logs()
    while True:
        if state.auto_log and not state.log_stopped_by_user:
            running = bool(state.log_proc and state.log_proc.poll() is None)

            # 새 인터페이스가 붙었으면 그것도 기록에 포함시킨다 (파일은 새로 시작)
            if running:
                added = set(loggable_interfaces(state, interfaces)) - set(state.log_ifaces)
                if added:
                    print("[log] %s 가 추가되어 로깅을 다시 시작합니다" % " ".join(sorted(added)))
                    stop_logging(state)
                    running = False

            if not running:
                had_file = state.log_file
                start_logging(state, interfaces)
                if state.log_proc and had_file:
                    print("[log] candump 가 중단되어 새 파일로 다시 시작했습니다")
                if state.log_proc:
                    await send_all(state, json.dumps(log_status_message(state)))
        await asyncio.sleep(3.0)


def log_status_message(state):
    running = bool(state.log_proc and state.log_proc.poll() is None)
    return {
        "type": "log_status",
        "logging": running,
        "file": state.log_file if running else None,
        "error": state.log_error,
    }


# ------------------------------------------------------------------ WebSocket


async def send_all(state, payload):
    if not state.clients:
        return
    dead = []
    for client in list(state.clients):
        try:
            await client.send(payload)
        except Exception:
            dead.append(client)
    for client in dead:
        state.clients.discard(client)


async def broadcaster(queue, state):
    while True:
        items = [await queue.get()]
        await asyncio.sleep(BROADCAST_INTERVAL_SEC)
        while len(items) < MAX_BATCH:
            try:
                items.append(queue.get_nowait())
            except asyncio.QueueEmpty:
                break
        if not state.clients:
            continue
        payload = json.dumps(items[0] if len(items) == 1 else {"type": "batch", "items": items})
        await send_all(state, payload)


async def watchdog(state):
    while True:
        await asyncio.sleep(0.5)
        now = time.time()
        for motor in MOTORS:
            alive = (now - state.last_seen.get(motor["id"], 0)) < MOTOR_TIMEOUT_SEC
            if alive != state.connected[motor["id"]]:
                state.connected[motor["id"]] = alive
                await send_all(state, json.dumps({
                    "type": "status", "motor": motor["id"], "connected": alive,
                }))


async def handle_load_log(websocket, state, msg):
    path = resolve_log_path(msg.get("path", ""))
    if path is None:
        await websocket.send(json.dumps({
            "type": "file_error",
            "message": "파일을 찾을 수 없거나 프로젝트 폴더 밖입니다: %s" % msg.get("path", ""),
        }))
        return

    buckets = int(msg.get("buckets") or 800)
    print("[file] 디코딩 시작: %s (buckets=%d)" % (path.name, buckets))
    started = time.time()
    loop = asyncio.get_event_loop()
    try:
        # 파일이 크면 오래 걸리므로 이벤트 루프를 막지 않게 별도 스레드에서 처리
        result = await loop.run_in_executor(None, decode_log_file, str(path), state, buckets)
    except Exception as err:  # 잘못된 로그 형식 등
        await websocket.send(json.dumps({"type": "file_error", "message": "디코딩 실패: %s" % err}))
        return

    print("[file] 완료: %d줄 중 %d프레임 디코딩, %.1fs" % (
        result.get("totalLines", 0), result.get("decodedFrames", 0), time.time() - started))
    await websocket.send(json.dumps(result))


async def ws_handler(websocket, state, interfaces):
    state.clients.add(websocket)
    print("[ws] 클라이언트 접속 (총 %d)" % len(state.clients))
    try:
        # 모터 목록은 로봇 설정에서 읽어오므로(웹 루트 밖일 수 있다) GUI 에 직접 내려준다
        await websocket.send(json.dumps({
            "type": "config",
            "motors": [
                {k: v for k, v in motor.items() if k in ("id", "label", "nodeId", "type", "motorType")}
                for motor in MOTORS
            ],
            "defaultMode": DEFAULT_MODE,
            "source": CONFIG_SOURCE,
        }))
        await websocket.send(json.dumps(log_status_message(state)))
        await websocket.send(json.dumps({"type": "log_list", "files": list_log_files()}))
        await websocket.send(json.dumps(bus_info_message(state)))
        for motor in MOTORS:
            await websocket.send(json.dumps({
                "type": "status", "motor": motor["id"], "connected": state.connected[motor["id"]],
            }))

        async for raw in websocket:
            try:
                msg = json.loads(raw)
            except ValueError:
                continue
            if msg.get("type") != "control":
                continue

            action = msg.get("action")
            if action == "set_modes":
                state.global_mode = msg.get("globalMode", state.global_mode)
                for motor_id, mode in (msg.get("overrides") or {}).items():
                    if motor_id in state.overrides and mode in ("global", "servo", "mit"):
                        state.overrides[motor_id] = mode
                print("[mode] global=%s overrides=%s" % (
                    state.global_mode,
                    {k: v for k, v in state.overrides.items() if v != "global"},
                ))
            elif action == "start_logging":
                state.log_stopped_by_user = False
                start_logging(state, interfaces)
                await send_all(state, json.dumps(log_status_message(state)))
            elif action == "stop_logging":
                # 사용자가 직접 껐으면 감시 태스크가 다시 켜지 않는다
                state.log_stopped_by_user = True
                stop_logging(state)
                await send_all(state, json.dumps(log_status_message(state)))
            elif action == "list_logs":
                await websocket.send(json.dumps({"type": "log_list", "files": list_log_files()}))
            elif action == "load_log":
                await handle_load_log(websocket, state, msg)
    except websockets.ConnectionClosed:
        pass
    finally:
        state.clients.discard(websocket)
        print("[ws] 클라이언트 종료 (총 %d)" % len(state.clients))


# ----------------------------------------------------------------------- main


def discover_can_interfaces():
    """[(이름, 링크 상태)] 목록. vcan 은 보통 UNKNOWN 으로 뜨며 정상 동작한다."""
    try:
        out = subprocess.check_output(["ip", "-o", "link", "show"], universal_newlines=True)
    except Exception:
        return []

    found = []
    for line in out.splitlines():
        match = re.match(r"^\d+:\s+([^:@\s]+)", line)
        if not match:
            continue
        name = match.group(1)
        if not (name.startswith("can") or name.startswith("vcan")):
            continue
        state_match = re.search(r"\sstate\s+(\S+)", line)
        found.append((name, state_match.group(1) if state_match else "UNKNOWN"))
    return found


def bus_info_message(state):
    top = sorted(state.unmapped.items(), key=lambda kv: -kv[1])[:8]

    interfaces = []
    for name, startup_state in state.interfaces:
        ok = state.link_ok.get(name)
        if ok is True:
            shown = "UP"
        elif ok is False:
            shown = "DOWN"
        else:
            shown = startup_state
        interfaces.append({"name": name, "state": shown})

    return {
        "type": "bus_info",
        "interfaces": interfaces,
        "framesSeen": state.frames_seen,
        "framesDecoded": state.frames_decoded,
        "unmapped": [{"id": k, "count": v} for k, v in top],
    }


async def interface_scanner(state, interfaces, loop, queue, stop_event, threads):
    """실행 중에 새로 생긴 CAN 인터페이스를 잡아서 수신을 시작한다.

    인터페이스 4개 중 일부만 연결해서 테스트하거나, USB 어댑터를 나중에 꽂는
    경우가 있어서 시작 시점의 목록만 보고 있으면 놓친다.
    (-i 로 직접 지정했을 때는 그 목록만 쓴다)
    """
    while True:
        await asyncio.sleep(5.0)
        found = discover_can_interfaces()
        known = {name for name, _ in state.interfaces}

        for name, link_state in found:
            if name in known:
                continue
            print("[can] %s 가 새로 생겼습니다 (state=%s). 수신을 시작합니다." % (name, link_state))
            state.interfaces.append((name, link_state))
            interfaces.append(name)
            thread = threading.Thread(
                target=can_reader_thread,
                args=(name, state, loop, queue, stop_event),
                daemon=True,
            )
            thread.start()
            threads.append(thread)


async def bus_reporter(state):
    """어떤 인터페이스에 뭐가 흐르는지, 매핑 안 되는 ID 가 있는지 주기적으로 알린다."""
    warned = False
    while True:
        await asyncio.sleep(5.0)

        if state.frames_seen and not state.frames_decoded and not warned:
            top = sorted(state.unmapped.items(), key=lambda kv: -kv[1])[:8]
            print("[진단] 프레임 %d개를 받았지만 하나도 매핑되지 않았습니다. "
                  "config/motors.json 의 nodeId 나 GUI 모드(Servo/MIT)를 확인하세요." % state.frames_seen)
            print("       미매핑 상위: %s" % ", ".join("%s x%d" % (k, v) for k, v in top))
            warned = True

        await send_all(state, json.dumps(bus_info_message(state)))


async def run(args):
    state = BridgeState()
    state.auto_log = not args.no_auto_log
    loop = asyncio.get_event_loop()
    queue = asyncio.Queue(maxsize=20000)
    stop_event = threading.Event()

    discovered = discover_can_interfaces()
    states = dict(discovered)
    if args.interfaces:
        interfaces = [s.strip() for s in args.interfaces.split(",") if s.strip()]
    else:
        interfaces = [name for name, _ in discovered]
    state.interfaces = [(name, states.get(name, "없음")) for name in interfaces]

    print("[config] 모터 설정: %s (CAN 모터 %d개)" % (CONFIG_SOURCE, len(MOTORS)))

    for name, link_state in state.interfaces:
        if link_state == "없음":
            print("[can] %s : 인터페이스가 존재하지 않습니다" % name)
        elif link_state == "DOWN":
            print("[can] %s : 링크가 DOWN 입니다. 프레임이 들어오지 않습니다 "
                  "(제어 프로세스나 ip link set %s up 으로 올려주세요)" % (name, name))
        else:
            print("[can] %s : state=%s" % (name, link_state))

    threads = []
    if args.replay:
        threads.append(threading.Thread(
            target=replay_thread,
            args=(args.replay, args.replay_speed, state, loop, queue, stop_event),
            daemon=True,
        ))
    else:
        if not interfaces:
            print("[can] CAN 인터페이스를 찾지 못했습니다. --replay 로 기록을 재생하거나 인터페이스를 올려주세요.")
        for ifname in interfaces:
            threads.append(threading.Thread(
                target=can_reader_thread,
                args=(ifname, state, loop, queue, stop_event),
                daemon=True,
            ))

    for thread in threads:
        thread.start()

    # SIGTERM 으로 죽을 때도 candump 정리가 되도록 (기본 동작은 finally 를 안 탄다)
    stop_signal = asyncio.Event()
    for sig in (signal.SIGTERM, signal.SIGINT):
        try:
            loop.add_signal_handler(sig, stop_signal.set)
        except (NotImplementedError, RuntimeError):
            pass

    async def wait_for_signal():
        await stop_signal.wait()
        print("\n[run] 종료 신호를 받았습니다")

    async with websockets.serve(lambda ws: ws_handler(ws, state, interfaces), args.host, args.port):
        print("[ws] ws://%s:%d 대기 중" % (args.host, args.port))
        try:
            tasks = [
                asyncio.ensure_future(broadcaster(queue, state)),
                asyncio.ensure_future(watchdog(state)),
                asyncio.ensure_future(bus_reporter(state)),
                asyncio.ensure_future(wait_for_signal()),
            ]
            if not args.replay:
                tasks.append(asyncio.ensure_future(log_supervisor(state, interfaces)))
                if not args.interfaces:
                    tasks.append(asyncio.ensure_future(
                        interface_scanner(state, interfaces, loop, queue, stop_event, threads)))

            # 아무거나 하나 끝나면(= 종료 신호) 나머지를 정리한다
            await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
            for task in tasks:
                task.cancel()
        finally:
            stop_event.set()
            stop_logging(state)


def main():
    parser = argparse.ArgumentParser(description="Drum Robot live CAN plotter backend")
    parser.add_argument("-i", "--interfaces", default="", help="쉼표로 구분한 CAN 인터페이스 (기본: 자동 탐색)")
    parser.add_argument("--host", default="127.0.0.1", help="다른 PC에서 붙으려면 0.0.0.0")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-auto-log", action="store_true",
                        help="CAN 이 살아있을 때 candump 로깅을 자동으로 켜지 않음")
    parser.add_argument("--replay", help="candump -l 로그 파일 재생")
    parser.add_argument("--replay-speed", type=float, default=1.0, help="재생 배속 (0 이면 최대 속도)")
    args = parser.parse_args()

    try:
        asyncio.get_event_loop().run_until_complete(run(args))
    except KeyboardInterrupt:
        print("\n종료")


if __name__ == "__main__":
    main()
