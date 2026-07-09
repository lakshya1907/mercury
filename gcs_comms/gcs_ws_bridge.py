#!/usr/bin/env python3
"""
gcs_ws_bridge.py  —  Telemetry → GUI bridge
Receives all rover UDP/TCP data and serves it to the Mercury React GUI via:
  • WebSocket  :9090  (JSON state at 10 Hz, consumed by useRobot.js)
  • HTTP MJPEG :8080  (/lane.mjpg, /turret.mjpg — camera streams)
  • HTTP tiles :8000  (/{z}/{x}/{y}.png — offline map tiles)

Also listens for commands sent from the GUI (joystick, E-STOP) and can
optionally forward them to the rover.

Usage:
    pip install websockets msgpack
    python3 gcs_ws_bridge.py

    # Forward joystick / E-STOP back to rover:
    python3 gcs_ws_bridge.py --rover-ip 192.168.88.10

    # Custom log / tile directories:
    python3 gcs_ws_bridge.py --log-dir ~/robot_logs --tile-dir ./tiles
"""

import argparse
import asyncio
import json
import logging
import math
import os
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from queue import Empty, Queue

import msgpack
import websockets

# ─── Logging ─────────────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
)
log = logging.getLogger("gcs_ws_bridge")

# ─── Ports (must match ugv_gcs_bridge.py) ────────────────────────────────────
UDP_IMU_PORT     = 5000
UDP_GPS_PORT     = 5001
UDP_ODOM_PORT    = 5002
UDP_ENC_PORT     = 5003
UDP_CMDVEL_PORT  = 5004
UDP_SYS_PORT     = 5005
UDP_VIDEO_PORT   = 5600      # lane / main camera
UDP_TURRET_PORT  = 5601      # turret camera

TCP_CMD_PORT     = 6000      # rover → GCS (HB, MODE, ALERT, ESTOP)
UDP_ROVER_CMD_PORT = 5700    # GCS → rover (cmd_vel, ESTOP), if --rover-ip set

# ─── GUI server ports ─────────────────────────────────────────────────────────
WS_PORT    = 9090
MJPEG_PORT = 8080
TILE_PORT  = 8000

CHUNK_SIZE = 60_000          # video chunk size (must match bridge)


# ─── Shared mutable state ─────────────────────────────────────────────────────
# All threads read/write this dict.  Python's GIL makes dict-level updates
# safe enough; we also use a lock when building the snapshot for WS broadcast.

_lock = threading.Lock()

_state = {
    "mode": "MANUAL",
    "drive":    {"vx": 0, "wz": 0, "rover_vx": None, "rover_wz": None, "ts": 0, "rover_ts": 0},
    "imu":      {"roll": None, "pitch": None, "yaw": None,
                 "ax": None, "ay": None, "az": None,
                 "wx": None, "wy": None, "wz": None, "ts": 0},
    "gps":      {"lat": None, "lon": None, "alt": None, "fix": "NO DATA", "ts": 0},
    "odom":     {"x": None, "y": None, "yaw": None, "vx": None, "wz": None, "ts": 0},
    "encoders": {"names": [], "position": [], "velocity": [], "ts": 0},
    "lidar":    {"front": None, "min": None, "mean": None, "max": None, "ts": 0},
    "nav":      {"goal_x": None, "goal_y": None, "status": "NO DATA", "ts": 0},
    "system":   {"cpu_pct": None, "mem_pct": None, "mem_used_mb": None, "ts": 0},
    "lane":     {"error_px": None, "visible": False, "both_visible": False,
                 "drift": "NO DATA", "ts": 0},
    "face":     {"active": False, "match": False, "h_error_px": None,
                 "v_error_px": None, "complete": False, "ts": 0},
    "mission":  {"wp_name": "NO DATA", "wp_idx": "NO DATA", "wp_dist": None,
                 "all_done": False, "ts": 0},
    "health":   {"all_ok": None, "missing": [], "ts": 0},
    "alerts":   [],
    "rx":       {"imu": 0, "gps": 0, "odom": 0, "enc": 0, "lane": 0,
                 "face": 0, "mission": 0, "alert": 0, "video": 0},
    "video":    {"lane_ts": 0, "turret_ts": 0},
    "ages":     {},
    "waypoints": [],
    "last_hb":  0.0,
}

# Last-seen timestamps for age computation
_last_ts: dict[str, float] = {}


def _ts():
    return time.time()


def _set(key: str, val: dict):
    """Merge val into _state[key] and update _last_ts."""
    with _lock:
        _state[key] = {**_state[key], **val}
        _last_ts[key] = _ts()


def _inc_rx(key: str):
    with _lock:
        _state["rx"][key] = _state["rx"].get(key, 0) + 1


def _add_alert(msg: str, atype: str = "info"):
    entry = {
        "id":   f"{_ts():.6f}-{threading.get_ident()}",
        "msg":  msg,
        "type": atype,
        "time": time.strftime("%H:%M:%S"),
    }
    with _lock:
        _state["alerts"] = [entry] + _state["alerts"][:29]


def _snapshot():
    """Return a copy of state with computed ages."""
    now = _ts()
    with _lock:
        snap = json.loads(json.dumps(_state))   # deep-copy via JSON
    ages = {k: round(now - t, 2) for k, t in _last_ts.items()}
    snap["ages"] = ages
    return snap


# ─── Quaternion → Euler helper ────────────────────────────────────────────────
def _quat_to_euler(ox, oy, oz, ow):
    """Returns (roll_deg, pitch_deg, yaw_deg)."""
    try:
        sinr = 2.0 * (ow * ox + oy * oz)
        cosr = 1.0 - 2.0 * (ox * ox + oy * oy)
        roll = math.degrees(math.atan2(sinr, cosr))

        sinp = 2.0 * (ow * oy - oz * ox)
        pitch = math.degrees(math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp))

        siny = 2.0 * (ow * oz + ox * oy)
        cosy = 1.0 - 2.0 * (oy * oy + oz * oz)
        yaw = math.degrees(math.atan2(siny, cosy))

        return roll, pitch, yaw
    except Exception:
        return None, None, None


# ─── UDP telemetry receivers ──────────────────────────────────────────────────
def _recv_udp_imu():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", UDP_IMU_PORT))
    log.info("IMU  listening on UDP :%d", UDP_IMU_PORT)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            p = msgpack.unpackb(data, raw=False)
            roll, pitch, yaw = _quat_to_euler(
                p.get("ox", 0), p.get("oy", 0), p.get("oz", 0), p.get("ow", 1))
            _set("imu", {
                "roll": roll, "pitch": pitch, "yaw": yaw,
                "ax": p.get("ax"), "ay": p.get("ay"), "az": p.get("az"),
                "wx": p.get("wx"), "wy": p.get("wy"), "wz": p.get("wz"),
                "ts": p.get("_t", _ts()),
            })
            _inc_rx("imu")
        except Exception as e:
            log.debug("IMU recv: %s", e)


def _recv_udp_gps():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", UDP_GPS_PORT))
    log.info("GPS  listening on UDP :%d", UDP_GPS_PORT)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            p = msgpack.unpackb(data, raw=False)
            lat, lon = p.get("lat"), p.get("lon")
            fix = "GPS FIX" if (lat is not None and lon is not None) else "NO FIX"
            _set("gps", {
                "lat": lat, "lon": lon, "alt": p.get("alt"), "fix": fix,
                "ts": p.get("_t", _ts()),
            })
            _inc_rx("gps")
        except Exception as e:
            log.debug("GPS recv: %s", e)


def _recv_udp_odom():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", UDP_ODOM_PORT))
    log.info("ODOM listening on UDP :%d", UDP_ODOM_PORT)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            p = msgpack.unpackb(data, raw=False)
            _set("odom", {
                "x": p.get("x"), "y": p.get("y"),
                "vx": p.get("vx"), "wz": p.get("wz"),
                "ts": p.get("_t", _ts()),
            })
            # rover_vx/wz also tracked from odom (actual rover speed)
            with _lock:
                _state["drive"]["rover_vx"] = p.get("vx")
                _state["drive"]["rover_wz"] = p.get("wz")
                _state["drive"]["rover_ts"] = p.get("_t", _ts())
            _inc_rx("odom")
        except Exception as e:
            log.debug("ODOM recv: %s", e)


def _recv_udp_enc():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", UDP_ENC_PORT))
    log.info("ENC  listening on UDP :%d", UDP_ENC_PORT)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            p = msgpack.unpackb(data, raw=False)
            _set("encoders", {
                "names":    p.get("names", []),
                "position": p.get("position", []),
                "velocity": p.get("velocity", []),
                "ts":       p.get("_t", _ts()),
            })
            _inc_rx("enc")
        except Exception as e:
            log.debug("ENC recv: %s", e)


def _recv_udp_cmdvel():
    """Rover's actual executed cmd_vel (echo back from rover side)."""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", UDP_CMDVEL_PORT))
    log.info("CMDV listening on UDP :%d", UDP_CMDVEL_PORT)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            p = msgpack.unpackb(data, raw=False)
            with _lock:
                _state["drive"]["rover_vx"] = p.get("linear")
                _state["drive"]["rover_wz"] = p.get("angular")
                _state["drive"]["rover_ts"] = p.get("_t", _ts())
        except Exception as e:
            log.debug("CMDV recv: %s", e)


def _recv_udp_sys():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", UDP_SYS_PORT))
    log.info("SYS  listening on UDP :%d", UDP_SYS_PORT)
    while True:
        try:
            data, _ = sock.recvfrom(65535)
            p = msgpack.unpackb(data, raw=False)
            # status is a string message; system metrics come separately
            status_str = str(p.get("status", ""))
            with _lock:
                _state["system"]["ts"] = p.get("_t", _ts())
            _last_ts["system"] = _ts()
        except Exception as e:
            log.debug("SYS recv: %s", e)


# ─── Video UDP receivers ──────────────────────────────────────────────────────
class _VideoReassembler:
    """Receives chunked JPEG UDP packets, reassembles frames into a queue."""
    def __init__(self, port: int, label: str, frame_q: Queue):
        self._port = port
        self._label = label
        self._q = frame_q
        self._frames: dict = {}
        self._order: list = []

    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 4 * 1024 * 1024)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", self._port))
        log.info("VID  %s listening on UDP :%d", self._label, self._port)

        while True:
            try:
                data, _ = sock.recvfrom(65535)
                if len(data) < 8:
                    continue
                frame_id, chunk_idx, total = struct.unpack(">IHH", data[:8])
                chunk = data[8:]
                if total == 0 or chunk_idx >= total:
                    continue

                if frame_id not in self._frames:
                    self._frames[frame_id] = {}
                    self._order.append(frame_id)
                self._frames[frame_id][chunk_idx] = chunk

                # evict oldest incomplete frames
                while len(self._order) > 5:
                    old = self._order.pop(0)
                    self._frames.pop(old, None)

                if frame_id in self._frames and len(self._frames[frame_id]) == total:
                    try:
                        jpeg = b"".join(self._frames[frame_id][i] for i in range(total))
                    except KeyError:
                        continue
                    finally:
                        self._frames.pop(frame_id, None)
                        if frame_id in self._order:
                            self._order.remove(frame_id)

                    if jpeg:
                        # Non-blocking put; drop frame if consumer is slow
                        if self._q.qsize() < 3:
                            self._q.put_nowait(jpeg)
                        _inc_rx("video")
                        now = _ts()
                        with _lock:
                            _state["video"][f"{self._label}_ts"] = now
                        _last_ts[f"{self._label}_video"] = now
            except Exception as e:
                log.debug("VID %s recv: %s", self._label, e)


# ─── MJPEG HTTP server ────────────────────────────────────────────────────────
_lane_q:   Queue = Queue(maxsize=3)
_turret_q: Queue = Queue(maxsize=3)

_BOUNDARY = b"--mjpegframe"

def _mjpeg_stream(q: Queue):
    """Generator yielding raw MJPEG multipart bytes."""
    while True:
        try:
            jpeg = q.get(timeout=5.0)
        except Empty:
            continue
        yield (
            _BOUNDARY + b"\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Content-Length: " + str(len(jpeg)).encode() + b"\r\n\r\n"
            + jpeg + b"\r\n"
        )


class _MjpegHandler(BaseHTTPRequestHandler):
    log_message = lambda *a: None   # silence access log

    def do_GET(self):
        if self.path in ("/lane.mjpg", "/lane"):
            q = _lane_q
        elif self.path in ("/turret.mjpg", "/turret"):
            q = _turret_q
        else:
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header("Content-Type",
                         'multipart/x-mixed-replace; boundary="mjpegframe"')
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        try:
            for chunk in _mjpeg_stream(q):
                self.wfile.write(chunk)
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass


# ─── Map tile HTTP server ─────────────────────────────────────────────────────
_TILE_DIR = "./tiles"


class _TileHandler(BaseHTTPRequestHandler):
    log_message = lambda *a: None

    def do_GET(self):
        # path like /19/374440/218390.png
        parts = self.path.lstrip("/").split("/")
        if len(parts) != 3:
            self.send_error(404)
            return
        try:
            z, x, y = parts
            tile_path = os.path.join(_TILE_DIR, z, x, y)
            with open(tile_path, "rb") as f:
                data = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "image/png")
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "public, max-age=86400")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(data)
        except FileNotFoundError:
            self.send_error(404)
        except Exception as e:
            log.debug("Tile error %s: %s", self.path, e)
            self.send_error(500)


# ─── TCP server (rover → GCS: HB, MODE, ALERT, ESTOP) ────────────────────────
def _recv_exact(sock: socket.socket, n: int) -> bytes:
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return b""
        buf += chunk
    return buf


def _handle_tcp_conn(conn: socket.socket, addr):
    try:
        while True:
            raw_len = _recv_exact(conn, 4)
            if not raw_len:
                break
            length = struct.unpack(">I", raw_len)[0]
            raw = _recv_exact(conn, length)
            if not raw:
                break

            payload = msgpack.unpackb(raw, raw=False)
            mtype = payload.get("type", "")
            seq   = payload.get("seq", 0)

            if mtype == "HB":
                with _lock:
                    _state["last_hb"] = _ts()
                continue   # no ACK for heartbeat

            if mtype == "MODE":
                mode = payload.get("mode", "UNKNOWN")
                goal = payload.get("goal", {})
                with _lock:
                    _state["mode"] = mode
                    _state["nav"]["status"] = mode
                    if isinstance(goal, dict):
                        _state["nav"]["goal_x"] = goal.get("x")
                        _state["nav"]["goal_y"] = goal.get("y")
                _last_ts["nav"] = _ts()

            elif mtype == "ESTOP":
                active = payload.get("active", False)
                _add_alert(f"E-STOP from rover — active={active}", "danger")

            elif mtype == "ALERT":
                msg = payload.get("msg", "")
                _add_alert(f"Rover alert: {msg}", "warn")
                _inc_rx("alert")

            # Send ACK
            ack = f"ACK:{seq}".encode()
            conn.sendall(ack)

    except OSError:
        pass
    finally:
        conn.close()


def _tcp_server():
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("", TCP_CMD_PORT))
    srv.listen(2)
    log.info("TCP  command server listening on :%d", TCP_CMD_PORT)
    while True:
        conn, addr = srv.accept()
        threading.Thread(target=_handle_tcp_conn, args=(conn, addr), daemon=True).start()


# ─── Command forwarding to rover (optional) ───────────────────────────────────
_rover_ip: str | None = None
_cmd_sock: socket.socket | None = None


def _forward_to_rover(cmd: dict):
    """Forward a GUI command to the rover via UDP."""
    global _cmd_sock
    if _rover_ip is None:
        return
    if _cmd_sock is None:
        _cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        data = json.dumps(cmd).encode()
        _cmd_sock.sendto(data, (_rover_ip, UDP_ROVER_CMD_PORT))
    except OSError as e:
        log.warning("Forward to rover failed: %s", e)


# ─── WebSocket server ─────────────────────────────────────────────────────────
_ws_clients: set = set()
_ws_lock = asyncio.Lock()


async def _ws_handler(websocket):
    """Handle one WebSocket client connection."""
    async with _ws_lock:
        _ws_clients.add(websocket)
    log.info("WS client connected: %s", websocket.remote_address)
    _add_alert("GCS connected", "success")

    try:
        async for raw in websocket:
            # Commands from GUI (joystick, E-STOP)
            try:
                cmd = json.loads(raw)
                ctype = cmd.get("type", "")
                if ctype == "cmd_vel":
                    vx  = float(cmd.get("linear_ms", 0))
                    wz  = float(cmd.get("angular_rads", 0))
                    with _lock:
                        _state["drive"]["vx"] = vx
                        _state["drive"]["wz"] = wz
                        _state["drive"]["ts"] = _ts()
                    _forward_to_rover(cmd)
                elif ctype == "ESTOP":
                    with _lock:
                        _state["drive"]["vx"] = 0
                        _state["drive"]["wz"] = 0
                    _add_alert("E-STOP from GUI", "danger")
                    _forward_to_rover(cmd)
            except Exception as e:
                log.debug("WS cmd parse: %s", e)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        async with _ws_lock:
            _ws_clients.discard(websocket)
        log.info("WS client disconnected")


async def _broadcast_loop():
    """Push state snapshot to all connected WebSocket clients at 10 Hz."""
    while True:
        await asyncio.sleep(0.1)
        if not _ws_clients:
            continue
        snap = _snapshot()
        msg  = json.dumps(snap)
        dead = set()
        for ws in list(_ws_clients):
            try:
                await ws.send(msg)
            except Exception:
                dead.add(ws)
        if dead:
            async with _ws_lock:
                _ws_clients.difference_update(dead)


async def _ws_main():
    """Run the WebSocket server."""
    server = await websockets.serve(
        _ws_handler,
        "0.0.0.0",
        WS_PORT,
        ping_interval=10,
        ping_timeout=5,
    )
    log.info("WS   server listening on :%d", WS_PORT)
    await asyncio.gather(server.wait_closed(), _broadcast_loop())


# ─── Main ─────────────────────────────────────────────────────────────────────
def main():
    global _TILE_DIR, _rover_ip

    parser = argparse.ArgumentParser(description="Mercury GCS WebSocket bridge")
    parser.add_argument("--rover-ip",  default=None,
                        help="Rover IP — enables forwarding joystick/E-STOP back to rover")
    parser.add_argument("--log-dir",   default=os.path.expanduser("~/robot_logs"),
                        help="Robot log directory (used if video frames come from files)")
    parser.add_argument("--tile-dir",  default="./mercury_gcs/tiles",
                        help="Map tile directory (default: ./mercury_gcs/tiles)")
    args = parser.parse_args()

    _TILE_DIR = args.tile_dir
    _rover_ip = args.rover_ip

    if args.rover_ip:
        log.info("Command forwarding ENABLED → %s:%d", args.rover_ip, UDP_ROVER_CMD_PORT)

    # ── UDP telemetry threads ──────────────────────────────────────────────
    for fn in (_recv_udp_imu, _recv_udp_gps, _recv_udp_odom,
               _recv_udp_enc, _recv_udp_cmdvel, _recv_udp_sys):
        threading.Thread(target=fn, daemon=True).start()

    # ── Video reassemblers ─────────────────────────────────────────────────
    threading.Thread(
        target=_VideoReassembler(UDP_VIDEO_PORT, "lane",   _lane_q).run,
        daemon=True,
    ).start()
    threading.Thread(
        target=_VideoReassembler(UDP_TURRET_PORT, "turret", _turret_q).run,
        daemon=True,
    ).start()

    # ── TCP command server ─────────────────────────────────────────────────
    threading.Thread(target=_tcp_server, daemon=True).start()

    # ── MJPEG HTTP server ──────────────────────────────────────────────────
    mjpeg_srv = ThreadingHTTPServer(("0.0.0.0", MJPEG_PORT), _MjpegHandler)
    mjpeg_srv.daemon_threads = True
    threading.Thread(target=mjpeg_srv.serve_forever, daemon=True).start()
    log.info("MJPEG server listening on :%d  (/lane.mjpg  /turret.mjpg)", MJPEG_PORT)

    # ── Tile HTTP server ───────────────────────────────────────────────────
    tile_srv = ThreadingHTTPServer(("0.0.0.0", TILE_PORT), _TileHandler)
    tile_srv.daemon_threads = True
    threading.Thread(target=tile_srv.serve_forever, daemon=True).start()
    log.info("Tile server  listening on :%d  (map tiles at %s)", TILE_PORT, _TILE_DIR)

    log.info("Bridge ready.  Open the React GUI (npm run dev) and connect to ws://localhost:%d", WS_PORT)

    # ── WebSocket server (runs the event loop in main thread) ─────────────
    try:
        asyncio.run(_ws_main())
    except KeyboardInterrupt:
        log.info("Shutting down")


if __name__ == "__main__":
    main()