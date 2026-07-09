#!/usr/bin/env python3
"""
ugv_gcs_bridge.py  —  Rover side, NO ROS 2 dependency
Reads from log files written by ugv_logger.py and forwards to GCS.

Mental model:
  ├── File tailer threads (one per log file)
  │   ├── state_log.json     → IMU  → UDP :5000
  │   │                      → GPS  → UDP :5001
  │   │                      → ODOM → UDP :5002
  │   ├── encoder_log.json   → Encoder → UDP :5003
  │   ├── control_log.json   → CMD_VEL → UDP :5004
  │   ├── system_log.json    → SYS_STATUS → UDP :5005
  │   ├── alerts_log.json    → ALERT → TCP :6000 (with ACK)
  │   └── navigation_log.json→ MODE/ESTOP/FACE_TASK → TCP :6000 (with ACK)
  ├── Heartbeat thread       → TCP :6000 at 10 Hz
  └── Video thread           → /camera/image_raw frames → UDP :5600
      (reads encoded frames from video_frames/ folder dropped by logger)

Usage:
    pip install msgpack opencv-python numpy
    python3 ugv_gcs_bridge.py --gcs-ip 192.168.88.2

    # Disable video:
    python3 ugv_gcs_bridge.py --gcs-ip 192.168.88.2 --no-video

    # Custom log directory:
    python3 ugv_gcs_bridge.py --gcs-ip 192.168.88.2 --log-dir ~/robot_logs
"""

import argparse
import json
import os
import socket
import struct
import threading
import time
import logging

import msgpack

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("ugv_bridge")

# ─── Ports (must match gcs_receiver.py) ──────────────────────────────────────
UDP_IMU_PORT     = 5000
UDP_GPS_PORT     = 5001
UDP_ODOM_PORT    = 5002
UDP_ENCODER_PORT = 5003
UDP_CMDVEL_PORT  = 5004
UDP_SYSSTAT_PORT = 5005
UDP_VIDEO_PORT   = 5600
UDP_TURRET_VIDEO_PORT = 5601
TCP_CMD_PORT     = 6000
TCP_FILE_PORT    = 7000

CMD_RETRANSMIT_INTERVAL = 0.5
CMD_MAX_RETRIES         = 10
CHUNK_SIZE              = 60000   # bytes per UDP video packet
TAIL_INTERVAL           = 0.05   # seconds between file tail polls (50ms)


# ─── UDP sender ───────────────────────────────────────────────────────────────
class UdpSender:
    def __init__(self, gcs_ip: str):
        self._gcs_ip = gcs_ip
        self._sock   = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq    = {}

    def send(self, port: int, topic: str, payload: dict):
        seq = self._seq.get(topic, 0) + 1
        self._seq[topic] = seq
        payload["_seq"] = seq
        payload["_t"]   = time.time()
        data = msgpack.packb(payload, use_bin_type=True)
        try:
            self._sock.sendto(data, (self._gcs_ip, port))
        except OSError as e:
            log.warning("UDP send error on %s: %s", topic, e)

    def close(self):
        self._sock.close()


# ─── TCP command sender (with ACK) ───────────────────────────────────────────
class TcpCmdSender:
    def __init__(self, gcs_ip: str, port: int = TCP_CMD_PORT):
        self._gcs_ip       = gcs_ip
        self._port         = port
        self._sock         = None
        self._lock         = threading.Lock()
        self._seq          = 0
        self._last_attempt = 0.0
        self._connect()

    def _connect(self):
        now = time.time()
        if now - self._last_attempt < 3.0:
            return
        self._last_attempt = now
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(3)
            s.connect((self._gcs_ip, self._port))
            s.settimeout(None)
            self._sock = s
            log.info("TCP command channel connected to %s:%d", self._gcs_ip, self._port)
        except OSError as e:
            log.warning("TCP not ready, will retry in 3s: %s", e)
            self._sock = None

    def send_reliable(self, msg_type: str, data: dict):
        with self._lock:
            self._seq += 1
            seq = self._seq
            payload = {"type": msg_type, "seq": seq, **data}
            raw   = msgpack.packb(payload, use_bin_type=True)
            frame = struct.pack(">I", len(raw)) + raw

            for attempt in range(CMD_MAX_RETRIES):
                if self._sock is None:
                    self._connect()
                if self._sock is None:
                    log.error("No TCP connection — cannot send %s (attempt %d)", msg_type, attempt + 1)
                    time.sleep(CMD_RETRANSMIT_INTERVAL)
                    continue
                try:
                    self._sock.sendall(frame)
                    self._sock.settimeout(CMD_RETRANSMIT_INTERVAL)
                    ack_raw = self._sock.recv(64)
                    self._sock.settimeout(None)
                    if ack_raw and ack_raw.strip() == f"ACK:{seq}".encode():
                        log.info("ACK received for %s seq=%d", msg_type, seq)
                        return True
                    else:
                        log.warning("Bad ACK for %s seq=%d: %r", msg_type, seq, ack_raw)
                except (socket.timeout, OSError) as e:
                    log.warning("Retransmit %s seq=%d attempt %d: %s", msg_type, seq, attempt + 1, e)
                    self._sock = None

            log.error("FAILED to deliver %s after %d attempts", msg_type, CMD_MAX_RETRIES)
            return False

    def send_heartbeat(self):
        if self._sock is None:
            self._connect()
        if self._sock is None:
            return
        try:
            hb    = msgpack.packb({"type": "HB", "t": time.time()}, use_bin_type=True)
            frame = struct.pack(">I", len(hb)) + hb
            self._sock.sendall(frame)
        except OSError:
            self._sock = None

    def close(self):
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass


# ─── TCP file sender (target photos) ─────────────────────────────────────────
class TcpFileSender:
    def __init__(self, gcs_ip: str, port: int = TCP_FILE_PORT):
        self._gcs_ip = gcs_ip
        self._port   = port

    def send_photo(self, data: bytes, filename: str = "target.jpg"):
        try:
            with socket.create_connection((self._gcs_ip, self._port), timeout=5) as s:
                name_enc = filename.encode()
                header   = struct.pack(">I", len(name_enc)) + name_enc + \
                           struct.pack(">Q", len(data))
                s.sendall(header + data)
                log.info("Photo sent: %s (%d bytes)", filename, len(data))
        except OSError as e:
            log.error("Photo transfer failed: %s", e)


# ─── Video sender (JPEG chunks over UDP) ─────────────────────────────────────
class VideoSender:
    def __init__(self, gcs_ip: str, port: int = UDP_VIDEO_PORT, quality: int = 60):
        self._addr     = (gcs_ip, port)
        self._quality  = quality
        self._sock     = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self._frame_id = 0

    def send_frame(self, jpeg_bytes: bytes):
        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF
        chunks = [jpeg_bytes[i:i+CHUNK_SIZE] for i in range(0, len(jpeg_bytes), CHUNK_SIZE)]
        total  = len(chunks)
        for idx, chunk in enumerate(chunks):
            header = struct.pack(">IHH", self._frame_id, idx, total)
            try:
                self._sock.sendto(header + chunk, self._addr)
            except OSError:
                pass

    def close(self):
        self._sock.close()


# ─── File tailer ─────────────────────────────────────────────────────────────
class FileTailer:
    """
    Tails a JSONL file and yields new lines as they are appended.
    Seeks to the end on startup so only new data is forwarded.
    """
    def __init__(self, path: str):
        self._path = path
        self._fh   = None

    def _open(self):
        try:
            self._fh = open(self._path, "r")
            self._fh.seek(0, 2)   # seek to end — don't replay old data
            log.info("Tailing %s", self._path)
            return True
        except OSError:
            return False

    def readlines(self):
        """Call repeatedly. Returns list of new JSON strings since last call."""
        if self._fh is None:
            if not self._open():
                return []
        lines = []
        while True:
            line = self._fh.readline()
            if not line:
                break
            line = line.strip()
            if line:
                lines.append(line)
        return lines


# ─── Log dispatch — maps log entries to network sends ────────────────────────
def _dispatch_state(entry: dict, udp: UdpSender):
    """state_log.json contains imu / gps / odom entries."""
    if "imu" in entry:
        d = entry["imu"]
        udp.send(UDP_IMU_PORT, "imu", {
            "ax": d["linear_acceleration"][0],
            "ay": d["linear_acceleration"][1],
            "az": d["linear_acceleration"][2],
            "wx": d["angular_velocity"][0],
            "wy": d["angular_velocity"][1],
            "wz": d["angular_velocity"][2],
            "ox": d["orientation"][0],
            "oy": d["orientation"][1],
            "oz": d["orientation"][2],
            "ow": d["orientation"][3],
        })
    elif "gps" in entry:
        d = entry["gps"]
        udp.send(UDP_GPS_PORT, "gps", {
            "lat": d["lat"],
            "lon": d["lon"],
            "alt": d["alt"],
        })
    elif "odom" in entry:
        d = entry["odom"]
        udp.send(UDP_ODOM_PORT, "odom", {
            "x":  d["pos"][0],
            "y":  d["pos"][1],
            "vx": d["vel"][0],
            "wz": d["vel"][1],
        })


def _dispatch_encoder(entry: dict, udp: UdpSender):
    if "encoder" in entry:
        d = entry["encoder"]
        udp.send(UDP_ENCODER_PORT, "encoder", {
            "names":    d["names"],
            "position": d["position"],
            "velocity": d["velocity"],
        })


def _dispatch_control(entry: dict, udp: UdpSender):
    if "cmd_vel" in entry:
        d = entry["cmd_vel"]
        udp.send(UDP_CMDVEL_PORT, "cmd_vel", {
            "linear":  d["linear"],
            "angular": d["angular"],
        })


def _dispatch_system(entry: dict, udp: UdpSender):
    if "system_status" in entry:
        udp.send(UDP_SYSSTAT_PORT, "sys_status", {"status": entry["system_status"]})


def _dispatch_alert(entry: dict, tcp: TcpCmdSender):
    if "alert" in entry:
        threading.Thread(
            target=tcp.send_reliable,
            args=("ALERT", {"msg": entry["alert"]}),
            daemon=True,
        ).start()


_last_nav_mode = {"v": None}   # track last sent mode to avoid repeats

def _dispatch_nav(entry: dict, tcp: TcpCmdSender):
    """navigation_log.json — forward goal only, skip repeated mode spam."""
    if "goal" in entry:
        new_mode = f"NAVIGATING:{entry['goal']}"
        if _last_nav_mode["v"] == new_mode:
            return   # already sent this
        _last_nav_mode["v"] = new_mode
        threading.Thread(
            target=tcp.send_reliable,
            args=("MODE", {"mode": "NAVIGATING", "goal": entry["goal"]}),
            daemon=True,
        ).start()


# ─── Tailer thread factory ────────────────────────────────────────────────────
def _start_tailer(path: str, dispatch_fn, stop_event: threading.Event):
    def _run():
        tailer = FileTailer(path)
        while not stop_event.is_set():
            for line in tailer.readlines():
                try:
                    entry = json.loads(line)
                    dispatch_fn(entry)
                except json.JSONDecodeError:
                    pass
            time.sleep(TAIL_INTERVAL)
    t = threading.Thread(target=_run, daemon=True)
    t.start()
    return t


# ─── Heartbeat thread ─────────────────────────────────────────────────────────
def _start_heartbeat(tcp: TcpCmdSender, stop_event: threading.Event):
    def _run():
        while not stop_event.is_set():
            tcp.send_heartbeat()
            time.sleep(0.1)   # 10 Hz
    threading.Thread(target=_run, daemon=True).start()


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="UGV → GCS bridge (no ROS)")
    parser.add_argument("--gcs-ip",   required=True,                    help="GCS laptop IP")
    parser.add_argument("--log-dir",  default=os.path.expanduser("~/robot_logs"),
                                                                         help="Log directory (default: ~/robot_logs)")
    parser.add_argument("--no-video", action="store_true",               help="Disable video stream")
    args = parser.parse_args()

    log_dir = args.log_dir
    log.info("Reading logs from %s", log_dir)
    log.info("Sending to GCS at %s", args.gcs_ip)

    udp  = UdpSender(args.gcs_ip)
    tcp  = TcpCmdSender(args.gcs_ip)
    file = TcpFileSender(args.gcs_ip)

    stop = threading.Event()

    # Start heartbeat immediately so TCP connects
    _start_heartbeat(tcp, stop)

    # Give TCP 3s to connect before processing log files
    log.info("Waiting 3s for TCP to connect...")
    time.sleep(3)

    # Start one tailer thread per log file
    tailers = [
        (os.path.join(log_dir, "state_log.json"),
            lambda e: _dispatch_state(e, udp)),
        (os.path.join(log_dir, "encoder_log.json"),
            lambda e: _dispatch_encoder(e, udp)),
        (os.path.join(log_dir, "control_log.json"),
            lambda e: _dispatch_control(e, udp)),
        (os.path.join(log_dir, "system_log.json"),
            lambda e: _dispatch_system(e, udp)),
        (os.path.join(log_dir, "alerts_log.json"),
            lambda e: _dispatch_alert(e, tcp)),
        (os.path.join(log_dir, "navigation_log.json"),
            lambda e: _dispatch_nav(e, tcp)),
    ]

    for path, fn in tailers:
        _start_tailer(path, fn, stop)

    # Video sender — reads JPEG frames dropped by logger into shared folders
    vsend = None
    vsend_turret = None
    if not args.no_video:
        try:
            import cv2
            import numpy as np

            def _make_watcher(folder: str, sender: "VideoSender", label: str):
                os.makedirs(folder, exist_ok=True)
                def _loop():
                    seen = set()
                    while not stop.is_set():
                        try:
                            # Only pick up finished .jpg files — logger writes to
                            # .tmp first then renames, so anything still .tmp is
                            # mid-write and would decode as a black/corrupt frame.
                            files = sorted(
                                f for f in os.listdir(folder) if f.endswith(".jpg")
                            )
                            for fname in files:
                                if fname in seen:
                                    continue
                                seen.add(fname)
                                fpath = os.path.join(folder, fname)
                                try:
                                    with open(fpath, "rb") as f:
                                        data = f.read()
                                    if data:
                                        sender.send_frame(data)
                                except OSError as e:
                                    log.warning("%s: failed reading %s: %s", label, fname, e)
                                finally:
                                    try:
                                        os.remove(fpath)
                                    except OSError:
                                        pass
                            if len(seen) > 500:
                                seen.clear()
                        except Exception as e:
                            log.warning("%s video loop error: %s", label, e)
                        time.sleep(0.033)   # ~30 fps poll
                threading.Thread(target=_loop, daemon=True).start()

            # Main camera
            vsend = VideoSender(args.gcs_ip, port=UDP_VIDEO_PORT)
            log.info("Main camera sender ready on UDP :%d", UDP_VIDEO_PORT)
            _make_watcher(os.path.join(log_dir, "video_frames"), vsend, "main")

            # Turret / face detection camera
            vsend_turret = VideoSender(args.gcs_ip, port=UDP_TURRET_VIDEO_PORT)
            log.info("Turret camera sender ready on UDP :%d", UDP_TURRET_VIDEO_PORT)
            _make_watcher(os.path.join(log_dir, "video_frames_turret"), vsend_turret, "turret")

        except ImportError:
            log.warning("opencv-python not installed — video disabled")

    log.info("Bridge running. Press Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("Shutting down")
        stop.set()
        udp.close()
        tcp.close()
        if vsend:
            vsend.close()
        if vsend_turret:
            vsend_turret.close()


if __name__ == "__main__":
    main()