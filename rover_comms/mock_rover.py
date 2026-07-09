#!/usr/bin/env python3
"""
mock_rover.py  —  ROS-free fake rover for frontend development/testing

Speaks the EXACT same wire protocol as ugv_gcs_bridge.py (UDP telemetry +
TCP heartbeat/MODE/ALERT + chunked-JPEG UDP video), so gcs_ws_bridge.py and
the React GUI need no changes and don't know the difference between this and
a real rover.

What it does, with ZERO ROS / hardware / camera dependency:
  • Listens for joystick / E-STOP commands forwarded by gcs_ws_bridge.py
    (UDP :5700, JSON — matches gcs_ws_bridge._forward_to_rover) and drives a
    simple unicycle-model simulation with first-order speed lag, so the map
    and telemetry panel respond live to the GUI joystick.
  • If no commands arrive for a while (or --auto-drive is set), it drives an
    autonomous demo pattern on its own — useful for showing the GUI to
    someone with no joystick/controller plugged in.
  • Echoes the "executed" cmd_vel back on UDP :5004, and streams synthetic
    IMU / GPS / odometry / encoder / system-status telemetry on the same
    ports ugv_gcs_bridge.py uses, all derived from the simulated pose.
  • Generates a synthetic main-camera feed (grid horizon + moving "obstacle"
    box) and a synthetic turret-camera feed (a wandering face-like blob with
    a tracking reticle) with OpenCV, chunked over UDP exactly like
    ugv_gcs_bridge.VideoSender, so CameraFeed.jsx / DetectionPanel.jsx have
    something to show without any physical camera.
  • Sends periodic ALERT/HB/MODE messages over TCP :6000 with the same
    length-prefixed msgpack framing + ACK handshake TcpCmdSender expects.

Usage:
    pip install msgpack opencv-python numpy
    python3 mock_rover.py --gcs-ip 127.0.0.1

    # Force autonomous demo driving even while the joystick is connected:
    python3 mock_rover.py --gcs-ip 127.0.0.1 --auto-drive

    # Disable the synthetic camera feeds (telemetry only):
    python3 mock_rover.py --gcs-ip 127.0.0.1 --no-video
"""

import argparse
import json
import logging
import math
import random
import socket
import struct
import threading
import time

import msgpack

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
log = logging.getLogger("mock_rover")

# ─── Ports (must match gcs_ws_bridge.py / ugv_gcs_bridge.py) ─────────────────
UDP_IMU_PORT      = 5000
UDP_GPS_PORT      = 5001
UDP_ODOM_PORT     = 5002
UDP_ENCODER_PORT  = 5003
UDP_CMDVEL_PORT   = 5004
UDP_SYSSTAT_PORT  = 5005
UDP_VIDEO_PORT    = 5600
UDP_TURRET_PORT   = 5601
TCP_CMD_PORT      = 6000
UDP_ROVER_CMD_PORT = 5700   # gcs_ws_bridge --rover-ip forwards joystick/ESTOP here

CHUNK_SIZE       = 60_000
CMD_TIMEOUT_S    = 1.5      # if no cmd_vel refresh in this long, coast to a stop
AUTO_DRIVE_AFTER = 6.0      # seconds of silence before auto-drive kicks in

# Reference GPS origin (DTU, Delhi) — fake GPS is this + integrated offset
ORIGIN_LAT = 28.7500
ORIGIN_LON = 77.1180
M_PER_DEG_LAT = 111_320.0


# ─── UDP sender (identical wire format to ugv_gcs_bridge.UdpSender) ──────────
class UdpSender:
    def __init__(self, ip: str):
        self._ip = ip
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._seq = {}

    def send(self, port: int, topic: str, payload: dict):
        seq = self._seq.get(topic, 0) + 1
        self._seq[topic] = seq
        payload = {**payload, "_seq": seq, "_t": time.time()}
        data = msgpack.packb(payload, use_bin_type=True)
        try:
            self._sock.sendto(data, (self._ip, port))
        except OSError as e:
            log.warning("UDP send error on %s: %s", topic, e)

    def close(self):
        self._sock.close()


# ─── TCP command sender (HB / MODE / ALERT, same framing as the real bridge) ─
class TcpCmdSender:
    def __init__(self, ip: str, port: int = TCP_CMD_PORT):
        self._ip = ip
        self._port = port
        self._sock = None
        self._lock = threading.Lock()
        self._seq = 0
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
            s.connect((self._ip, self._port))
            s.settimeout(None)
            self._sock = s
            log.info("TCP command channel connected to %s:%d", self._ip, self._port)
        except OSError as e:
            log.warning("TCP not ready, will retry: %s", e)
            self._sock = None

    def send_reliable(self, msg_type: str, data: dict, retries: int = 3):
        with self._lock:
            self._seq += 1
            seq = self._seq
            payload = {"type": msg_type, "seq": seq, **data}
            raw = msgpack.packb(payload, use_bin_type=True)
            frame = struct.pack(">I", len(raw)) + raw

            for attempt in range(retries):
                if self._sock is None:
                    self._connect()
                if self._sock is None:
                    time.sleep(0.5)
                    continue
                try:
                    self._sock.sendall(frame)
                    self._sock.settimeout(0.5)
                    ack = self._sock.recv(64)
                    self._sock.settimeout(None)
                    if ack and ack.strip() == f"ACK:{seq}".encode():
                        return True
                except (socket.timeout, OSError) as e:
                    log.debug("Retransmit %s seq=%d attempt %d: %s", msg_type, seq, attempt + 1, e)
                    self._sock = None
            return False

    def send_heartbeat(self):
        if self._sock is None:
            self._connect()
        if self._sock is None:
            return
        try:
            hb = msgpack.packb({"type": "HB", "t": time.time()}, use_bin_type=True)
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


# ─── Video sender (identical chunking to ugv_gcs_bridge.VideoSender) ─────────
class VideoSender:
    def __init__(self, ip: str, port: int):
        self._addr = (ip, port)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 * 1024 * 1024)
        self._frame_id = 0

    def send_frame(self, jpeg_bytes: bytes):
        self._frame_id = (self._frame_id + 1) & 0xFFFFFFFF
        chunks = [jpeg_bytes[i:i + CHUNK_SIZE] for i in range(0, len(jpeg_bytes), CHUNK_SIZE)]
        total = len(chunks)
        for idx, chunk in enumerate(chunks):
            header = struct.pack(">IHH", self._frame_id, idx, total)
            try:
                self._sock.sendto(header + chunk, self._addr)
            except OSError:
                pass

    def close(self):
        self._sock.close()


# ─── Shared simulated rover state ─────────────────────────────────────────────
class RoverSim:
    def __init__(self, auto_drive: bool = False):
        self.lock = threading.Lock()
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0        # radians
        self.vx = 0.0         # current (actual, lagged) linear speed m/s
        self.wz = 0.0         # current (actual, lagged) angular speed rad/s
        self.target_vx = 0.0
        self.target_wz = 0.0
        self.estopped = False
        self.auto_drive = auto_drive
        self.last_cmd_t = 0.0
        self.t0 = time.time()

    def set_target(self, vx: float, wz: float):
        with self.lock:
            self.target_vx = max(-1.0, min(1.0, vx))
            self.target_wz = max(-1.5, min(1.5, wz))
            self.last_cmd_t = time.time()
            self.estopped = False

    def estop(self):
        with self.lock:
            self.target_vx = 0.0
            self.target_wz = 0.0
            self.estopped = True

    def step(self, dt: float):
        """Advance the simulation by dt seconds. Call from one thread only."""
        with self.lock:
            silent_for = time.time() - self.last_cmd_t
            if self.estopped:
                tgt_vx, tgt_wz = 0.0, 0.0
            elif self.auto_drive or silent_for > AUTO_DRIVE_AFTER:
                # gentle demo circle so the GUI has something to look at
                tgt_vx, tgt_wz = 0.35, 0.25 * math.sin((time.time() - self.t0) * 0.15)
            else:
                tgt_vx, tgt_wz = self.target_vx, self.target_wz

            # first-order lag so speed changes look physical, not stepwise
            lag = min(1.0, dt / 0.35)
            self.vx += (tgt_vx - self.vx) * lag
            self.wz += (tgt_wz - self.wz) * lag

            self.yaw += self.wz * dt
            self.x += self.vx * math.cos(self.yaw) * dt
            self.y += self.vx * math.sin(self.yaw) * dt

            return dict(x=self.x, y=self.y, yaw=self.yaw, vx=self.vx, wz=self.wz)


# ─── Command listener: receives joystick/E-STOP forwarded by gcs_ws_bridge ───
def cmd_listener(sim: RoverSim, listen_port: int, stop: threading.Event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", listen_port))
    sock.settimeout(1.0)
    log.info("Listening for GUI cmd_vel/ESTOP on UDP :%d", listen_port)
    while not stop.is_set():
        try:
            data, _ = sock.recvfrom(4096)
            cmd = json.loads(data.decode())
            ctype = cmd.get("type", "")
            if ctype == "cmd_vel":
                sim.set_target(float(cmd.get("linear_ms", 0)), float(cmd.get("angular_rads", 0)))
            elif ctype == "ESTOP":
                sim.estop()
                log.info("E-STOP received from GUI")
        except socket.timeout:
            continue
        except Exception as e:
            log.debug("cmd_listener: %s", e)


# ─── Telemetry loop: derives fake IMU/GPS/odom/encoder/cmd_vel from the sim ──
def telemetry_loop(sim: RoverSim, udp: UdpSender, stop: threading.Event, hz: float = 20.0):
    dt = 1.0 / hz
    tick = 0
    while not stop.is_set():
        pose = sim.step(dt)
        yaw_deg = math.degrees(pose["yaw"])

        # ODOM — every tick
        udp.send(UDP_ODOM_PORT, "odom", {
            "x": pose["x"], "y": pose["y"], "vx": pose["vx"], "wz": pose["wz"],
        })

        # cmd_vel echo ("what the rover actually executed") — every tick
        udp.send(UDP_CMDVEL_PORT, "cmd_vel", {
            "linear": pose["vx"], "angular": pose["wz"],
        })

        # IMU — ~20 Hz, small synthetic noise so plots aren't dead-flat
        qz = math.sin(pose["yaw"] / 2.0)
        qw = math.cos(pose["yaw"] / 2.0)
        udp.send(UDP_IMU_PORT, "imu", {
            "ax": random.uniform(-0.05, 0.05), "ay": random.uniform(-0.05, 0.05), "az": 9.81,
            "wx": random.uniform(-0.02, 0.02), "wy": random.uniform(-0.02, 0.02), "wz": pose["wz"],
            "ox": 0.0, "oy": 0.0, "oz": qz, "ow": qw,
        })

        # GPS — 5 Hz is plenty
        if tick % max(1, int(hz / 5)) == 0:
            dlat = pose["y"] / M_PER_DEG_LAT
            dlon = pose["x"] / (M_PER_DEG_LAT * math.cos(math.radians(ORIGIN_LAT)))
            udp.send(UDP_GPS_PORT, "gps", {
                "lat": ORIGIN_LAT + dlat, "lon": ORIGIN_LON + dlon, "alt": 216.0,
            })

        # Encoders — 10 Hz, derived from wheel kinematics (fake track width)
        if tick % max(1, int(hz / 10)) == 0:
            track = 0.4
            wl = pose["vx"] - pose["wz"] * track / 2.0
            wr = pose["vx"] + pose["wz"] * track / 2.0
            udp.send(UDP_ENCODER_PORT, "encoder", {
                "names": ["left_wheel", "right_wheel"],
                "position": [pose["x"], pose["x"]],   # placeholder cumulative pos
                "velocity": [wl, wr],
            })

        # System status string — 1 Hz
        if tick % int(hz) == 0:
            cpu = 20 + 15 * abs(math.sin(time.time() * 0.1))
            udp.send(UDP_SYSSTAT_PORT, "sys_status", {
                "status": f"OK cpu={cpu:.0f}% mode={'AUTO' if sim.auto_drive else 'MANUAL'}",
            })

        tick += 1
        time.sleep(dt)


# ─── TCP heartbeat / occasional alerts ────────────────────────────────────────
def heartbeat_loop(tcp: TcpCmdSender, stop: threading.Event):
    while not stop.is_set():
        tcp.send_heartbeat()
        time.sleep(0.1)


def alert_loop(tcp: TcpCmdSender, stop: threading.Event):
    messages = [
        ("Rover systems nominal", "info"),
        ("Battery at 87%", "info"),
        ("GPS fix acquired", "success"),
        ("Wheel encoder jitter detected", "warn"),
    ]
    while not stop.is_set():
        time.sleep(random.uniform(15, 30))
        if stop.is_set():
            break
        msg, sev = random.choice(messages)
        tcp.send_reliable("ALERT", {"msg": msg})
        log.info("Sent fake alert: %s", msg)


# ─── Synthetic camera feeds (OpenCV, no real camera needed) ──────────────────
def video_loop(sim: RoverSim, sender: VideoSender, label: str, stop: threading.Event,
                fps: float = 15.0, turret: bool = False):
    import cv2
    import numpy as np

    w, h = 640, 480
    t0 = time.time()
    while not stop.is_set():
        t = time.time() - t0
        frame = np.zeros((h, w, 3), dtype=np.uint8)

        if not turret:
            # Main/lane camera: horizon + fake ground grid + a drifting obstacle box
            frame[:] = (40, 30, 25)                      # dark ground
            frame[: h // 2, :] = (110, 70, 30)            # sky
            for gy in range(h // 2, h, 30):
                cv2.line(frame, (0, gy), (w, gy), (60, 60, 60), 1)
            vx = sim.vx if not sim.lock.locked() else 0.0
            cx = int(w / 2 + 120 * math.sin(t * 0.4))
            cy = int(h * 0.75)
            cv2.rectangle(frame, (cx - 30, cy - 30), (cx + 30, cy + 30), (0, 140, 255), -1)
            cv2.putText(frame, f"MAIN CAM  vx={sim.vx:+.2f} wz={sim.wz:+.2f}",
                        (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        else:
            # Turret/face camera: a wandering "face" blob with a tracking reticle
            frame[:] = (25, 25, 25)
            fx = int(w / 2 + 150 * math.sin(t * 0.6))
            fy = int(h / 2 + 80 * math.sin(t * 0.37 + 1.0))
            cv2.circle(frame, (fx, fy), 55, (180, 170, 150), -1)      # face
            cv2.circle(frame, (fx - 18, fy - 10), 6, (20, 20, 20), -1)  # eyes
            cv2.circle(frame, (fx + 18, fy - 10), 6, (20, 20, 20), -1)
            cv2.ellipse(frame, (fx, fy + 20), (18, 8), 0, 0, 180, (20, 20, 20), 2)
            cv2.rectangle(frame, (fx - 65, fy - 65), (fx + 65, fy + 65), (0, 255, 0), 2)
            cv2.line(frame, (w // 2, 0), (w // 2, h), (0, 255, 0), 1)
            cv2.line(frame, (0, h // 2), (w, h // 2), (0, 255, 0), 1)
            cv2.putText(frame, "TURRET CAM  TRACKING", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1, cv2.LINE_AA)

        cv2.putText(frame, time.strftime("%H:%M:%S"), (w - 110, h - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            sender.send_frame(buf.tobytes())

        time.sleep(1.0 / fps)


# ─── Entry point ──────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="ROS-free fake rover for GUI testing")
    parser.add_argument("--gcs-ip", default="127.0.0.1", help="GCS bridge IP (default: 127.0.0.1)")
    parser.add_argument("--no-video", action="store_true", help="Disable synthetic camera feeds")
    parser.add_argument("--auto-drive", action="store_true",
                         help="Always drive the demo pattern, ignoring GUI joystick input")
    parser.add_argument("--hz", type=float, default=20.0, help="Telemetry/sim update rate")
    args = parser.parse_args()

    log.info("Faking a rover at %s (no ROS, no hardware, no camera required)", args.gcs_ip)

    sim = RoverSim(auto_drive=args.auto_drive)
    udp = UdpSender(args.gcs_ip)
    tcp = TcpCmdSender(args.gcs_ip)
    stop = threading.Event()

    threading.Thread(target=heartbeat_loop, args=(tcp, stop), daemon=True).start()
    threading.Thread(target=alert_loop, args=(tcp, stop), daemon=True).start()
    threading.Thread(target=cmd_listener, args=(sim, UDP_ROVER_CMD_PORT, stop), daemon=True).start()
    threading.Thread(target=telemetry_loop, args=(sim, udp, stop, args.hz), daemon=True).start()

    vsend_main = vsend_turret = None
    if not args.no_video:
        try:
            import cv2  # noqa: F401
            import numpy  # noqa: F401
            vsend_main = VideoSender(args.gcs_ip, UDP_VIDEO_PORT)
            vsend_turret = VideoSender(args.gcs_ip, UDP_TURRET_PORT)
            threading.Thread(target=video_loop, args=(sim, vsend_main, "main", stop),
                              kwargs={"turret": False}, daemon=True).start()
            threading.Thread(target=video_loop, args=(sim, vsend_turret, "turret", stop),
                              kwargs={"turret": True}, daemon=True).start()
            log.info("Synthetic camera feeds started (main :%d, turret :%d)",
                      UDP_VIDEO_PORT, UDP_TURRET_PORT)
        except ImportError:
            log.warning("opencv-python/numpy not installed — video disabled (pip install opencv-python numpy)")

    log.info("Mock rover running. Drive it from the GUI joystick, or use --auto-drive. Ctrl+C to stop.")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        log.info("Shutting down")
        stop.set()
        udp.close()
        tcp.close()
        if vsend_main:
            vsend_main.close()
        if vsend_turret:
            vsend_turret.close()


if __name__ == "__main__":
    main()
