#!/usr/bin/env python3
import asyncio, time, csv, os, sys, logging, tempfile, queue  # add queue
from typing import Optional  # 3.8-compatible Optional[]
from collections import defaultdict
from cot_utils import generate_chat_cot, generate_gps_cot, generate_obstacle_cot

# === Latest textual snippets used to build GeoChat ===
latest_msgs = defaultdict(str)

# === Latest GPS state used by the position CoT ===
latest_gps = {"lat": None, "lon": None, "hae": 999999.0}

# === Last receive epoch for each message type (used to compute age_ms) ===
_last_rx = {"gps": None, "vel": None, "can": None, "hdg": None, "pose": None}

# === Extra state for logging and message composition (seq, heading, linear_x, gps_latency_ms, etc.) ===
latest_state = {
    "gps": {"seq": None, "latency_ms": None, "stamp": None},
    "vel": {"seq": None, "linear_x": None},
    "can": {"throttle": None, "steering": None, "gear": None,
            "engineState": None, "externalControl": None, "driveLine": None},
    "hdg": {"heading": None},
    "pose": {},
}

# === “New data” counters ===
RX_COUNTER = 0          # Incremented on *any* inbound message
GPS_COUNTER = 0         # Incremented only on inbound GPS
LAST_ANY_RX_TS = None   # Last time any message was received (epoch seconds)

# Text sent when there is no new inbound message in the last check interval
NODATA_MSG = os.getenv("NODATA_MSG", "(No new data) No telemetry/control updates in the last 1s")

def _now() -> float:
    return time.time()

def _iso(ts: Optional[float]) -> Optional[str]:
    """Format epoch seconds into ISO8601 with millisecond precision, UTC."""
    if ts is None:
        return None
    try:
        return time.strftime('%Y-%m-%dT%H:%M:%S', time.gmtime(ts)) + f".{int((ts%1)*1000):03d}Z"
    except Exception:
        return None

# ---- Dynamic CSV path: logs/<program_name>_<YYYYmmdd_HHMMSS>.csv ----
_RUN_TS = time.strftime("%Y%m%d_%H%M%S", time.localtime())
_PROG   = os.path.splitext(os.path.basename(sys.argv[0] if sys.argv and sys.argv[0] else "session"))[0]
_LOGDIR = os.path.join(os.path.dirname(__file__), "logs")
os.makedirs(_LOGDIR, exist_ok=True)
_DEFAULT_CSV_PATH = os.path.join(_LOGDIR, f"{_PROG}_{_RUN_TS}.csv")

class _CSVLogger:
    """CSV logger that writes one row per outbound CoT (chat/pos/nodata)."""
    def __init__(self, path: Optional[str] = None):
        self.path = path or _DEFAULT_CSV_PATH
        self._init()

    def _try_open(self, path):
        # Open in append mode; handle Windows/OneDrive/Excel file locks gracefully.
        try:
            return open(path, "a", newline="", encoding="utf-8")
        except PermissionError:
            base, ext = os.path.splitext(path)
            alt1 = f"{base}_pid{os.getpid()}{ext}"
            try:
                return open(alt1, "a", newline="", encoding="utf-8")
            except PermissionError:
                alt2 = os.path.join(tempfile.gettempdir(), os.path.basename(alt1))
                logging.warning(f"[CSV] Permission denied for '{path}'. Falling back to '{alt2}'.")
                return open(alt2, "a", newline="", encoding="utf-8")

    def _init(self):
        new = not os.path.exists(self.path)
        self._f = self._try_open(self.path)
        self._w = csv.writer(self._f)
        if new:
            # Columns include both receiver-side and sender-side timestamps
            self._w.writerow([
                "recv_epoch", "recv_iso",         # receiver timestamp (when we send to TAK)
                "sent_epoch", "sent_iso",         # sender timestamp from packet (currently GPS only)
                "kind",                           # "chat" / "pos" / "nodata"
                "msg_type",                       # "chat" or "gps" etc.
                "seq",
                "lat", "lon",
                "linear_x",
                "heading",
                "throttle", "steering",
                "gps_latency_ms",
                "vel_age_ms", "can_age_ms", "hdg_age_ms", "pose_age_ms",
            ])
            self._f.flush()

    def log(self, **fields):
        recv_epoch = float(fields.get("recv_epoch", _now()))
        sent_epoch = fields.get("sent_epoch")
        row = [
            f"{recv_epoch:.3f}",
            _iso(recv_epoch),
            f"{sent_epoch:.3f}" if isinstance(sent_epoch, (int, float)) else None,
            _iso(sent_epoch) if isinstance(sent_epoch, (int, float)) else None,
            fields.get("kind"),
            fields.get("msg_type"),
            fields.get("seq"),
            fields.get("lat"),
            fields.get("lon"),
            fields.get("linear_x"),
            fields.get("heading"),
            fields.get("throttle"),
            fields.get("steering"),
            fields.get("gps_latency_ms"),
            fields.get("vel_age_ms"),
            fields.get("can_age_ms"),
            fields.get("hdg_age_ms"),
            fields.get("pose_age_ms"),
        ]
        self._w.writerow(row)
        self._f.flush()

LOGGER = _CSVLogger()  # Module-level singleton

async def handle_queue_data(q, tx_queue, metrics_q=None):
    """
    Pull from multiprocessing.Queue using a blocking get() in a worker thread
    (run_in_executor), so it's reliable (no empty() race) and doesn't block the
    asyncio loop. Compatible with Python 3.8.
    """
    gear_map = {0: "Neutral", 1: "Forward", 2: "Reverse"}
    engine_map = {0: "Stopped", 1: "Running", 2: "Cranking", 3: "Fault"}
    ctrl_map = {0: "Inactive", 1: "Active", 2: "Takeover", 3: "E-Stop"}
    drive_map = {0: "Port", 1: "Starboard"}

    loop = asyncio.get_running_loop()

    while True:
        try:
            # Run blocking q.get(timeout=1.0) in thread pool (3.8-safe)
            msg_type, data = await loop.run_in_executor(None, q.get, True, 1.0)
        except queue.Empty:
            await asyncio.sleep(0)  # yield control; BundlerWorker handles "no data" message
            continue
        except asyncio.CancelledError:
            raise
        except Exception as e:
            logging.exception(f"[handle_queue_data] queue read error: {e}")
            await asyncio.sleep(0.1)
            continue

        t = _now()
        _last_rx[msg_type] = t

        # mark new data (for Bundler/GPS workers)
        global RX_COUNTER, GPS_COUNTER, LAST_ANY_RX_TS
        RX_COUNTER += 1
        LAST_ANY_RX_TS = t

        if msg_type == "gps":
            latency_ms = data.get("latency_ms")
            seq   = data.get("seq")
            stamp = data.get("stamp")
            lat   = data.get("lat")
            lon   = data.get("lon")

            latency_str = f", Latency={latency_ms:.1f} ms" if isinstance(latency_ms, (int, float)) else ""
            latest_msgs['gps'] = f"[GPS] Seq={seq}, Time={stamp:.3f}, Lat={lat:.6f}, Lon={lon:.6f}{latency_str}"

            latest_gps['lat'] = lat
            latest_gps['lon'] = lon
            latest_state["gps"].update({"seq": seq, "stamp": stamp, "latency_ms": latency_ms})
            GPS_COUNTER += 1

        elif msg_type == "vel":
            seq = data.get("seq")
            linear_x = data.get("linear_x")
            latest_msgs['vel'] = f"[VEL] Seq={seq}, LinearX={linear_x:.3f}"
            latest_state["vel"].update({"seq": seq, "linear_x": linear_x})

        elif msg_type == "can":
            if "gear" in data:
                latest_msgs['can'] = (
                    f"[CAN] Drive: {drive_map.get(data.get('driveLine'), '?')}, "
                    f"Ctrl: {ctrl_map.get(data.get('externalControl'), '?')}, "
                    f"Engine: {engine_map.get(data.get('engineState'), '?')}, "
                    f"Gear: {gear_map.get(data.get('gear'), '?')}, "
                    f"Throttle: {data.get('throttle')}%, "
                    f"Steering: {data.get('steering')}"
                )
                latest_state["can"].update({
                    "throttle": data.get("throttle"),
                    "steering": data.get("steering"),
                    "gear": data.get("gear"),
                    "engineState": data.get("engineState"),
                    "externalControl": data.get("externalControl"),
                    "driveLine": data.get("driveLine"),
                })
            else:
                latest_msgs['can'] = f"[CAN] Throttle={data.get('throttle')}, Steering={data.get('steering')}"
                latest_state["can"].update({
                    "throttle": data.get("throttle"),
                    "steering": data.get("steering"),
                })

        elif msg_type == "hdg":
            heading = data.get("heading")
            latest_msgs['hdg'] = f"[HDG] Heading={heading:.2f}°"
            latest_state["hdg"].update({"heading": heading})

        # elif msg_type == "pose":
        #     ...

def _ages_ms(now_s: float):
    """Return age_ms for each message type (time since last receive)."""
    def age_of(key):
        ts = _last_rx.get(key)
        return int((now_s - ts) * 1000) if ts is not None else None
    return {
        "vel_age_ms": age_of("vel"),
        "can_age_ms": age_of("can"),
        "hdg_age_ms": age_of("hdg"),
        "pose_age_ms": age_of("pose"),
    }

class BundlerWorker:
    """
    Every second:
      - If there is new inbound data since the last tick → send a combined GeoChat (GPS/CAN/HDG/VEL).
      - Otherwise → send a single-line “no new data” message.
    All outbound messages are logged to CSV (kind="chat" or "nodata").
    """
    def __init__(self, tx_queue):
        self.tx_queue = tx_queue
        self._last_rx_counter_seen = -1

    async def run(self):
        while True:
            now_s = _now()
            current_counter = RX_COUNTER

            if current_counter != self._last_rx_counter_seen:
                # New data arrived → build and send combined GeoChat
                combined_msg = "\n".join([
                    latest_msgs.get('gps', ''),
                    latest_msgs.get('can', ''),
                    latest_msgs.get('hdg', ''),
                    latest_msgs.get('vel', '')
                ]).strip()

                if combined_msg:
                    cot_msg = generate_chat_cot(combined_msg)
                    logging.info("Sending:\n%s", cot_msg.decode())
                    await self.tx_queue.put(cot_msg)

                    ages = _ages_ms(now_s)
                    LOGGER.log(
                        recv_epoch=now_s,
                        sent_epoch=latest_state["gps"].get("stamp"),
                        kind="chat",
                        msg_type="chat",
                        seq=latest_state["gps"].get("seq"),
                        lat=latest_gps.get("lat"),
                        lon=latest_gps.get("lon"),
                        linear_x=latest_state["vel"].get("linear_x"),
                        heading=latest_state["hdg"].get("heading"),
                        throttle=latest_state["can"].get("throttle"),
                        steering=latest_state["can"].get("steering"),
                        gps_latency_ms=latest_state["gps"].get("latency_ms"),
                        **ages,
                    )

                self._last_rx_counter_seen = current_counter

            else:
                # No new data → send a single-line message (do not resend last content)
                nodata_text = NODATA_MSG
                if LAST_ANY_RX_TS is not None:
                    gap = now_s - LAST_ANY_RX_TS
                    nodata_text = f"{NODATA_MSG} (gap {gap:.1f}s)"

                cot_msg = generate_chat_cot(nodata_text)
                logging.info("Sending (NoData):\n%s", cot_msg.decode())
                await self.tx_queue.put(cot_msg)

                ages = _ages_ms(now_s)
                LOGGER.log(
                    recv_epoch=now_s,
                    sent_epoch=None,
                    kind="nodata",
                    msg_type="chat",
                    seq=None,
                    lat=latest_gps.get("lat"),
                    lon=latest_gps.get("lon"),
                    linear_x=latest_state["vel"].get("linear_x"),
                    heading=latest_state["hdg"].get("heading"),
                    throttle=latest_state["can"].get("throttle"),
                    steering=latest_state["can"].get("steering"),
                    gps_latency_ms=latest_state["gps"].get("latency_ms"),
                    **ages,
                )

            await asyncio.sleep(1)

class GPSMemberWorker:
    """
    Send a position CoT only when a *new GPS message* has been received since the last tick.
    Otherwise, do not send any position CoT.
    """
    def __init__(self, tx_queue):
        self.tx_queue = tx_queue
        self._last_gps_counter_seen = -1

    async def run(self):
        while True:
            now_s = _now()
            lat = latest_gps.get("lat")
            lon = latest_gps.get("lon")

            if GPS_COUNTER != self._last_gps_counter_seen and lat is not None and lon is not None:
                cot_msg = generate_gps_cot(latest_gps)
                logging.info("Sending:\n%s", cot_msg.decode())
                await self.tx_queue.put(cot_msg)

                ages = _ages_ms(now_s)
                LOGGER.log(
                    recv_epoch=now_s,
                    sent_epoch=latest_state["gps"].get("stamp"),
                    kind="pos",
                    msg_type="gps",
                    seq=latest_state["gps"].get("seq"),
                    lat=lat, lon=lon,
                    linear_x=latest_state["vel"].get("linear_x"),
                    heading=latest_state["hdg"].get("heading"),
                    throttle=latest_state["can"].get("throttle"),
                    steering=latest_state["can"].get("steering"),
                    gps_latency_ms=latest_state["gps"].get("latency_ms"),
                    **ages,
                )

                self._last_gps_counter_seen = GPS_COUNTER

            # No new GPS → do not send
            await asyncio.sleep(1)

class MyReceiver:
    """Debug receiver for inbound CoT (optional)."""
    def __init__(self, rx_queue):
        self.rx_queue = rx_queue

    async def handle_data(self, data):
        print("[RX CoT]", data.decode())

    async def run(self):
        while True:
            data = await self.rx_queue.get()
            await self.handle_data(data)
