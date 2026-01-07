#!/usr/bin/env python3
# tak_launcher_gui.py
#
# ① 啟動 TAK 主流程 (main.py)
# ② Waypoint / Obstacle: CoT -> UDP (最多 5 個 IP，可啟動/停止、修改 IP 後重啟)
# ③ 本地搖桿 -> UDP (在 UI 顯示搖桿狀態與按鍵/軸值)

import asyncio
import struct
import time
import socket
import xml.etree.ElementTree as ET
import logging
import json
import math
import traceback
import threading
import queue

import tkinter as tk
from tkinter import ttk, messagebox

from configparser import ConfigParser
from multiprocessing import freeze_support

import pytak
import pygame

# 匯入你原本的 TAK 主流程 async main()
from main import main as tak_core_async_main  # :contentReference[oaicite:2]{index=2}

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s tak_launcher %(levelname)s - %(message)s"
)

# =======================
# ① TAK 主流程 (main.py)
# =======================
tak_core_thread = None
tak_core_running = False


def start_tak_core_thread():
    """在背景 thread 中執行 main.py 的 asyncio main()。"""
    global tak_core_thread, tak_core_running

    if tak_core_running:
        logging.info("TAK core already running.")
        return

    def runner():
        try:
            asyncio.run(tak_core_async_main())
        except Exception:
            logging.error("TAK core crashed:\n%s", traceback.format_exc())

    tak_core_thread = threading.Thread(target=runner, daemon=True)
    tak_core_thread.start()
    tak_core_running = True
    logging.info("TAK core started.")


# =====================================
# ② Waypoint / Obstacle: CoT -> UDP
#   （多 IP、可 Start/Stop）
# =====================================

WAYPOINT_HEADER = 0xAF
OBSTACLE_HEADER = 0xB0
END = 0xC0

PACKET_LENGTH_NO_TS = 25
PACKET_LENGTH_WITH_TS = 33

# 這兩個由 UI 設定
WAYPOINT_UDP_TARGETS = []   # [(ip, port), ...]
WAYPOINT_INCLUDE_TS = True

waypoint_udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

waypoint_buffer = []
obstacle_buffer = []

waypoint_thread = None
waypoint_loop = None
waypoint_running = False


def waypoint_parse_cot(xml_data: str):
    root = ET.fromstring(xml_data)

    event_type = root.get("type", "Unknown")

    point = root.find("point")
    lat = float(point.get("lat", 0)) if point is not None else 0.0
    lon = float(point.get("lon", 0)) if point is not None else 0.0
    hae = float(point.get("hae", 0)) if point is not None else 0.0

    contact = root.find("./detail/contact")
    callsign = contact.get("callsign", "Unknown") if contact is not None else "Unknown"

    return callsign, event_type, lat, lon, hae


def waypoint_send_udp(index: int, lat: float, lon: float, hae: float, header: int):
    """
    封包格式 (little-endian):
      不含時間戳:
        < B   B       B     d     d     d    B >
          hdr len(payload) idx   lat   lon   hae  END
      含時間戳:
        < B   B       B     d     d     d     d      B >
          hdr len(payload) idx   lat   lon   hae  ts(sec) END
    """
    global WAYPOINT_UDP_TARGETS, WAYPOINT_INCLUDE_TS

    if not WAYPOINT_UDP_TARGETS:
        print("[WAYPOINT] No UDP targets, skip sending.")
        return

    if WAYPOINT_INCLUDE_TS:
        payload_len = PACKET_LENGTH_WITH_TS
        ts = float(time.time())
        pack_fmt = "<BBBddddB"
        packet = struct.pack(pack_fmt, header, payload_len, index, lat, lon, hae, ts, END)
    else:
        payload_len = PACKET_LENGTH_NO_TS
        pack_fmt = "<BBBdddB"
        packet = struct.pack(pack_fmt, header, payload_len, index, lat, lon, hae, END)

    label = "WAYPOINT" if header == WAYPOINT_HEADER else "OBSTACLE"

    for ip, port in WAYPOINT_UDP_TARGETS:
        try:
            waypoint_udp_sock.sendto(packet, (ip, port))
            if WAYPOINT_INCLUDE_TS:
                print(
                    f"[UDP] Sent {label}{index} -> {ip}:{port}: "
                    f"lat={lat:.6f}, lon={lon:.6f}, hae={hae:.2f}, ts={ts:.3f}"
                )
            else:
                print(
                    f"[UDP] Sent {label}{index} -> {ip}:{port}: "
                    f"lat={lat:.6f}, lon={lon:.6f}, hae={hae:.2f}"
                )
        except Exception as e:
            print(f"[ERROR] UDP send to {ip}:{port} failed: {e}")


class WaypointCotReceiver(pytak.QueueWorker):
    """接收 TAK CoT 並轉成 waypoint / obstacle 的 UDP 封包"""

    async def handle_data(self, data):
        try:
            xml_str = data.decode()
            callsign, event_type, lat, lon, hae = waypoint_parse_cot(xml_str)

            # 只處理你指定的型別（例：b-m-p-s-m）
            if event_type != "b-m-p-s-m":
                return

            cls = callsign.lower()

            if cls.startswith("waypoint"):
                waypoint_buffer.append((lat, lon, hae))
                # callsign 以 "!" 結尾視為 flush
                if callsign.endswith("!"):
                    for idx, (la, lo, h) in enumerate(waypoint_buffer):
                        waypoint_send_udp(idx, la, lo, h, WAYPOINT_HEADER)
                    print(f"[WAYPOINT] Published {len(waypoint_buffer)} waypoints via UDP\n")
                    waypoint_buffer.clear()

            elif cls.startswith("obstacle"):
                obstacle_buffer.append((lat, lon, hae))
                if callsign.endswith("!"):
                    for idx, (la, lo, h) in enumerate(obstacle_buffer):
                        waypoint_send_udp(idx, la, lo, h, OBSTACLE_HEADER)
                    print(f"[WAYPOINT] Published {len(obstacle_buffer)} obstacles via UDP\n")
                    obstacle_buffer.clear()

        except Exception as e:
            print(f"[ERROR] Failed to parse/send CoT: {e}")

    async def run(self):
        while True:
            data = await self.queue.get()
            await self.handle_data(data)


async def waypoint_async_main():
    """獨立的 pytak CLITool，用來收 CoT 並轉發為 UDP waypoint / obstacle。"""
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://140.113.148.80:8089",
        "PYTAK_TLS_CLIENT_CERT": "argtest2_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "argtest2_key.pem",
        "PYTAK_TLS_CA_CERT": "argtest2-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1",
    }
    cfg = config["mycottool"]

    clitool = pytak.CLITool(cfg)
    await clitool.setup()

    clitool.add_tasks({WaypointCotReceiver(clitool.rx_queue, cfg)})
    await clitool.run()


def start_waypoint_thread():
    """在背景 thread 裡跑 waypoint_async_main（可透過 stop_waypoint_thread() 停止）。"""
    global waypoint_thread, waypoint_loop, waypoint_running

    if waypoint_running:
        logging.info("Waypoint forwarder already running.")
        return

    def runner():
        global waypoint_loop
        try:
            waypoint_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(waypoint_loop)
            waypoint_loop.run_until_complete(waypoint_async_main())
        except RuntimeError as e:
            # 常見於 loop.stop 後的 "Event loop stopped before Future completed"
            logging.warning("Waypoint loop stopped: %s", e)
        except Exception:
            logging.error("Waypoint forwarder crashed:\n%s", traceback.format_exc())
        finally:
            if waypoint_loop is not None:
                waypoint_loop.close()
                waypoint_loop = None
            logging.info("Waypoint thread exit.")

    waypoint_thread = threading.Thread(target=runner, daemon=True)
    waypoint_thread.start()
    waypoint_running = True
    logging.info("Waypoint forwarder started.")


def stop_waypoint_thread():
    """停止 waypoint asyncio 迴圈並等待 thread 結束。"""
    global waypoint_thread, waypoint_loop, waypoint_running

    if not waypoint_running:
        return

    if waypoint_loop is not None:
        try:
            waypoint_loop.call_soon_threadsafe(waypoint_loop.stop)
        except Exception:
            logging.error("Failed to stop waypoint loop:\n%s", traceback.format_exc())

    if waypoint_thread is not None:
        waypoint_thread.join(timeout=3.0)

    waypoint_thread = None
    waypoint_loop = None
    waypoint_running = False
    logging.info("Waypoint forwarder stopped.")


# ======================================
# ③ 本地搖桿 → UDP（顯示狀態與數值）
#    邏輯來自 sender_joystick_udp.py
# ======================================

joystick_thread = None
joystick_stop_event = threading.Event()
joystick_running = False
joystick_ui_queue: "queue.Queue[dict]" = queue.Queue()


def ewma(prev, cur, alpha=0.25):
    return cur if prev is None else alpha * cur + (1 - alpha) * prev


def apply_deadzone(v, dz=0.08):
    if abs(v) < dz:
        return 0.0
    s = (abs(v) - dz) / (1 - dz)
    s = max(0.0, min(1.0, s))
    return math.copysign(s, v)


def joystick_open(idx):
    try:
        if pygame.joystick.get_count() <= idx:
            msg = f"[ERR] No joystick at index {idx}. Detected={pygame.joystick.get_count()}"
            print(msg)
            joystick_ui_queue.put({
                "type": "status",
                "connected": False,
                "message": msg,
                "device": "",
                "axes": [],
                "buttons": [],
                "hats": []
            })
            return None
        j = pygame.joystick.Joystick(idx)
        j.init()
        msg = (f"[INFO] Using joystick [{idx}]: {j.get_name()} | "
               f"axes={j.get_numaxes()} buttons={j.get_numbuttons()} hats={j.get_numhats()}")
        print(msg)
        joystick_ui_queue.put({
            "type": "status",
            "connected": True,
            "message": msg,
            "device": j.get_name(),
            "axes": [],
            "buttons": [],
            "hats": []
        })
        return j
    except Exception:
        msg = "[EXC] open_joystick failed:\n" + traceback.format_exc()
        print(msg)
        joystick_ui_queue.put({
            "type": "status",
            "connected": False,
            "message": msg,
            "device": "",
            "axes": [],
            "buttons": [],
            "hats": []
        })
        return None


def joystick_worker(config: dict, stop_event: threading.Event, ui_queue: queue.Queue):
    """
    背景 thread 版本的 sender_joystick_udp.main()
    config: {
        "ip", "port", "hz", "deadzone", "alpha", "joy_index", "broadcast"
    }
    """
    ip = config["ip"]
    port = config["port"]
    hz = config["hz"]
    deadzone = config["deadzone"]
    alpha = config["alpha"]
    joy_index = config["joy_index"]
    broadcast = config["broadcast"]

    try:
        pygame.init()
        pygame.joystick.init()
    except Exception:
        msg = "[EXC] pygame init failed:\n" + traceback.format_exc()
        print(msg)
        ui_queue.put({
            "type": "status",
            "connected": False,
            "message": msg,
            "device": "",
            "axes": [],
            "buttons": [],
            "hats": []
        })
        return

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    if broadcast:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    target = (ip, port)

    j = joystick_open(joy_index)
    last_axes = None
    last_btns = None
    interval = 1.0 / max(1, hz)
    last_send = 0.0

    print(f"[INFO] Joystick UDP Target {target}  |  Hz={hz}  deadzone={deadzone}  alpha={alpha}")
    ui_queue.put({
        "type": "status",
        "connected": (j is not None and j.get_init() if j else False),
        "message": f"Target={target}, Hz={hz}, deadzone={deadzone}, alpha={alpha}",
        "device": j.get_name() if j else "",
        "axes": [],
        "buttons": [],
        "hats": []
    })

    try:
        while not stop_event.is_set():
            try:
                # 事件循環（必要，否則搖桿不會更新）
                pygame.event.pump()
                for _ in pygame.event.get():
                    pass

                if j is None or not j.get_init():
                    # 試著重新打開
                    time.sleep(0.5)
                    j = joystick_open(joy_index)
                    continue

                now = time.time()
                if now - last_send >= interval:
                    axes = []
                    for i in range(j.get_numaxes()):
                        v = apply_deadzone(j.get_axis(i), deadzone)
                        v = ewma(last_axes[i] if (last_axes and i < len(last_axes)) else None, v, alpha)
                        axes.append(v)
                    btns = [int(j.get_button(i)) for i in range(j.get_numbuttons())]
                    hats = [j.get_hat(i) for i in range(j.get_numhats())]

                    payload = {
                        "ts_unix_ms": int(now * 1000),
                        "device": j.get_name(),
                        "index": joy_index,
                        "axes": axes,
                        "buttons": btns,
                        "hats": hats,
                        "meta": {"hz": hz, "deadzone": deadzone, "alpha": alpha},
                    }
                    data = json.dumps(payload).encode("utf-8")

                    try:
                        sock.sendto(data, target)
                    except OSError as e:
                        msg = (f"[SOCK ERR] {e}  ip={ip} port={port}  "
                               f"(Tip: use broadcast for 255.255.255.255)")
                        print(msg)
                        ui_queue.put({
                            "type": "status",
                            "connected": True,
                            "message": msg,
                            "device": j.get_name(),
                            "axes": axes,
                            "buttons": btns,
                            "hats": hats,
                        })

                    last_send = now

                    # 推送狀態到 UI
                    ui_queue.put({
                        "type": "data",
                        "connected": True,
                        "message": "",
                        "device": j.get_name(),
                        "axes": axes,
                        "buttons": btns,
                        "hats": hats,
                    })

                    last_axes = axes
                    last_btns = btns

                time.sleep(0.001)

            except KeyboardInterrupt:
                print("[INFO] Joystick loop interrupted by user.")
                break
            except Exception:
                msg = "[EXC] Joystick loop exception:\n" + traceback.format_exc()
                print(msg)
                ui_queue.put({
                    "type": "status",
                    "connected": j is not None and j.get_init() if j else False,
                    "message": msg,
                    "device": j.get_name() if j else "",
                    "axes": [],
                    "buttons": [],
                    "hats": [],
                })
                time.sleep(0.5)

    finally:
        pygame.quit()
        print("[INFO] Joystick worker exit.")
        ui_queue.put({
            "type": "status",
            "connected": False,
            "message": "Joystick worker stopped.",
            "device": "",
            "axes": [],
            "buttons": [],
            "hats": [],
        })


def start_joystick_thread(config: dict):
    global joystick_thread, joystick_stop_event, joystick_running, joystick_ui_queue

    if joystick_running:
        logging.info("Joystick sender already running.")
        return

    joystick_stop_event = threading.Event()
    joystick_ui_queue = queue.Queue()

    def runner():
        joystick_worker(config, joystick_stop_event, joystick_ui_queue)

    joystick_thread = threading.Thread(target=runner, daemon=True)
    joystick_thread.start()
    joystick_running = True
    logging.info("Joystick sender started.")


def stop_joystick_thread():
    global joystick_thread, joystick_stop_event, joystick_running

    if not joystick_running:
        return

    joystick_stop_event.set()
    if joystick_thread is not None:
        joystick_thread.join(timeout=3.0)
    joystick_thread = None
    joystick_running = False
    logging.info("Joystick sender stopped.")


# ======================
# Tkinter App
# ======================

class TakLauncherApp(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("TAK Launcher GUI")
        self.geometry("720x620")

        # Notebook 內三個頁籤：1.main.py, 2.waypoint, 3.joystick
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=10, pady=10)

        # Tab1: TAK main.py
        self.tab_main = ttk.Frame(nb)
        nb.add(self.tab_main, text="① TAK 主流程 (main.py)")

        # Tab2: Waypoint/Obstacle
        self.tab_waypoint = ttk.Frame(nb)
        nb.add(self.tab_waypoint, text="② Waypoint / Obstacle COT→UDP")

        # Tab3: Local Joystick
        self.tab_joystick = ttk.Frame(nb)
        nb.add(self.tab_joystick, text="③ 本地搖桿 → UDP")

        self.build_tab_main()
        self.build_tab_waypoint()
        self.build_tab_joystick()

        # Joystick UI 監聽
        self.after(100, self.poll_joystick_ui)

    # ---------- TAB 1: TAK main.py ----------
    def build_tab_main(self):
        frame = self.tab_main

        lbl = tk.Label(frame, text="啟動 TAK 主流程（main.py）", font=("Microsoft JhengHei", 12, "bold"))
        lbl.pack(pady=10)

        desc = tk.Label(
            frame,
            text="按下下面按鈕後，會在背景啟動 main.py（ROS ↔ UDP ↔ TAK ↔ Unity 整合流程）。\n"
                 "此流程啟動後通常不需要在這裡停止，如需關閉請直接關閉整個程式。",
            justify="left",
        )
        desc.pack(pady=5)

        self.btn_start_tak = tk.Button(
            frame,
            text="啟動 TAK 主流程",
            width=20,
            command=self.on_start_tak
        )
        self.btn_start_tak.pack(pady=10)

        self.lbl_tak_status = tk.Label(frame, text="狀態：尚未啟動", fg="red")
        self.lbl_tak_status.pack(pady=5)

    def on_start_tak(self):
        global tak_core_running
        if tak_core_running:
            messagebox.showinfo("提示", "TAK 主流程已經在執行中。")
            return
        start_tak_core_thread()
        self.lbl_tak_status.config(text="狀態：執行中", fg="green")
        self.btn_start_tak.config(state="disabled")

    # ---------- TAB 2: Waypoint / Obstacle ----------
    def build_tab_waypoint(self):
        frame = self.tab_waypoint

        title = tk.Label(frame, text="Waypoint / Obstacle: CoT → UDP 轉發", font=("Microsoft JhengHei", 12, "bold"))
        title.pack(pady=10)

        info = tk.Label(
            frame,
            text="此模組會從 TAK 收到的 CoT 中解析 waypoint / obstacle，並以 UDP 發送給最多 5 個目標 IP。\n"
                 "你可以停止後修改 IP / Port，再重新啟動。",
            justify="left",
        )
        info.pack(pady=5)

        # IP 列表
        self.waypoint_ip_entries = []
        ip_frame = tk.Frame(frame)
        ip_frame.pack(fill="x", padx=20, pady=5)

        tk.Label(ip_frame, text="UDP 目標 IP（最多 5 個）：").pack(anchor="w")

        for i in range(5):
            row = tk.Frame(ip_frame)
            row.pack(fill="x", pady=2)
            tk.Label(row, text=f"IP {i + 1}:", width=6, anchor="e").pack(side="left")
            entry = tk.Entry(row)
            entry.pack(side="left", fill="x", expand=True)
            self.waypoint_ip_entries.append(entry)

        # Port
        port_frame = tk.Frame(frame)
        port_frame.pack(fill="x", padx=20, pady=5)
        tk.Label(port_frame, text="Port:", width=6, anchor="e").pack(side="left")
        self.waypoint_port_entry = tk.Entry(port_frame, width=10)
        self.waypoint_port_entry.insert(0, "49157")
        self.waypoint_port_entry.pack(side="left")

        # Timestamp checkbox
        self.waypoint_ts_var = tk.BooleanVar(value=True)
        cb = tk.Checkbutton(frame, text="封包中附帶 timestamp (UNIX seconds)", variable=self.waypoint_ts_var)
        cb.pack(pady=5)

        # Buttons
        btn_frame = tk.Frame(frame)
        btn_frame.pack(pady=10)

        self.btn_waypoint_start = tk.Button(
            btn_frame, text="啟動 Waypoint 轉發", width=18, command=self.on_start_waypoint
        )
        self.btn_waypoint_start.pack(side="left", padx=5)

        self.btn_waypoint_stop = tk.Button(
            btn_frame, text="停止 Waypoint 轉發", width=18, command=self.on_stop_waypoint, state="disabled"
        )
        self.btn_waypoint_stop.pack(side="left", padx=5)

        self.lbl_waypoint_status = tk.Label(frame, text="狀態：尚未啟動", fg="red")
        self.lbl_waypoint_status.pack(pady=5)

    def on_start_waypoint(self):
        global WAYPOINT_UDP_TARGETS, WAYPOINT_INCLUDE_TS

        # 收集 IP/Port
        try:
            port = int(self.waypoint_port_entry.get())
        except ValueError:
            messagebox.showerror("錯誤", "Port 必須是數字")
            return

        targets = []
        for e in self.waypoint_ip_entries:
            ip = e.get().strip()
            if ip:
                targets.append((ip, port))

        if not targets:
            messagebox.showerror("錯誤", "請至少輸入一個 IP")
            return
        if len(targets) > 5:
            messagebox.showerror("錯誤", "最多只能設定 5 個 IP")
            return

        WAYPOINT_UDP_TARGETS = targets
        WAYPOINT_INCLUDE_TS = self.waypoint_ts_var.get()

        start_waypoint_thread()

        # 鎖定輸入，避免啟動中被修改
        self.btn_waypoint_start.config(state="disabled")
        self.btn_waypoint_stop.config(state="normal")
        for e in self.waypoint_ip_entries:
            e.config(state="disabled")
        self.waypoint_port_entry.config(state="disabled")

        self.lbl_waypoint_status.config(
            text=f"狀態：執行中（Targets={WAYPOINT_UDP_TARGETS}）",
            fg="green"
        )

    def on_stop_waypoint(self):
        stop_waypoint_thread()

        # 解鎖輸入
        self.btn_waypoint_start.config(state="normal")
        self.btn_waypoint_stop.config(state="disabled")
        for e in self.waypoint_ip_entries:
            e.config(state="normal")
        self.waypoint_port_entry.config(state="normal")

        self.lbl_waypoint_status.config(text="狀態：已停止，可修改 IP 後重新啟動", fg="orange")

    # ---------- TAB 3: Joystick ----------
    def build_tab_joystick(self):
        frame = self.tab_joystick

        title = tk.Label(frame, text="本地搖桿 → UDP 發送（並在此顯示搖桿狀態）", font=("Microsoft JhengHei", 12, "bold"))
        title.pack(pady=10)

        info = tk.Label(
            frame,
            text="此模組會在本機讀取搖桿（pygame），並以 UDP JSON 封包發送出去。\n"
                 "下方會顯示搖桿是否有讀取成功、目前軸值與按鍵狀態。",
            justify="left",
        )
        info.pack(pady=5)

        # 連線設定
        net_frame = tk.LabelFrame(frame, text="連線設定")
        net_frame.pack(fill="x", padx=10, pady=5)

        row1 = tk.Frame(net_frame)
        row1.pack(fill="x", pady=3)
        tk.Label(row1, text="目標 IP:", width=8, anchor="e").pack(side="left")
        self.joy_ip_entry = tk.Entry(row1, width=20)
        self.joy_ip_entry.insert(0, "140.113.148.99")
        self.joy_ip_entry.pack(side="left")

        row2 = tk.Frame(net_frame)
        row2.pack(fill="x", pady=3)
        tk.Label(row2, text="Port:", width=8, anchor="e").pack(side="left")
        self.joy_port_entry = tk.Entry(row2, width=10)
        self.joy_port_entry.insert(0, "9999")
        self.joy_port_entry.pack(side="left")

        row3 = tk.Frame(net_frame)
        row3.pack(fill="x", pady=3)
        tk.Label(row3, text="Joystick Index:", width=12, anchor="e").pack(side="left")
        self.joy_index_entry = tk.Entry(row3, width=5)
        self.joy_index_entry.insert(0, "0")
        self.joy_index_entry.pack(side="left")

        row4 = tk.Frame(net_frame)
        row4.pack(fill="x", pady=3)
        tk.Label(row4, text="頻率 Hz:", width=12, anchor="e").pack(side="left")
        self.joy_hz_entry = tk.Entry(row4, width=5)
        self.joy_hz_entry.insert(0, "60")
        self.joy_hz_entry.pack(side="left")

        row5 = tk.Frame(net_frame)
        row5.pack(fill="x", pady=3)
        tk.Label(row5, text="Deadzone:", width=12, anchor="e").pack(side="left")
        self.joy_deadzone_entry = tk.Entry(row5, width=5)
        self.joy_deadzone_entry.insert(0, "0.12")
        self.joy_deadzone_entry.pack(side="left")

        row6 = tk.Frame(net_frame)
        row6.pack(fill="x", pady=3)
        tk.Label(row6, text="Alpha:", width=12, anchor="e").pack(side="left")
        self.joy_alpha_entry = tk.Entry(row6, width=5)
        self.joy_alpha_entry.insert(0, "1.0")
        self.joy_alpha_entry.pack(side="left")

        self.joy_broadcast_var = tk.BooleanVar(value=False)
        cb_broadcast = tk.Checkbutton(net_frame, text="啟用 UDP Broadcast (255.255.255.255)", variable=self.joy_broadcast_var)
        cb_broadcast.pack(anchor="w", padx=5, pady=3)

        # 按鈕
        btn_frame = tk.Frame(frame)
        btn_frame.pack(pady=8)

        self.btn_joy_start = tk.Button(
            btn_frame, text="啟動搖桿發送", width=16, command=self.on_start_joystick
        )
        self.btn_joy_start.pack(side="left", padx=5)

        self.btn_joy_stop = tk.Button(
            btn_frame, text="停止搖桿發送", width=16, command=self.on_stop_joystick, state="disabled"
        )
        self.btn_joy_stop.pack(side="left", padx=5)

        # 狀態與數值
        status_frame = tk.LabelFrame(frame, text="搖桿狀態與數值")
        status_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.lbl_joy_status = tk.Label(status_frame, text="狀態：尚未啟動", fg="red", anchor="w", justify="left")
        self.lbl_joy_status.pack(fill="x", pady=3)

        self.lbl_joy_device = tk.Label(status_frame, text="裝置：", anchor="w", justify="left")
        self.lbl_joy_device.pack(fill="x", pady=3)

        self.lbl_joy_axes = tk.Label(status_frame, text="Axes：", anchor="w", justify="left")
        self.lbl_joy_axes.pack(fill="x", pady=3)

        self.lbl_joy_buttons = tk.Label(status_frame, text="Buttons：", anchor="w", justify="left")
        self.lbl_joy_buttons.pack(fill="x", pady=3)

        self.lbl_joy_hats = tk.Label(status_frame, text="Hats：", anchor="w", justify="left")
        self.lbl_joy_hats.pack(fill="x", pady=3)

    def on_start_joystick(self):
        global joystick_running

        if joystick_running:
            messagebox.showinfo("提示", "搖桿發送已在執行中。")
            return

        # 讀取設定
        ip = self.joy_ip_entry.get().strip()
        if not ip:
            messagebox.showerror("錯誤", "請輸入目標 IP")
            return
        try:
            port = int(self.joy_port_entry.get())
        except ValueError:
            messagebox.showerror("錯誤", "Port 必須是數字")
            return
        try:
            joy_idx = int(self.joy_index_entry.get())
        except ValueError:
            messagebox.showerror("錯誤", "Joystick Index 必須是數字")
            return
        try:
            hz = int(self.joy_hz_entry.get())
        except ValueError:
            messagebox.showerror("錯誤", "Hz 必須是數字")
            return
        try:
            dz = float(self.joy_deadzone_entry.get())
            alpha = float(self.joy_alpha_entry.get())
        except ValueError:
            messagebox.showerror("錯誤", "Deadzone / Alpha 必須是數字")
            return

        cfg = {
            "ip": ip,
            "port": port,
            "hz": hz,
            "deadzone": dz,
            "alpha": alpha,
            "joy_index": joy_idx,
            "broadcast": self.joy_broadcast_var.get(),
        }

        # 啟動背景 thread
        start_joystick_thread(cfg)

        # UI 更新
        self.btn_joy_start.config(state="disabled")
        self.btn_joy_stop.config(state="normal")
        self.lbl_joy_status.config(text="狀態：執行中，等待搖桿資料...", fg="green")

    def on_stop_joystick(self):
        stop_joystick_thread()
        self.btn_joy_start.config(state="normal")
        self.btn_joy_stop.config(state="disabled")
        self.lbl_joy_status.config(text="狀態：已停止", fg="orange")

    def poll_joystick_ui(self):
        """
        定期從 joystick_ui_queue 取出訊息，更新 UI。
        """
        global joystick_ui_queue

        try:
            while True:
                msg = joystick_ui_queue.get_nowait()
                if msg["type"] == "status":
                    connected = msg["connected"]
                    text = msg["message"] or ("搖桿連線成功" if connected else "尚未偵測到搖桿")
                    self.lbl_joy_status.config(
                        text=f"狀態：{'已連線' if connected else '未連線'} - " + text,
                        fg=("green" if connected else "red"),
                    )
                    if msg["device"]:
                        self.lbl_joy_device.config(
                            text=f"裝置：{msg['device']}"
                        )
                    else:
                        self.lbl_joy_device.config(text="裝置：")

                    # 清空數值
                    if not connected:
                        self.lbl_joy_axes.config(text="Axes：")
                        self.lbl_joy_buttons.config(text="Buttons：")
                        self.lbl_joy_hats.config(text="Hats：")

                elif msg["type"] == "data":
                    axes = msg["axes"]
                    btns = msg["buttons"]
                    hats = msg["hats"]
                    dev = msg["device"]

                    self.lbl_joy_status.config(
                        text="狀態：已連線並傳送中",
                        fg="green",
                    )
                    self.lbl_joy_device.config(text=f"裝置：{dev}")

                    axes_str = ", ".join(f"{i}:{v:.2f}" for i, v in enumerate(axes))
                    btn_str = " ".join(str(b) for b in btns)
                    hats_str = ", ".join(str(h) for h in hats)

                    self.lbl_joy_axes.config(text=f"Axes：{axes_str}")
                    self.lbl_joy_buttons.config(text=f"Buttons：{btn_str}")
                    self.lbl_joy_hats.config(text=f"Hats：{hats_str}")

        except queue.Empty:
            pass

        # 每 100ms poll 一次
        self.after(100, self.poll_joystick_ui)


if __name__ == "__main__":
    freeze_support()  # PyInstaller + multiprocessing 在 Windows 必備
    app = TakLauncherApp()
    app.mainloop()
