#!/usr/bin/env python
# -*- coding: utf-8 -*-

import socket, uuid, platform
import xml.etree.ElementTree as ET
import pytak
import json, os, math

config_path = os.path.expanduser("~/Downloads/pytak/catkin_ws/src/tak/src/cot_config.json")
with open(config_path, "r") as f:
    cot_config = json.load(f)

DEVICE_CALLSIGN = socket.gethostname()
DEVICE_UID = str(uuid.uuid4())
DEVICE_OS = platform.system()


# === Generate chat CoT ===
def generate_chat_cot(message: str):
    cfg = cot_config["chat"]
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", f"GeoChat.{DEVICE_UID}.{uuid.uuid4()}")
    root.set("how", cfg["event"]["how"])
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))
    ET.SubElement(root, "point", cfg["point"])
    detail = ET.SubElement(root, "detail")
    chat = ET.SubElement(detail, "__chat", {
        "chatroom": cfg["chatroom"]["name"],
        "senderCallsign": DEVICE_CALLSIGN,
        "parent": cfg["chatroom"]["parent"],
        "groupOwner": cfg["chatroom"]["groupOwner"],
        "id": cfg["chatroom"]["name"]
    })
    ET.SubElement(chat, "chatgrp", {
        "id": cfg["chatroom"]["name"],
        "uid0": DEVICE_UID,
        "uid1": cfg["chatroom"]["name"]
    })
    ET.SubElement(detail, "remarks", {
        "time": pytak.cot_time(),
        "source": DEVICE_UID,
        "to": cfg["chatroom"]["name"]
    }).text = message
    return ET.tostring(root)


# === Generate boat GPS member CoT ===
def generate_gps_cot(latest_gps):
    cfg = cot_config["gps"]
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", DEVICE_UID)
    root.set("how", cfg["event"]["how"])
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))
    ET.SubElement(root, "point", {
        "lat": str(latest_gps["lat"]),
        "lon": str(latest_gps["lon"]),
        "hae": str(latest_gps["hae"]),
        "ce": "999999", "le": "999999"
    })
    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "takv", {
        "device": "JS_05", "platform": "Python", "os": DEVICE_OS
    })
    ET.SubElement(detail, "contact", {"callsign": "JS_05"})
    ET.SubElement(detail, "__group", cfg["detail"]["group"])
    ET.SubElement(detail, "uid", {"Droid": cfg["detail"]["uid"]})
    ET.SubElement(detail, "usericon", {"iconsetpath": cfg["detail"]["iconsetpath"]})
    return ET.tostring(root)


# === Generate obstacle CoT ===
def generate_obstacle_cot(idx, x, y, z):
    cfg = cot_config["obstacle"]
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", f"obstacle-{idx}")
    root.set("how", cfg["event"]["how"])
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))
    ET.SubElement(root, "point", {
        "lat": str(x), "lon": str(y), "hae": str(z),
        "ce": "999999", "le": "999999"
    })
    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "takv", {"device": DEVICE_CALLSIGN, "platform": "Python", "os": DEVICE_OS})
    ET.SubElement(detail, "contact", {"callsign": f"obstacle{idx}"})
    ET.SubElement(detail, "__group", cfg["detail"]["group"])
    ET.SubElement(detail, "uid", {"Droid": cfg["detail"]["uid"]})
    ET.SubElement(detail, "usericon", {"iconsetpath": cfg["detail"]["iconsetpath"]})
    ET.SubElement(detail, "color", {"argb": cfg["detail"]["color"]})
    return ET.tostring(root)

# === Generate goalpoint CoT ===
def generate_goal_cot(idx, lat, lon, hae=99):
    cfg = cot_config["goalpoint"]
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", f"goalpoint-{idx}") # 確保多個目標點時 UID 不同
    root.set("how", cfg["event"]["how"])
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))

    ET.SubElement(root, "point", {
        "lat": str(lat), "lon": str(lon), "hae": str(hae),
        "ce": "999999", "le": "999999"
    })

    detail = ET.SubElement(root, "detail")
    # 設定名稱為 "目標點"
    ET.SubElement(detail, "contact", {"callsign": f"目標點-{idx}"})
    ET.SubElement(detail, "__group", cfg["detail"]["group"])
    
    # 這裡加入您要求的 Google/blu-blank.png 圖示
    ET.SubElement(detail, "usericon", {"iconsetpath": cfg["detail"]["iconsetpath"]})
    
    # 其他 TAK 必要的標記
    takv_attr = {"device": "Python-Bridge", "platform": "ROS", "os": platform.system(), "version": "1.0"}
    ET.SubElement(detail, "takv", takv_attr)
    
    return ET.tostring(root)

# === Generate start CoT ===
def generate_start_cot(idx, lat, lon, hae=99):
    cfg = cot_config["startpoint"]
    now = pytak.cot_time() # 統一名稱與開始時間，避免 TAK 顯示延遲
    
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", f"startpoint-{idx}")
    root.set("how", cfg["event"]["how"])
    root.set("time", now)
    root.set("start", now)
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))

    ET.SubElement(root, "point", {
        "lat": str(lat), "lon": str(lon), "hae": str(hae),
        "ce": "999999", "le": "999999"
    })

    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "contact", {"callsign": f"起點-{idx}"}) # 名稱修改為起點
    ET.SubElement(detail, "usericon", {"iconsetpath": cfg["detail"]["iconsetpath"]}) # 使用 pink-blank.png
    
    takv_attr = {"device": DEVICE_CALLSIGN, "platform": "Python", "os": DEVICE_OS}
    ET.SubElement(detail, "takv", takv_attr)
    
    return ET.tostring(root)

# ====== Generate Boat to Waypoint Line ======

def calculate_bearing(lat1, lon1, lat2, lon2):
    """計算兩點間的方位角 (Degrees)"""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)
    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    return (math.degrees(math.atan2(y, x)) + 360) % 360

def calculate_range(lat1, lon1, lat2, lon2):
    """使用哈維辛公式計算兩點間距離 (Meters)"""
    R = 6371000  # 地球半徑
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)
    a = math.sin(d_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(d_lambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))

# cot_utils.py 內的修改

def generate_line_cot(vessel_gps, goal_pose, idx, vessel_uid):
    """產生連接船隻與目標點的 R&B 直線 CoT"""
    v_lat, v_lon = vessel_gps["lat"], vessel_gps["lon"]
    g_lat, g_lon = goal_pose.position.x, goal_pose.position.y
    
    rng = calculate_range(v_lat, v_lon, g_lat, g_lon)
    brg = calculate_bearing(v_lat, v_lon, g_lat, g_lon)
    
    now = pytak.cot_time()

    root = ET.Element("event", version="2.0", type="u-rb-a", how="h-g-i-g-o")
    # 使用包含 vessel_uid 的唯一識別碼
    root.set("uid", f"{vessel_uid}.line.{idx}") 
    root.set("time", now)
    root.set("start", now)
    root.set("stale", pytak.cot_time(30)) 

    ET.SubElement(root, "point", {
        "lat": str(v_lat), "lon": str(v_lon), 
        "hae": "999999.0", "ce": "999999.0", "le": "999999.0"
    })

    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "contact", {"callsign": f"導引線-{idx}"})
    ET.SubElement(detail, "bearing", {"value": str(brg)})
    ET.SubElement(detail, "range", {"value": str(rng)})
    
    # 建立與船隻的關聯 (Link)
    ET.SubElement(detail, "link", {
        "relation": "p-p", 
        "type": "a-f-G-U-C-I", 
        "uid": vessel_uid
    })
    
    ET.SubElement(detail, "color", {"value": "-65536"}) 
    ET.SubElement(detail, "strokeColor", {"value": "-65536"})
    ET.SubElement(detail, "strokeWeight", {"value": "3"})
    
    return ET.tostring(root)

# ===== Generate Range Ring =====
def generate_range_ring_cot(vessel_gps, radius_meters, vessel_uid, color_hex="-65536"):
    """
    根據 WebTAK 範例產生的圓形 CoT，確保包含 Style 連結與正確的 ellipse 屬性。
    """
    v_lat, v_lon = vessel_gps["lat"], vessel_gps["lon"]
    now = pytak.cot_time()
    # 建立唯一的 UID 並確保 Style 連結使用相同的基礎
    ring_uid = f"{vessel_uid}.ring.{radius_meters}"

    root = ET.Element("event", {
        "version": "2.0",
        "uid": ring_uid,
        "type": "u-r-b-c-c",
        "how": "h-g-i-g-o",
        "time": now,
        "start": now,
        "stale": pytak.cot_time(30)
    })

    ET.SubElement(root, "point", {
        "lat": str(v_lat), "lon": str(v_lon),
        "hae": "999999.0", "ce": "999999.0", "le": "999999.0"
    })

    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "contact", {"callsign": f"R&B Circle {radius_meters}m"})
    ET.SubElement(detail, "archive", {"text": "True"}) # 範例中為 <archive>True</archive>
    
    # === Shape 定義與核心 Style 邏輯 ===
    shape = ET.SubElement(detail, "shape")
    # 範例中 angle 為 360
    ET.SubElement(shape, "ellipse", {
        "major": str(radius_meters),
        "minor": str(radius_meters),
        "angle": "360"
    })
    
    # 仿照範例中的 <link type="b-xKmlStyle" ...>
    link_style = ET.SubElement(shape, "link", {
        "type": "b-xKmlStyle",
        "uid": f"{ring_uid}.Style",
        "relation": "p-c"
    })
    style_node = ET.SubElement(link_style, "Style")
    line_style = ET.SubElement(style_node, "LineStyle")
    ET.SubElement(line_style, "color").text = color_hex
    ET.SubElement(line_style, "width").text = "3"
    ET.SubElement(line_style, "alpha").text = "0"
    
    poly_style = ET.SubElement(style_node, "PolyStyle")
    ET.SubElement(poly_style, "color").text = "00ff0000" # 透明填滿

    # === 其他外觀屬性 (確保與範例欄位一致) ===
    ET.SubElement(detail, "rangeUnits", {"value": "1"})
    ET.SubElement(detail, "color", {"argb": color_hex})
    ET.SubElement(detail, "strokeColor", {"value": color_hex})
    ET.SubElement(detail, "strokeWeight", {"value": "3"})
    ET.SubElement(detail, "fillColor", {"value": "16776960"})
    
    # 建立與船隻的關聯
    ET.SubElement(detail, "link", {
        "relation": "p-p", 
        "type": "a-f-G-U-C-I", 
        "uid": vessel_uid
    })

    return ET.tostring(root)