import socket, uuid, platform
import xml.etree.ElementTree as ET
import pytak
import json, os

config_path = os.path.expanduser("cot_config.json")
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
def generate_goal_cot(idx, lat, lon, hae):
    cfg = cot_config["goalpoint"]
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", f"goalpoint-{idx}")
    root.set("how", cfg["event"]["how"])
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))
    ET.SubElement(root, "point", {
        "lat": str(lat), "lon": str(lon), "hae": str(hae),
        "ce": "999999", "le": "999999"
    })
    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "takv", {"device": DEVICE_CALLSIGN, "platform": "Python", "os": DEVICE_OS})
    ET.SubElement(detail, "contact", {"callsign": f"goalpoint-{idx}"})
    ET.SubElement(detail, "__group", cfg["detail"]["group"])
    ET.SubElement(detail, "uid", {"Droid": cfg["detail"]["uid"]})
    ET.SubElement(detail, "usericon", {"iconsetpath": cfg["detail"]["iconsetpath"]})
    ET.SubElement(detail, "color", {"argb": cfg["detail"]["color"]})
    return ET.tostring(root)

# === Generate start CoT ===
def generate_start_cot(idx, lat, lon, hae):
    cfg = cot_config["startpoint"]
    root = ET.Element("event")
    root.set("version", cfg["event"]["version"])
    root.set("type", cfg["event"]["type"])
    root.set("uid", f"startpoint-{idx}")
    root.set("how", cfg["event"]["how"])
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(cfg["event"]["stale_seconds"]))
    ET.SubElement(root, "point", {
        "lat": str(lat), "lon": str(lon), "hae": str(hae),
        "ce": "999999", "le": "999999"
    })
    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "takv", {"device": DEVICE_CALLSIGN, "platform": "Python", "os": DEVICE_OS})
    ET.SubElement(detail, "contact", {"callsign": f"startpoint-{idx}"})
    ET.SubElement(detail, "__group", cfg["detail"]["group"])
    ET.SubElement(detail, "uid", {"Droid": cfg["detail"]["uid"]})
    ET.SubElement(detail, "usericon", {"iconsetpath": cfg["detail"]["iconsetpath"]})
    ET.SubElement(detail, "color", {"argb": cfg["detail"]["color"]})
    return ET.tostring(root)
