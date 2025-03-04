import socket
import xml.etree.ElementTree as ET

SYNC_IP = "127.0.0.1"
SYNC_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((SYNC_IP, SYNC_PORT))

print(f"[SYNC] Listening on {SYNC_IP}:{SYNC_PORT}...")

def parse_cot(xml_data):
    """Parse a CoT XML string and return a dictionary."""
    root = ET.fromstring(xml_data)

    cot_data = {
        "uid": root.get("uid"),
        "type": root.get("type"),
        "how": root.get("how"),
        "time": root.get("time"),
        "stale": root.get("stale"),
    }

    point = root.find("point")
    if point is not None:
        cot_data["position"] = {
            "lat": float(point.get("lat", 0)),
            "lon": float(point.get("lon", 0)),
            "hae": float(point.get("hae", 0)),
        }

    detail = root.find("detail")
    if detail is not None:
        contact = detail.find("contact")
        takv = detail.find("takv")
        cot_data["contact"] = contact.get("callsign") if contact is not None else ""
        cot_data["device"] = takv.get("device") if takv is not None else ""

    return cot_data

while True:
    data, addr = sock.recvfrom(65535)
    xml_str = data.decode()
    
    try:
        cot = parse_cot(xml_str)
        print(f"[SYNC] Parsed CoT:\n{cot}\n")
    except Exception as e:
        print(f"[SYNC] Error parsing CoT: {e}")

