#!/usr/bin/env python3
import sys
import socket

# 你 Unity 的 IP & Port
FORWARD_IP = "192.168.133.15"
FORWARD_PORT = 49154

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"[FORWARD] Starting forwarder → {FORWARD_IP}:{FORWARD_PORT}")

for line in sys.stdin:
    data = line.strip()

    if not data:
        continue

    print(f"[RECV]  {data}")
    sock.sendto(data.encode(), (FORWARD_IP, FORWARD_PORT))
    print(f"[SEND] → {FORWARD_IP}:{FORWARD_PORT}")
