#!/usr/bin/env python3
import socket

LISTEN_IP = "0.0.0.0"   # 監聽所有介面
LISTEN_PORT = 13550     # ← 修改成你 Unity 也要接的 Port

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))

print(f"[UDP Receiver] Listening on {LISTEN_IP}:{LISTEN_PORT} ...\n")

while True:
    data, addr = sock.recvfrom(2048)

    try:
        text = data.decode('utf-8').strip()
        print(f"[ASCII] From {addr}: {text}")
    except:
        print(f"[RAW BYTES] From {addr}: {data}")
