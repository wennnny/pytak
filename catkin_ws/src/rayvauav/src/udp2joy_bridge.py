#!/usr/bin/env python3
import socket
import json
import time
import argparse

import rospy
from sensor_msgs.msg import Joy

def clamp(v, lo, hi): return max(lo, min(hi, v))

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="0.0.0.0", help="UDP bind IP")
    parser.add_argument("--port", type=int, default=9999, help="UDP bind port")
    parser.add_argument("--frame_id", default="/udp_joystick", help="Joy.header.frame_id")
    parser.add_argument("--rate", type=float, default=60.0, help="Publish rate (Hz)")
    args = parser.parse_args()

    rospy.init_node("udp2joy_bridge")
    pub = rospy.Publisher("/joy_win", Joy, queue_size=1)  # 避免排隊

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    sock.setblocking(False)                           # 非阻塞
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1<<20)  # 放大接收緩衝

    last_msg = None
    last_recv_time = 0.0
    keepalive_interval = 1.0 / max(1.0, args.rate)

    while not rospy.is_shutdown():
        # 盡可能把最新一包取出，丟掉舊的
        got_any = False
        while True:
            try:
                data, addr = sock.recvfrom(65535)
                msg = json.loads(data.decode("utf-8"))
                if isinstance(msg, dict) and msg.get("type") == "heartbeat":
                    continue
                last_msg = msg
                last_recv_time = time.time()
                got_any = True
            except BlockingIOError:
                break
            except Exception as e:
                rospy.logwarn_throttle(2.0, f"bad packet: {e}")
                break

        now = time.time()
        joy = Joy()
        joy.header.stamp = rospy.Time.now()           # 用本機時間蓋章 → 不受對時影響
        joy.header.frame_id = args.frame_id

        if last_msg is not None:
            axes = list(last_msg.get("axes") or [])
            hats = last_msg.get("hats") or []
            hatx, haty = 0.0, 0.0
            if isinstance(hats, list) and hats and len(hats[0]) == 2:
                hatx, haty = float(hats[0][0]), float(hats[0][1])
            while len(axes) < 8:
                axes.append(0.0)
            axes[6], axes[7] = hatx, haty

            buttons = list(last_msg.get("buttons") or [])
            while len(buttons) < 11:
                buttons.append(0)
        else:
            axes = [0.0]*8
            buttons = [0]*11

        joy.axes = axes
        joy.buttons = buttons

        # 有新包就立即發；沒有新包則每 keepalive_interval 發一次 keep-alive
        if got_any or (now - last_recv_time) > keepalive_interval:
            pub.publish(joy)
            if not got_any:
                last_recv_time = now

        rospy.sleep(0.001)


if __name__ == "__main__":
    main()
