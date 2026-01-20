#!/usr/bin/env python3
import socket
import json
import time
import argparse
import rospy
from sensor_msgs.msg import Joy

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="0.0.0.0", help="UDP bind IP")
    parser.add_argument("--port", type=int, default=9999, help="UDP bind port")
    parser.add_argument("--frame_id", default="/udp_joystick", help="Joy.header.frame_id")
    parser.add_argument("--rate", type=float, default=60.0, help="Publish rate (Hz)")
    args = parser.parse_args()

    rospy.init_node("udp2joy_bridge")
    # 發佈到 /joy_win，這是你後續 ROS 控制節點訂閱的主題
    pub = rospy.Publisher("/joy_win", Joy, queue_size=1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    sock.setblocking(False)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1<<20)

    last_msg = None
    last_recv_time = 0.0
    keepalive_interval = 1.0 / max(1.0, args.rate)

    rospy.loginfo(f"UDP Bridge started on {args.ip}:{args.port}")

    while not rospy.is_shutdown():
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
            except (BlockingIOError, socket.error):
                break
            except Exception as e:
                rospy.logwarn_throttle(2.0, f"Bad packet: {e}")
                break

        now = time.time()
        joy = Joy()
        joy.header.stamp = rospy.Time.now()
        joy.header.frame_id = args.frame_id
        
        # 初始化標準長度 (ROS 標準通常為 8 軸 11 鍵)
        axes = [0.0] * 8
        buttons = [0] * 11

        if last_msg is not None:
            # 取得從 js_gui.py 傳過來的原始列表
            r_axes = last_msg.get("axes", [])
            r_btns = last_msg.get("buttons", [])
            r_hats = last_msg.get("hats", [])

            # --- 一個一個對應 Axes (類比搖桿) ---
            if len(r_axes) >= 6:
                axes[0] = float(r_axes[0]) # 左搖桿 左右 (Steering)
                axes[1] = -float(r_axes[1]) # 左搖桿 上下 (Throttle)
                axes[2] = float(r_axes[4])
                axes[3] = float(r_axes[2]) # 右搖桿 左右
                axes[4] = -float(r_axes[3])
                axes[5] = float(r_axes[5])

            if int(r_btns[11]) == 1: 
                axes[7] = int(r_btns[11]) # 十字鍵 左右
            if int(r_btns[14]) == 1:
                axes[6] = int(r_btns[14]) # 十字鍵 上下
            if int(r_btns[12]) == 1:
                axes[7] = -1
            if int(r_btns[13]) == 1:
                axes[6] = -1

            # --- 一個一個對應 Buttons (按鍵) ---
            # 這裡根據 js_gui.py 傳來的順序明確賦值
            if len(r_btns) >= 11:
                buttons[0] = int(r_btns[0])  # A 鍵
                buttons[1] = int(r_btns[1])  # B 鍵
                buttons[2] = int(r_btns[2])  # X 鍵
                buttons[3] = int(r_btns[3])  # Y 鍵
                buttons[4] = int(r_btns[9])  # LB
                buttons[5] = int(r_btns[10])  # RB
                buttons[6] = int(r_btns[4])  # Back / Select
                buttons[7] = int(r_btns[6])  # Start
                buttons[8] = int(r_btns[15])  # Logi/Guide 鍵
                buttons[9] = int(r_btns[7])  # 左搖桿下壓 (L3)
                buttons[10] = int(r_btns[8]) # 右搖桿下壓 (R3)

        joy.axes = axes
        joy.buttons = buttons

        # 發送邏輯：收到新包立即發，或是超時發送 keep-alive
        if got_any or (now - last_recv_time) > keepalive_interval:
            pub.publish(joy)
            if not got_any:
                last_recv_time = now

        rospy.sleep(0.001)

if __name__ == "__main__":
    main()

