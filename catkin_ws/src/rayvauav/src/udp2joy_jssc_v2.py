#!/usr/bin/env python3
import socket
import json
import time
import argparse
import rospy
from sensor_msgs.msg import Joy

def safe_btn(btns, i):
    return int(btns[i]) if i < len(btns) else 0

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", default="0.0.0.0", help="UDP bind IP")
    parser.add_argument("--port", type=int, default=9999, help="UDP bind port")
    parser.add_argument("--frame_id", default="/udp_joystick", help="Joy.header.frame_id")
    parser.add_argument("--rate", type=float, default=60.0, help="Publish rate (Hz)")
    args, unknown = parser.parse_known_args()

    rospy.init_node("udp2joy_bridge")
    pub = rospy.Publisher("/js1_manual/joy", Joy, queue_size=1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    sock.setblocking(False)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1 << 20)

    last_msg = None
    last_recv_time = 0.0
    keepalive_interval = 1.0 / max(1.0, args.rate)

    rospy.loginfo(f"UDP Bridge started on {args.ip}:{args.port}")

    while not rospy.is_shutdown():
        got_any = False

        while True:
            try:
                data, _ = sock.recvfrom(65535)
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

        joy = Joy()
        joy.header.stamp = rospy.Time.now()
        joy.header.frame_id = args.frame_id

        axes = [0.0] * 8
        buttons = [0] * 11

        r_axes = []
        r_btns = []
        r_hats = []

        if last_msg is not None:
            r_axes = last_msg.get("axes", [])
            r_btns = last_msg.get("buttons", [])
            r_hats = last_msg.get("hats", [])

            rospy.loginfo_once(
                f"UDP Joy shape: axes={len(r_axes)} buttons={len(r_btns)} hats={r_hats}"
            )
        
        if len(r_axes) >= 6:
            # Left stick (same everywhere)
            axes[0] = float(r_axes[0])
            axes[1] = -float(r_axes[1])

            is_ubuntu = len(r_hats) > 0

            if is_ubuntu:
                # Ubuntu (SDL / evdev)
                # 0 LX, 1 LY, 2 L2, 3 RX, 4 RY, 5 R2
                axes[2] = float(r_axes[2])      # LT
                axes[3] = -float(r_axes[3])     # RX
                axes[4] = -float(r_axes[4])     # RY
                axes[5] = float(r_axes[5])      # RT
            else:
                # Windows (XInput)
                # 0 LX, 1 LY, 2 RX, 3 RY, 4 L2, 5 R2
                axes[2] = float(r_axes[4])      # LT
                axes[3] = -float(r_axes[2])     # RX
                axes[4] = -float(r_axes[3])     # RY
                axes[5] = float(r_axes[5])      # RT




            # -----------------------------
            # D-PAD (FIXED)
            # -----------------------------
            axes[6] = 0.0
            axes[7] = 0.0

            if len(r_hats) > 0:
                # Ubuntu
                hat_x, hat_y = r_hats[0]
                axes[6] = float(hat_x)
                axes[7] = float(hat_y)
            else:
                # Windows (original behavior)
                if safe_btn(r_btns, 13):
                    axes[6] = 1
                if safe_btn(r_btns, 14):
                    axes[6] = -1
                if safe_btn(r_btns, 11):
                    axes[7] = 1
                if safe_btn(r_btns, 12):
                    axes[7] = -1

            # -----------------------------
            # BUTTONS (OS-AWARE)
            # -----------------------------
            if is_ubuntu:
                # Ubuntu (evdev)
                buttons[0]  = safe_btn(r_btns, 0)   # A (X)
                buttons[1]  = safe_btn(r_btns, 1)   # B (Circle)
                buttons[2]  = safe_btn(r_btns, 3)   # X (Square)
                buttons[3]  = safe_btn(r_btns, 2)   # Y (Triangle)

                buttons[4]  = safe_btn(r_btns, 4)   # LB (L1) ✅ FIX
                buttons[5]  = safe_btn(r_btns, 5)   # RB (R1)

                buttons[6]  = safe_btn(r_btns, 8)   # Back (Share)
                buttons[7]  = safe_btn(r_btns, 9)   # Start (Options)
                buttons[8]  = safe_btn(r_btns, 10)  # Guide (PS)

                buttons[9]  = safe_btn(r_btns, 11)  # L3
                buttons[10] = safe_btn(r_btns, 12)  # R3
            else:
                # Windows (XInput)
                buttons[0]  = safe_btn(r_btns, 0)   # A
                buttons[1]  = safe_btn(r_btns, 1)   # B
                buttons[2]  = safe_btn(r_btns, 2)   # X
                buttons[3]  = safe_btn(r_btns, 3)   # Y

                buttons[4]  = safe_btn(r_btns, 9)   # LB (L1)
                buttons[5]  = safe_btn(r_btns, 10)  # RB (R1)

                buttons[6]  = safe_btn(r_btns, 4)   # Back
                buttons[7]  = safe_btn(r_btns, 6)   # Start
                buttons[8]  = safe_btn(r_btns, 15)  # Guide

                buttons[9]  = safe_btn(r_btns, 7)   # L3
                buttons[10] = safe_btn(r_btns, 8)   # R3


        joy.axes = axes
        joy.buttons = buttons

        now = time.time()
        if got_any or (now - last_recv_time) > keepalive_interval:
            pub.publish(joy)
            if not got_any:
                last_recv_time = now

        rospy.sleep(0.001)

if __name__ == "__main__":
    main()
