#!/usr/bin/env python3
import rospy
import socket
import threading
import time
import os
from sensor_msgs.msg import NavSatFix
from datetime import datetime, timedelta

# 本機監聽 UDP
LISTEN_IP = "127.0.0.1"
LISTEN_PORT = 13550

# 轉發目標
TARGET_IP = "192.168.133.15"
TARGET_PORT = 49154

# 建立 socket（只做轉發，不含 GPS）
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))

# 建立 log 資料夾
if not os.path.exists("log"):
    os.makedirs("log")

current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
log_path = f"log/{current_time}.txt"
log_file = open(log_path, "a")


# --------------------------------------------------------
# 1️⃣ 記錄 GPS，不做 UDP 發送
# --------------------------------------------------------
def gps_callback(msg: NavSatFix):
    try:
        local_time = datetime.utcfromtimestamp(time.time()) + timedelta(hours=8)
        lat = msg.latitude
        lon = msg.longitude

        log_file.write(
            f"[GPS] Time: {local_time}, Lat: {lat:.6f}, Lon: {lon:.6f}\n"
        )
        log_file.flush()

    except Exception as e:
        rospy.logerr(f"[GPS] Log error: {e}")


# --------------------------------------------------------
# 2️⃣ UDP 接收 → 記錄 → 轉發（不阻塞主線程）
# --------------------------------------------------------
def udp_receiver():
    rospy.loginfo(f"[UDP] Listening on {LISTEN_IP}:{LISTEN_PORT}")

    while not rospy.is_shutdown():
        try:
            data, addr = sock.recvfrom(2048)

            # 記錄原始資料
            try:
                decoded = data.decode("utf-8", errors="replace")
            except:
                decoded = str(data)

            log_file.write(
                f"[RAW] From {addr}: {decoded}\n"
            )
            log_file.flush()

            # 轉發資料
            sock.sendto(data, (TARGET_IP, TARGET_PORT))

        except Exception as e:
            rospy.logerr(f"[UDP] Receive/forward error: {e}")


# --------------------------------------------------------
# Main
# --------------------------------------------------------
def main():
    rospy.init_node("udp_forwarder", anonymous=True)

    # 訂閱 GPS，只記錄
    rospy.Subscriber("/mavros/global_position/raw/fix", NavSatFix, gps_callback)
    rospy.loginfo("GPS logging enabled.")

    # 開啟 UDP 接收 thread
    t = threading.Thread(target=udp_receiver, daemon=True)
    t.start()

    rospy.spin()
    log_file.close()


if __name__ == "__main__":
    main()
