#!/usr/bin/env python3
import rospy
import socket
import threading
import time
import os
import subprocess
from sensor_msgs.msg import NavSatFix
from datetime import datetime, timedelta

# 轉發目標（Base station）
TARGET_IP = "192.168.133.15"
TARGET_PORT = 49154

# nc 監聽的 port（跟你平常打的 nc -lk 13550 一樣）
NC_PORT = 13550

# 全域變數
send_sock = None
log_file = None
nc_proc = None


# --------------------------------------------------------
# 1️⃣ 記錄 GPS，不做 UDP 發送
# --------------------------------------------------------
def gps_callback(msg: NavSatFix):
    global log_file
    try:
        # 轉成本地時間（+8）
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
# 2️⃣ 從 nc -lk 13550 讀資料，逐行轉發
# --------------------------------------------------------
def nc_reader():
    global nc_proc, send_sock, log_file

    rospy.loginfo(f"[NC] Starting: nc -lk {NC_PORT}")

    # 開 subprocess 跑 nc，接 stdout
    # -l: listen
    # -k: keep listening
    # 建議如果原本是 UDP，就改成 `nc -luk` 並在下面 command 加上 -u
    cmd = ["nc", "-lk", str(NC_PORT)]
    nc_proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,          # 直接拿到 str，而不是 bytes
        bufsize=1           # line-buffered
    )

    # 一行一行讀 nc 輸出的資料
    for line in nc_proc.stdout:
        if rospy.is_shutdown():
            break

        line = line.rstrip("\n")
        if not line:
            continue

        # log 原始文字
        log_file.write(f"[NC_RAW] {line}\n")
        log_file.flush()

        # 轉成 bytes 丟出去（這裡我保留換行，視你需要改）
        payload = (line + "\n").encode("utf-8")

        try:
            send_sock.sendto(payload, (TARGET_IP, TARGET_PORT))
            rospy.loginfo(
                f"[UDP] Forwarded {len(payload)} bytes -> "
                f"{TARGET_IP}:{TARGET_PORT}"
            )
        except OSError as e:
            rospy.logerr(f"[UDP] sendto failed: {e}")
            continue

    rospy.loginfo("[NC] Reader thread exiting")


# --------------------------------------------------------
# Main
# --------------------------------------------------------
def main():
    global send_sock, log_file, nc_proc

    rospy.init_node("udp_zyxel_to_basestation", anonymous=True)

    # 建立 log 資料夾
    if not os.path.exists("log"):
        os.makedirs("log")

    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join("log", f"{current_time}.txt")
    log_file = open(log_path, "a")

    # 只需要一顆「發送用」UDP socket
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 訂閱 GPS，只記錄
    rospy.Subscriber("/fix", NavSatFix, gps_callback)
    rospy.loginfo("GPS logging enabled.")

    # 開 thread 跑 nc reader
    t = threading.Thread(target=nc_reader, daemon=True)
    t.start()

    try:
        rospy.spin()
    finally:
        # 結束時關掉資源
        try:
            if nc_proc is not None:
                nc_proc.terminate()
        except Exception:
            pass
        try:
            log_file.close()
        except Exception:
            pass
        try:
            send_sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rospy
import socket
import threading
import time
import os
import subprocess
from sensor_msgs.msg import NavSatFix
from datetime import datetime, timedelta

# 轉發目標（Base station）
TARGET_IP = "192.168.133.15"
TARGET_PORT = 49154

# nc 監聽的 port（跟你平常打的 nc -lk 13550 一樣）
NC_PORT = 13550

# 全域變數
send_sock = None
log_file = None
nc_proc = None


# --------------------------------------------------------
# 1️⃣ 記錄 GPS，不做 UDP 發送
# --------------------------------------------------------
def gps_callback(msg: NavSatFix):
    global log_file
    try:
        # 轉成本地時間（+8）
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
# 2️⃣ 從 nc -lk 13550 讀資料，逐行轉發
# --------------------------------------------------------
def nc_reader():
    global nc_proc, send_sock, log_file

    rospy.loginfo(f"[NC] Starting: nc -lk {NC_PORT}")

    # 開 subprocess 跑 nc，接 stdout
    # -l: listen
    # -k: keep listening
    # 建議如果原本是 UDP，就改成 `nc -luk` 並在下面 command 加上 -u
    cmd = ["nc", "-lk", str(NC_PORT)]
    nc_proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,          # 直接拿到 str，而不是 bytes
        bufsize=1           # line-buffered
    )

    # 一行一行讀 nc 輸出的資料
    for line in nc_proc.stdout:
        if rospy.is_shutdown():
            break

        line = line.rstrip("\n")
        if not line:
            continue

        # log 原始文字
        log_file.write(f"[NC_RAW] {line}\n")
        log_file.flush()

        # 轉成 bytes 丟出去（這裡我保留換行，視你需要改）
        payload = (line + "\n").encode("utf-8")

        try:
            send_sock.sendto(payload, (TARGET_IP, TARGET_PORT))
            rospy.loginfo(
                f"[UDP] Forwarded {len(payload)} bytes -> "
                f"{TARGET_IP}:{TARGET_PORT}"
            )
        except OSError as e:
            rospy.logerr(f"[UDP] sendto failed: {e}")
            continue

    rospy.loginfo("[NC] Reader thread exiting")


# --------------------------------------------------------
# Main
# --------------------------------------------------------
def main():
    global send_sock, log_file, nc_proc

    rospy.init_node("udp_zyxel_to_basestation", anonymous=True)

    # 建立 log 資料夾
    if not os.path.exists("log"):
        os.makedirs("log")

    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join("log", f"{current_time}.txt")
    log_file = open(log_path, "a")

    # 只需要一顆「發送用」UDP socket
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 訂閱 GPS，只記錄
    rospy.Subscriber("/fix", NavSatFix, gps_callback)
    rospy.loginfo("GPS logging enabled.")

    # 開 thread 跑 nc reader
    t = threading.Thread(target=nc_reader, daemon=True)
    t.start()

    try:
        rospy.spin()
    finally:
        # 結束時關掉資源
        try:
            if nc_proc is not None:
                nc_proc.terminate()
        except Exception:
            pass
        try:
            log_file.close()
        except Exception:
            pass
        try:
            send_sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rospy
import socket
import threading
import time
import os
import subprocess
from sensor_msgs.msg import NavSatFix
from datetime import datetime, timedelta

# 轉發目標（Base station）
TARGET_IP = "192.168.133.15"
TARGET_PORT = 49154

# nc 監聽的 port（跟你平常打的 nc -lk 13550 一樣）
NC_PORT = 13550

# 全域變數
send_sock = None
log_file = None
nc_proc = None


# --------------------------------------------------------
# 1️⃣ 記錄 GPS，不做 UDP 發送
# --------------------------------------------------------
def gps_callback(msg: NavSatFix):
    global log_file
    try:
        # 轉成本地時間（+8）
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
# 2️⃣ 從 nc -lk 13550 讀資料，逐行轉發
# --------------------------------------------------------
def nc_reader():
    global nc_proc, send_sock, log_file

    rospy.loginfo(f"[NC] Starting: nc -lk {NC_PORT}")

    # 開 subprocess 跑 nc，接 stdout
    # -l: listen
    # -k: keep listening
    # 建議如果原本是 UDP，就改成 `nc -luk` 並在下面 command 加上 -u
    cmd = ["nc", "-lk", str(NC_PORT)]
    nc_proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,          # 直接拿到 str，而不是 bytes
        bufsize=1           # line-buffered
    )

    # 一行一行讀 nc 輸出的資料
    for line in nc_proc.stdout:
        if rospy.is_shutdown():
            break

        line = line.rstrip("\n")
        if not line:
            continue

        # log 原始文字
        log_file.write(f"[NC_RAW] {line}\n")
        log_file.flush()

        # 轉成 bytes 丟出去（這裡我保留換行，視你需要改）
        payload = (line + "\n").encode("utf-8")

        try:
            send_sock.sendto(payload, (TARGET_IP, TARGET_PORT))
            rospy.loginfo(
                f"[UDP] Forwarded {len(payload)} bytes -> "
                f"{TARGET_IP}:{TARGET_PORT}"
            )
        except OSError as e:
            rospy.logerr(f"[UDP] sendto failed: {e}")
            continue

    rospy.loginfo("[NC] Reader thread exiting")


# --------------------------------------------------------
# Main
# --------------------------------------------------------
def main():
    global send_sock, log_file, nc_proc

    rospy.init_node("udp_zyxel_to_basestation", anonymous=True)

    # 建立 log 資料夾
    if not os.path.exists("log"):
        os.makedirs("log")

    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join("log", f"{current_time}.txt")
    log_file = open(log_path, "a")

    # 只需要一顆「發送用」UDP socket
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 訂閱 GPS，只記錄
    rospy.Subscriber("/fix", NavSatFix, gps_callback)
    rospy.loginfo("GPS logging enabled.")

    # 開 thread 跑 nc reader
    t = threading.Thread(target=nc_reader, daemon=True)
    t.start()

    try:
        rospy.spin()
    finally:
        # 結束時關掉資源
        try:
            if nc_proc is not None:
                nc_proc.terminate()
        except Exception:
            pass
        try:
            log_file.close()
        except Exception:
            pass
        try:
            send_sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rospy
import socket
import threading
import time
import os
import subprocess
from sensor_msgs.msg import NavSatFix
from datetime import datetime, timedelta

# 轉發目標（Base station）
TARGET_IP = "192.168.133.15"
TARGET_PORT = 49154

# nc 監聽的 port（跟你平常打的 nc -lk 13550 一樣）
NC_PORT = 13550

# 全域變數
send_sock = None
log_file = None
nc_proc = None


# --------------------------------------------------------
# 1️⃣ 記錄 GPS，不做 UDP 發送
# --------------------------------------------------------
def gps_callback(msg: NavSatFix):
    global log_file
    try:
        # 轉成本地時間（+8）
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
# 2️⃣ 從 nc -lk 13550 讀資料，逐行轉發
# --------------------------------------------------------
def nc_reader():
    global nc_proc, send_sock, log_file

    rospy.loginfo(f"[NC] Starting: nc -lk {NC_PORT}")

    # 開 subprocess 跑 nc，接 stdout
    # -l: listen
    # -k: keep listening
    # 建議如果原本是 UDP，就改成 `nc -luk` 並在下面 command 加上 -u
    cmd = ["nc", "-lk", str(NC_PORT)]
    nc_proc = subprocess.Popen(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,          # 直接拿到 str，而不是 bytes
        bufsize=1           # line-buffered
    )

    # 一行一行讀 nc 輸出的資料
    for line in nc_proc.stdout:
        if rospy.is_shutdown():
            break

        line = line.rstrip("\n")
        if not line:
            continue

        # log 原始文字
        log_file.write(f"[NC_RAW] {line}\n")
        log_file.flush()

        # 轉成 bytes 丟出去（這裡我保留換行，視你需要改）
        payload = (line + "\n").encode("utf-8")

        try:
            send_sock.sendto(payload, (TARGET_IP, TARGET_PORT))
            rospy.loginfo(
                f"[UDP] Forwarded {len(payload)} bytes -> "
                f"{TARGET_IP}:{TARGET_PORT}"
            )
        except OSError as e:
            rospy.logerr(f"[UDP] sendto failed: {e}")
            continue

    rospy.loginfo("[NC] Reader thread exiting")


# --------------------------------------------------------
# Main
# --------------------------------------------------------
def main():
    global send_sock, log_file, nc_proc

    rospy.init_node("udp_zyxel_to_basestation", anonymous=True)

    # 建立 log 資料夾
    if not os.path.exists("log"):
        os.makedirs("log")

    current_time = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_path = os.path.join("log", f"{current_time}.txt")
    log_file = open(log_path, "a")

    # 只需要一顆「發送用」UDP socket
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # 訂閱 GPS，只記錄
    rospy.Subscriber("/fix", NavSatFix, gps_callback)
    rospy.loginfo("GPS logging enabled.")

    # 開 thread 跑 nc reader
    t = threading.Thread(target=nc_reader, daemon=True)
    t.start()

    try:
        rospy.spin()
    finally:
        # 結束時關掉資源
        try:
            if nc_proc is not None:
                nc_proc.terminate()
        except Exception:
            pass
        try:
            log_file.close()
        except Exception:
            pass
        try:
            send_sock.close()
        except Exception:
            pass


if __name__ == "__main__":
    main()

