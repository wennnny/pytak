#!/usr/bin/env python3

import asyncio
import socket
import xml.etree.ElementTree as ET
from configparser import ConfigParser
import pytak

# === UDP 傳送目標設定 ===
UDP_TARGET_IP = "140.113.148.99"
UDP_TARGET_PORT = 49158  # 這個 port 可以自行調整
udp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def extract_chat_message(xml_data):
    try:
        root = ET.fromstring(xml_data)
        event_type = root.get("type", "")

        if event_type != "b-t-f":
            return None

        remarks = root.find("./detail/remarks")
        message = remarks.text.strip() if remarks is not None else None
        return message
    except Exception as e:
        print(f"[ERROR] XML parse failed: {e}")
        return None

def send_udp_message(message: str):
    try:
        data = message.encode("utf-8")
        udp_sock.sendto(data, (UDP_TARGET_IP, UDP_TARGET_PORT))
        print(f"[UDP] Sent message: {message}")
    except Exception as e:
        print(f"[ERROR] Failed to send UDP: {e}")

class ChatMessageForwarder(pytak.QueueWorker):
    """接收 TAK 聊天訊息並轉送 message string 到 UDP"""

    async def handle_data(self, data):
        try:
            xml_str = data.decode()
            message = extract_chat_message(xml_str)
            if message and message.lower().startswith("moos"):
                send_udp_message(message)
        except Exception as e:
            print(f"[ERROR] handle_data failed: {e}")

    async def run(self):
        while True:
            data = await self.queue.get()
            await self.handle_data(data)

async def main():
    # === pytak 連線設定 ===
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://140.113.148.80:8089",
        "PYTAK_TLS_CLIENT_CERT": "/home/moos-dawg/Downloads/TAK_Server_Configurator/pytak/catkin_ws/files_WAN/argtest2_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/moos-dawg/Downloads/TAK_Server_Configurator/pytak/catkin_ws/files_WAN/argtest2_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/moos-dawg/Downloads/TAK_Server_Configurator/pytak/catkin_ws/files_WAN/argtest2-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }

    config = config["mycottool"]
    clitool = pytak.CLITool(config)
    await clitool.setup()

    clitool.add_tasks({ChatMessageForwarder(clitool.rx_queue, config)})
    await clitool.run()

if __name__ == "__main__":
    asyncio.run(main())

