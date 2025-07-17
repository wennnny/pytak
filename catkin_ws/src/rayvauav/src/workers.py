import asyncio, socket, struct
import logging
from collections import defaultdict
from cot_utils import generate_chat_cot, generate_gps_cot, generate_obstacle_cot

latest_msgs = defaultdict(str)
latest_gps = {"lat": None, "lon": None, "hae": 999999.0}

async def handle_queue_data(q, tx_queue):
    while True:
        if not q.empty():
            msg_type, data = q.get()
            if msg_type == "gps":
                latest_msgs['gps'] = f"[GPS] Seq={data['seq']}, Time={data['stamp']:.3f}, Lat={data['lat']:.6f}, Lon={data['lon']:.6f}"
                latest_gps['lat'] = data['lat']
                latest_gps['lon'] = data['lon']
            elif msg_type == "vel":
                latest_msgs['vel'] = f"[VEL] Seq={data['seq']}, LinearX={data['linear_x']:.3f}"
            elif msg_type == "can":
                latest_msgs['can'] = f"[CAN] Throttle={data['throttle']}, Steering={data['steering']}"
            elif msg_type == "hdg":
                latest_msgs['hdg'] = f"[HDG] Heading={data['heading']:.2f}°"
            # elif msg_type == "obspos":
            #     idx, x, y, z = data["idx"], data["x"], data["y"], data["z"]
            #     latest_msgs[f'obstacle{idx}'] = f"[OBS] idx={idx} x={x:.2f}, y={y:.2f}, z={z:.2f}"
            #     cot_msg = generate_obstacle_cot(idx, x, y, z)
            #     logging.info("Sending:\n%s", cot_msg.decode())
            #     await tx_queue.put(cot_msg)
        await asyncio.sleep(0.01)

class BundlerWorker:
    def __init__(self, tx_queue):
        self.tx_queue = tx_queue

    async def run(self):
        while True:
            combined_msg = "\n".join([
                latest_msgs.get('gps', ''),
                latest_msgs.get('can', ''),
                latest_msgs.get('hdg', ''),
                latest_msgs.get('vel', '')
            ]).strip()
            if combined_msg:
                cot_msg = generate_chat_cot(combined_msg)
                logging.info("Sending:\n%s", cot_msg.decode())
                await self.tx_queue.put(cot_msg)
            await asyncio.sleep(1)

class GPSMemberWorker:
    def __init__(self, tx_queue):
        self.tx_queue = tx_queue

    async def run(self):
        while True:
            if latest_gps["lat"] and latest_gps["lon"]:
                cot_msg = generate_gps_cot(latest_gps)
                logging.info("Sending:\n%s", cot_msg.decode())
                await self.tx_queue.put(cot_msg)
            await asyncio.sleep(1)

class MyReceiver:
    def __init__(self, rx_queue):
        self.rx_queue = rx_queue

    async def handle_data(self, data):
        print("[RX CoT]", data.decode())

    async def run(self):
        while True:
            data = await self.rx_queue.get()
            await self.handle_data(data)
