#!/usr/bin/env python3
import asyncio
import socket
import struct
import logging
from cot_utils import generate_obstacle_cot, generate_goal_cot, generate_start_cot

class ObstacleUDPWorker:
    def __init__(self, tx_queue, listen_ip="0.0.0.0", listen_port=49153):
        self.tx_queue = tx_queue
        self.listen_ip = listen_ip
        self.listen_port = listen_port

    async def run(self):
        loop = asyncio.get_running_loop()
        transport, _ = await loop.create_datagram_endpoint(
            lambda: ObstacleProtocol(self.tx_queue),
            local_addr=(self.listen_ip, self.listen_port)
        )
        logging.info(f"[UDP] Listening for Pose packets on {self.listen_ip}:{self.listen_port}")
        while True:
            await asyncio.sleep(3600)

class ObstacleProtocol(asyncio.DatagramProtocol):
    def __init__(self, tx_queue):
        self.tx_queue = tx_queue

    def datagram_received(self, data, addr):
        try:
            if len(data) == 16:
                header = data[0]
                _, _, idx, x, y, z, _ = struct.unpack('=BBBfffB', data)

                if header == 0xAE:
                    logging.info(f"[OBS] From {addr} → idx={idx}, x={x:.6f}, y={y:.6f}, z={z:.6f}")
                    cot = generate_obstacle_cot(idx, x, y, z)

                elif header == 0xAF:
                    logging.info(f"[GOAL] From {addr} → idx={idx}, x={x:.6f}, y={y:.6f}, z={z:.6f}")
                    cot = generate_goal_cot(idx, x, y, z)

                elif header == 0xAC:
                    logging.info(f"[START] From {addr} → idx={idx}, x={x:.6f}, y={y:.6f}, z={z:.6f}")
                    cot = generate_start_cot(idx, x, y, z)

                else:
                    logging.warning(f"[UNKNOWN HEADER] {header} from {addr}")
                    return

                asyncio.create_task(self.tx_queue.put(cot))

            else:
                logging.warning(f"[IGNORED] Invalid size {len(data)} bytes from {addr}: {data.hex()}")

        except Exception as e:
            logging.error(f"[ERROR] Parsing packet from {addr}: {e}")
