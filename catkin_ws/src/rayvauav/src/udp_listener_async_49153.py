#!/usr/bin/env python3
import asyncio
import socket
import struct
import logging
from cot_utils import generate_obstacle_cot

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
        logging.info(f"[UDP] Listening for obstacles on {self.listen_ip}:{self.listen_port}")
        while True:
            await asyncio.sleep(3600)

class ObstacleProtocol(asyncio.DatagramProtocol):
    def __init__(self, tx_queue):
        self.tx_queue = tx_queue

    def datagram_received(self, data, addr):
        if len(data) == 16 and data[0] == 0xAE:
            try:
                _, _, idx, x, y, z, _ = struct.unpack('=BBBfffB', data)
                logging.info(f"[OBS] From {addr} → idx={idx}, x={x:.6f}, y={y:.6f}, z={z:.6f}")
                cot = generate_obstacle_cot(idx, x, y, z)
                asyncio.create_task(self.tx_queue.put(cot))
            except Exception as e:
                logging.error(f"[ERROR] Parsing obs packet: {e}")
        else:
            logging.warning(f"[IGNORED] {len(data)} bytes from {addr}: {data.hex()}")
