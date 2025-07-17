#!/usr/bin/env python3

import asyncio, logging
import pytak
from configparser import ConfigParser
from multiprocessing import Queue, Process
from udp_listener_async_49152 import listener_worker
from udp_listener_async_49153 import ObstacleUDPWorker
from workers import handle_queue_data, BundlerWorker, GPSMemberWorker, MyReceiver


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s pytak %(levelname)s - %(message)s"
)

async def main():
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://140.113.148.80:8089",
        "PYTAK_TLS_CLIENT_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/moos-dawg/Downloads/pytak/catkin_ws/files_WAN/argtest2-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    config = config["mycottool"]

    clitool = pytak.CLITool(config)
    await clitool.setup()

    queue = Queue()
    listener_process = Process(target=listener_worker, args=(queue,))
    listener_process.start()

    parser_task = asyncio.create_task(handle_queue_data(queue, clitool.tx_queue))

    clitool.add_tasks({
        BundlerWorker(clitool.tx_queue),
        GPSMemberWorker(clitool.tx_queue),
        MyReceiver(clitool.rx_queue),
        ObstacleUDPWorker(clitool.tx_queue)  # ← Add obstacle worker
    })

    try:
        await asyncio.gather(clitool.run(), parser_task)
    finally:
        listener_process.terminate()
        listener_process.join()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[INFO] Interrupted by user. Shutting down.")

