#!/usr/bin/env python3

from multiprocessing import Queue, Process
from udp_listener_async_49152 import listener_worker
import time

if __name__ == "__main__":
    queue = Queue()
    p = Process(target=listener_worker, args=(queue,))
    p.start()

    try:
        while True:
            if not queue.empty():
                msg_type, data = queue.get()

                if msg_type == "gps" and "latency_ms" in data:
                    lat = data.get("lat")
                    lon = data.get("lon")
                    seq = data.get("seq")
                    lat_ms = data["latency_ms"]
                    print(f"[GPS] seq={seq} lat={lat:.6f} lon={lon:.6f} "
                          f"latency={lat_ms:.1f} ms")
                else:
                    print(f"[TEST RECEIVER] {msg_type.upper()} => {data}")

            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Receiver interrupted.")
        p.terminate()
        p.join()
