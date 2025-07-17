#!/usr/bin/env python3

from multiprocessing import Queue, Process
from udp_listener_async import listener_worker
import time

if __name__ == "__main__":
    queue = Queue()
    p = Process(target=listener_worker, args=(queue,))
    p.start()

    try:
        while True:
            if not queue.empty():
                msg_type, data = queue.get()
                print(f"[TEST RECEIVER] {msg_type.upper()} => {data}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("Receiver interrupted.")
        p.terminate()
        p.join()

