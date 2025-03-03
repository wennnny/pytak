#!/usr/bin/env python3

import asyncio
import xml.etree.ElementTree as ET
import pytak

from configparser import ConfigParser

#!/usr/bin/env python3

import asyncio
import xml.etree.ElementTree as ET
import pytak
from configparser import ConfigParser


def gen_presence_cot():
    """Generate simple CoT Event to indicate presence."""
    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", "a-f-G-U-C")  # Friendly Ground Unit
    root.set("uid", "receiver-device-001")
    root.set("how", "m-g")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(60))

    point = ET.SubElement(root, "point")
    point.set("lat", "25.0330")
    point.set("lon", "121.5654")
    point.set("hae", "0")
    point.set("ce", "9999999")
    point.set("le", "9999999") 

    return ET.tostring(root)


class MySender(pytak.QueueWorker):
    """Periodic presence signal sender."""

    async def handle_data(self, data):
        await self.put_queue(data)

    async def run(self):
        while True:
            data = gen_presence_cot()
            self._logger.info("[Presence] Sending:\n%s\n", data.decode())
            await self.handle_data(data)
            await asyncio.sleep(60)  # Send every 60 seconds


class MyReceiver(pytak.QueueWorker):
    """Handle incoming CoT events."""

    async def handle_data(self, data):
        self._logger.info("[Received] CoT Event:\n%s\n", data.decode())

    async def run(self):
        while True:
            data = await self.queue.get()
            await self.handle_data(data)


async def main():
    """Main definition of your program, sets config params and
    adds your serializer to the asyncio task list.
    """
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://140.113.148.80:8089",
        "PYTAK_TLS_CLIENT_CERT": "/home/wen/Downloads/user1_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/wen/Downloads/user1_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/wen/Downloads/user1-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    config = config["mycottool"]

    # Initializes worker queues and tasks.
    clitool = pytak.CLITool(config)
    await clitool.setup()

    # Add your serializer to the asyncio task list.
    clitool.add_tasks(
        set([MySender(clitool.tx_queue, config), MyReceiver(clitool.rx_queue, config)])
    )

    # Start all tasks.
    await clitool.run()


if __name__ == "__main__":
    asyncio.run(main())
