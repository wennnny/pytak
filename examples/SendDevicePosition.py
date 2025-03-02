#!/usr/bin/env python3

import asyncio
import xml.etree.ElementTree as ET
import pytak, uuid
from configparser import ConfigParser

class MySerializer(pytak.QueueWorker):
    """
    Defines how you process or generate your Cursor on Target Events.
    From there it adds the CoT Events to a queue for TX to a COT_URL.
    """

    async def handle_data(self, data):
        """Handle pre-CoT data, serialize to CoT Event, then puts on queue."""
        event = data
        await self.put_queue(event)

    async def run(self):
        """Run the loop for processing or generating pre-CoT data."""
        while True:
            data = generate_gps_cot()
            self._logger.info("Sending:\n%s\n", data.decode())
            await self.handle_data(data)
            await asyncio.sleep(10)


def generate_gps_cot():
    """Generate a simple takPong CoT Event."""
    root = ET.Element("event")
    root.set("version", "2.0")
    root.set("type", "a-f-G-U-C-I")
    root.set("uid", "test_20250301") 
    root.set("how", "m-g")
    root.set("time", pytak.cot_time())
    root.set("start", pytak.cot_time())
    root.set("stale", pytak.cot_time(300))

    gps_data = {
        "lat": "24.784074",
        "lon": "120.998384",
        "hae": "999999",
        "ce": "999999",
        "le": "999999",
    }
    ET.SubElement(root, "point", attrib=gps_data)
    detail = ET.SubElement(root, "detail")
    ET.SubElement(detail, "takv", {"device": "MyDevice", "platform": "Python", "os": "Linux"})
    ET.SubElement(detail, "contact", {"callsign": "MyDevice1"})
    ET.SubElement(detail, "__group", {"name": "Purple", "role": "Team Member"})
    ET.SubElement(detail, "uid", {"Droid": "my-tak-device"})

    return ET.tostring(root)


async def main():
    """Main definition of your program, sets config params and
    adds your serializer to the asyncio task list.
    """
    config = ConfigParser()
    config["mycottool"] = {
        "COT_URL": "tls://140.113.148.80:8089",
        "PYTAK_TLS_CLIENT_CERT": "/home/weny/Downloads/TAK_Server_Configurator/tak-server/tak/certs/files/admin_cert.pem",
        "PYTAK_TLS_CLIENT_KEY": "/home/weny/Downloads/TAK_Server_Configurator/tak-server/tak/certs/files/admin_key.pem",
        "PYTAK_TLS_CA_CERT": "/home/weny/Downloads/TAK_Server_Configurator/tak-server/tak/certs/files/admin-trusted.pem",
        "PYTAK_TLS_DONT_CHECK_HOSTNAME": "1",
        "PYTAK_TLS_DONT_VERIFY": "1"
    }
    config = config["mycottool"]

    # Initializes worker queues and tasks.
    clitool = pytak.CLITool(config)
    await clitool.setup()

    # Add your serializer to the asyncio task list.
    clitool.add_tasks(set([MySerializer(clitool.tx_queue, config)]))

    # Start all tasks.
    await clitool.run()


if __name__ == "__main__":
    asyncio.run(main())
