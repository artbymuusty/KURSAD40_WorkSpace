import os
import sys
import time

os.environ["GZ_IP"] = "127.0.0.1"
sys.path.append("/usr/lib/python3/dist-packages")

import gz.transport13
from gz.msgs11.image_pb2 import Image as ImageMsg
from config import DEFAULT_GZ_TOPIC

node = gz.transport13.Node()
topic = DEFAULT_GZ_TOPIC

def cb(msg):
    print("Received frame!")

print("Subscribing with keyword arguments...")
res = node.subscribe(topic=topic, callback=cb, msg_type=ImageMsg)
print(f"Subscribe returned: {res}")

time.sleep(2)
