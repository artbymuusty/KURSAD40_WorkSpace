import os
import sys
import time

os.environ["GZ_IP"] = "127.0.0.1"
sys.path.append("/usr/lib/python3/dist-packages")
sys.path.append("/home/muusty/autokursad/.scripts/olds/v30")

if "--python" in sys.argv:
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    print("Using PURE PYTHON protobuf")
else:
    print("Using C++ protobuf")

import gz.transport13
from gz.msgs11.image_pb2 import Image as ImageMsg
from config import DEFAULT_GZ_TOPIC

node = gz.transport13.Node()
topic = DEFAULT_GZ_TOPIC

def cb(msg):
    print("Received frame!")

print("Subscribing...")
res = node.subscribe(ImageMsg, topic, cb)
print(f"Subscribe returned: {res}")

time.sleep(2)
