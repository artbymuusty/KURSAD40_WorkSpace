import os
import sys
import time
from config import DEFAULT_GZ_TOPIC

if "--python" in sys.argv:
    os.environ["PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION"] = "python"
    print("Using PURE PYTHON protobuf")
else:
    print("Using C++ protobuf")

import gz.transport13
from gz.msgs11.image_pb2 import Image as ImageMsg

node = gz.transport13.Node()
topic = DEFAULT_GZ_TOPIC

def cb(msg):
    print("Received frame!")

print("Subscribing...")
res = node.subscribe(ImageMsg, topic, cb)
print(f"Subscribe returned: {res}")

time.sleep(2)
