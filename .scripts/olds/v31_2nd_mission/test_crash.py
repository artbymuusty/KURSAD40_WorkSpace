import os
import sys

os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")
sys.path.append("/usr/lib/python3/dist-packages")

try:
    print("Importing Node...")
    from gz.transport13 import Node
    print("Importing ImageMsg...")
    from gz.msgs11.image_pb2 import Image as ImageMsg
    print("Success")
except ImportError as e:
    print("ImportError:", e)
except Exception as e:
    print("Exception:", e)
    print(type(e))
