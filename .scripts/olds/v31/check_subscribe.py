import sys
sys.path.append('/usr/lib/python3/dist-packages')
from gz.transport13 import Node

print("Docstring for Node.subscribe:")
print(Node.subscribe.__doc__)

import inspect
print("\nSignature:")
try:
    print(inspect.signature(Node.subscribe))
except Exception as e:
    print(e)
