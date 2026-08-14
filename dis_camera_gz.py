import dis
import marshal
import sys

with open('.scripts/olds/v30/__pycache__/camera_gz.cpython-312.pyc', 'rb') as f:
    f.seek(16)
    code = marshal.load(f)

with open('dis_camera_gz.txt', 'w') as out:
    sys.stdout = out
    dis.dis(code)
    sys.stdout = sys.__stdout__
