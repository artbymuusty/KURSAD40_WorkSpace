import sys
sys.path.append("/usr/lib/python3/dist-packages")

try:
    import gz.msgs10.image_pb2 as msgs10
    print("msgs10:", msgs10.Image.DESCRIPTOR.full_name)
except Exception as e:
    print("msgs10 error:", e)

try:
    import gz.msgs11.image_pb2 as msgs11
    print("msgs11:", msgs11.Image.DESCRIPTOR.full_name)
except Exception as e:
    print("msgs11 error:", e)
