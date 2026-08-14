import subprocess
out = subprocess.check_output("gz topic -i -t /world/default/model/x500_mono_cam_down_0/link/camera_link/sensor/camera/image", shell=True).decode()
with open("topic_info.txt", "w") as f:
    f.write(out)
