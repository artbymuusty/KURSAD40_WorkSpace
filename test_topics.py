import subprocess
out = subprocess.check_output("gz topic -l", shell=True).decode()
with open("topics.txt", "w") as f:
    f.write(out)
