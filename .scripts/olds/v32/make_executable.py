import os
import stat

scripts = [
    '/home/muusty/autokursad/.scripts/olds/v32/run_mission_v32_gz.sh',
    '/home/muusty/autokursad/.scripts/olds/v32/run_mission_v32_real.sh',
    '/home/muusty/autokursad/.scripts/olds/v32/run_mission_v32_dual.sh'
]

for s in scripts:
    st = os.stat(s)
    os.chmod(s, st.st_mode | stat.S_IEXEC)
