import os

base = '/home/muusty/autokursad/.scripts/olds/v32/v32_flight_stack'
dirs = [
    'core/interfaces',
    'core/config',
    'core/detection',
    'core/navigation',
    'core/position_log',
    'core/mission',
    'core/telemetry',
    'real_system/config',
    'gz_system/config',
    'mavsdk_common',
    'dual_system',
    'tests/mocks',
    'docs'
]

init_files = [
    'core/__init__.py',
    'core/interfaces/__init__.py',
    'core/config/__init__.py',
    'core/detection/__init__.py',
    'core/navigation/__init__.py',
    'core/position_log/__init__.py',
    'core/mission/__init__.py',
    'core/telemetry/__init__.py',
    'real_system/__init__.py',
    'real_system/config/__init__.py',
    'gz_system/__init__.py',
    'gz_system/config/__init__.py',
    'mavsdk_common/__init__.py',
    'dual_system/__init__.py',
    'tests/__init__.py',
    'tests/mocks/__init__.py',
]

for d in dirs:
    os.makedirs(os.path.join(base, d), exist_ok=True)

for f in init_files:
    open(os.path.join(base, f), 'a').close()
