"""Python-side counterpart of gz_env.sh -- the KURSAD40 gz-transport environment.

Python entrypoints must not depend on a shell having sourced gz_env.sh, so
this module applies the same two values directly to os.environ.

WHY THIS EXISTS (macOS port finding): gz-transport's DEFAULT partition is
"<hostname>:<username>". On macOS the hostname is DHCP/reverse-DNS derived and
changes with the network, so the simulator and anything started later computed
different default partitions and never discovered each other -- Gazebo was
publishing 1280x960 @30Hz while camera_service.py saw zero frames. Pinning an
explicit fixed partition removes the hostname from the equation.

Keep GZ_PARTITION/GZ_IP in sync with gz_env.sh.
"""
import os

# Fixed literal, deliberately NOT hostname-derived (see module docstring).
GZ_PARTITION = "kursad40"
GZ_IP = "127.0.0.1"


def apply_gz_env(env=None):
    """Apply the KURSAD40 gz-transport settings to `env` (default: os.environ).

    Existing values are respected (setdefault), so an explicit override from
    the shell or a parent process still wins. Returns the mapping.
    """
    target = os.environ if env is None else env
    target.setdefault("GZ_PARTITION", GZ_PARTITION)
    target.setdefault("GZ_IP", GZ_IP)
    return target


def describe_gz_env(env=None):
    """One-line summary of the EFFECTIVE settings, for startup logging.

    Drift between processes is invisible until frames silently stop arriving,
    so every entrypoint prints this.
    """
    target = os.environ if env is None else env
    return (f"GZ_PARTITION={target.get('GZ_PARTITION', '<unset>')} "
            f"GZ_IP={target.get('GZ_IP', '<unset>')}")
