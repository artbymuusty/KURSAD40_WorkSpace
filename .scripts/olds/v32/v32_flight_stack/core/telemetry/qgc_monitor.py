"""
Operator-requested visibility: "let's see in the mission board whether QGC
is connected or not." MAVSDK's Core module exposes only OUR OWN connection
state (core.connection_state()) -- nothing about other GCS clients on the
MAVLink network, so there is no direct API to ask "is QGroundControl
connected." That's a real platform limitation, not an oversight.

The most honest available signal without deep MAVLink packet sniffing:
QGroundControl binds UDP 14550 (its conventional default) when it starts,
whether or not it is currently receiving telemetry. Detecting a locally
bound socket on that port is a best-effort heuristic proxy for "a GCS is
running here" -- not a definitive confirmation that it is actively talking
to this specific vehicle. Labelled and reported as such throughout.
"""
import logging
from typing import Optional

from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

logger = logging.getLogger("telemetry.qgc_monitor")

try:
    import psutil
    _PSUTIL_AVAILABLE = True
except Exception:  # noqa: BLE001
    _PSUTIL_AVAILABLE = False


class QgcMonitor:
    def __init__(self, publisher: EventPublisher = NULL_PUBLISHER, udp_port: int = 14550):
        self.publisher = publisher
        self.udp_port = udp_port
        self._last_state: Optional[bool] = None
        if not _PSUTIL_AVAILABLE:
            logger.warning("psutil not available -- QGC connection status will report UNKNOWN.")

    def _port_is_bound(self) -> Optional[bool]:
        """None means "couldn't determine" (psutil unavailable or a
        permission/platform error) -- must be distinguished from a
        confirmed False, so the dashboard can show UNKNOWN rather than
        falsely claiming QGC definitely isn't connected."""
        if not _PSUTIL_AVAILABLE:
            return None
        try:
            for conn in psutil.net_connections(kind="udp"):
                if conn.laddr and conn.laddr.port == self.udp_port:
                    return True
            return False
        except Exception as e:  # noqa: BLE001 -- permission errors on some platforms, must not crash the checker
            logger.warning(f"QGC port check failed, reporting UNKNOWN: {e}")
            return None

    def check(self) -> Optional[bool]:
        connected = self._port_is_bound()
        if connected != self._last_state:
            self._last_state = connected
            self.publisher.publish(Event(
                code="QGC_STATUS", subsystem="QgcMonitor", category=Category.TELEMETRY,
                severity=Severity.INFO,
                message=f"QGC {'detected' if connected else 'not detected' if connected is False else 'status unknown'} "
                        f"on UDP {self.udp_port} (heuristic: local port bound, not a MAVLink-level confirmation)",
                data={"connected": connected, "port": self.udp_port},
            ))
        return connected
