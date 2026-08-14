from typing import Tuple, Optional
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity

class MissionCheckpoint:
    def __init__(self, publisher: EventPublisher = NULL_PUBLISHER):
        self._checkpoint: Optional[Tuple[float, float, float]] = None
        self.publisher = publisher

    def save(self, lat: float, lon: float, alt: float) -> None:
        """Kalkış sonrası, Mission Route başlamadan hemen önce konumu kaydeder."""
        self._checkpoint = (lat, lon, alt)
        self.publisher.publish(Event(
            code="CHECKPOINT_SAVED", subsystem="MissionCheckpoint", category=Category.LIFECYCLE,
            data={"checkpoint": [lat, lon, alt]},
        ))

    def get(self) -> Tuple[float, float, float]:
        """Kaydedilmemişse RuntimeError fırlatır — bu, 'checkpoint kaydı yapılmadan
        Mission Route adımına geçilmez' kuralının kod seviyesinde garantisidir
        (Görev 2 Rapor Bölüm 4.2, KRİTİK — SÜRÜM 2)."""
        if self._checkpoint is None:
            self.publisher.publish(Event(
                code="CHECKPOINT_MISSING_VIOLATION", subsystem="MissionCheckpoint", category=Category.LIFECYCLE,
                severity=Severity.CRITICAL, message="checkpoint.get() called before save()",
            ))
            raise RuntimeError("Görev 2 Rapor Bölüm 4.2 KRİTİK IHLAL: Checkpoint kaydi yapilmadan Mission Route adimina gecilemez!")
        return self._checkpoint

    def is_saved(self) -> bool:
        """Checkpoint kaydedildi mi?"""
        return self._checkpoint is not None
