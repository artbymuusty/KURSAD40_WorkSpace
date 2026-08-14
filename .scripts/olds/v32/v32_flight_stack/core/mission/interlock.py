"""
Görev 2 Rapor Bölüm 11.1'deki EN KRİTİK kuralın saf, backend'den tamamen bağımsız,
kolayca test edilebilir implementasyonu.
"""
from core.telemetry.event_bus import NULL_PUBLISHER, EventPublisher
from core.telemetry.events import Category, Event, Severity


class PayloadInterlock:
    def __init__(self, publisher: EventPublisher = NULL_PUBLISHER):
        self._payload_1_released: bool = False   # Mavi Altıgen
        self._payload_2_released: bool = False   # Kırmızı Üçgen
        self.publisher = publisher

    def mark_payload_1_released(self) -> None:
        """Yalnızca Mavi Altıgen'e yük bırakıldığında çağrılır."""
        self._payload_1_released = True
        self.publisher.publish(Event(
            code="PAYLOAD_1_RELEASED", subsystem="PayloadInterlock", category=Category.PAYLOAD,
            message="payload 1 (MAVI_ALTIGEN) released",
            data={"payload_1_released": True, "payload_2_released": self._payload_2_released},
        ))

    def can_release_payload_2(self) -> bool:
        """KRİTİK: Bu, Kırmızı Üçgen'e yük bırakılmasının TEK kapısıdır.
        payload_1_released False iken bu HER ZAMAN False döner — istisnasız."""
        return self._payload_1_released

    def mark_payload_2_released(self) -> None:
        """ÖNKOŞUL: can_release_payload_2() True olmalı. Değilse RuntimeError fırlat —
        bu, interlock kuralının yazılım seviyesinde İHLAL EDİLEMEZ olmasını sağlar
        (Görev 2 Rapor Bölüm 11.1: 'Kırmızı Üçgen'e yük... KESİNLİKLE gerçekleştirilemez')."""
        if not self.can_release_payload_2():
            # ADR-004 §9.4: this is the interlock working CORRECTLY -- but a
            # raised exception alone reaches a log/traceback, not the
            # operator's screen. Publish it as CRITICAL before raising so
            # the Mission Operations Center shows it regardless of whether
            # anyone is watching a terminal.
            self.publisher.publish(Event(
                code="INTERLOCK_VIOLATION_BLOCKED", subsystem="PayloadInterlock", category=Category.PAYLOAD,
                severity=Severity.CRITICAL,
                message="payload_2 release attempted while payload_1_released=False -- blocked",
                data={"payload_1_released": False, "payload_2_released": self._payload_2_released},
            ))
            raise RuntimeError(
                "INTERLOCK IHLALI: payload_1_released=False iken payload_2 birakilamaz "
                "(Gorev 2 Rapor Bolum 11.1)"
            )
        self._payload_2_released = True
        self.publisher.publish(Event(
            code="PAYLOAD_2_RELEASED", subsystem="PayloadInterlock", category=Category.PAYLOAD,
            message="payload 2 (KIRMIZI_UCGEN) released",
            data={"payload_1_released": True, "payload_2_released": True},
        ))

    def both_released(self) -> bool:
        return self._payload_1_released and self._payload_2_released

    @property
    def payload_1_released(self) -> bool:
        return self._payload_1_released

    @property
    def payload_2_released(self) -> bool:
        return self._payload_2_released
