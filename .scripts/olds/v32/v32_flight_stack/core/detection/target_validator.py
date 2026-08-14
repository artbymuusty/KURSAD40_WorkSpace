import math
from typing import Dict
from core.detection.types import Detection
from core.config.parameters import MISSION_ALTITUDE_M

class TargetValidator:
    def __init__(self, min_consecutive_frames: int = 5, center_tolerance_px: float = 20.0):
        self.min_consecutive_frames = min_consecutive_frames
        self.center_tolerance_px = center_tolerance_px
        self._consecutive_counts: Dict[str, int] = {}
        self._is_centered: Dict[str, bool] = {}
        self._is_navigating_to: Dict[str, bool] = {}
        self._altitude_ok: Dict[str, bool] = {}

    # BUG FIX (single source of truth): this default was a hardcoded 15.0
    # literal, independent of parameters.py's MISSION_ALTITUDE_M -- harmless
    # only by coincidence (both happened to be 15.0). If MISSION_ALTITUDE_M
    # is ever retuned, this would silently stop matching, and altitude_ok
    # would gate on the wrong altitude with no error.
    def update(self, detection: Detection, current_altitude_m: float, frame_center: tuple[float, float], target_altitude_m: float = MISSION_ALTITUDE_M) -> None:
        shape = detection.shape_type
        
        # 1. Görüntüde sürekli takip edilmesi
        self._consecutive_counts[shape] = self._consecutive_counts.get(shape, 0) + 1
        
        # 2. Merkezlenebilmesi
        dist = math.hypot(detection.center_px[0] - frame_center[0], detection.center_px[1] - frame_center[1])
        self._is_centered[shape] = dist <= self.center_tolerance_px
        
        # 4. Doğru irtifada olması
        # Tolerance for altitude could be added, here checking exact or near target altitude
        self._altitude_ok[shape] = abs(current_altitude_m - target_altitude_m) < 0.5

    def set_navigating_to(self, shape_type: str, navigating: bool) -> None:
        """Hedefe gidiliyor bayrağını günceller (navigasyon modülünden çağrılır)."""
        self._is_navigating_to[shape_type] = navigating

    def is_track_ready(self, shape_type: str) -> bool:
        """Koşul 3 ('hedefe gidilmesi') VE koşul 2 ('merkezlenebilmesi') hariç,
        pursuit BAŞLAMADAN ÖNCE değerlendirilebilecek koşullar (1, 4).

        BUG FIX (operator-reported: Mission -> Offboard handover never
        triggers): koşul 2 önceden burada da kontrol ediliyordu ve
        `_is_centered`, hedefin HENÜZ HİÇBİR aktif kontrol uygulanmamış ham
        Mission-modu karesinde ZATEN kare merkezine 20px yakın olmasını
        şart koşuyordu. 19 adımlı durum makinesinde merkezleme (adım 9),
        Offboard'a geçişten (adım 7) VE hedefe gidişten (adım 8) SONRA
        gelir -- yani "önce zaten merkezde ol, sonra merkezlemeye izin
        ver" mantıksal olarak tersti ve pratikte neredeyse hiç
        sağlanamıyordu (uçuş rotası şans eseri hedefin tam üzerinden
        geçmedikçe). Bu da switch_to_offboard()'un neredeyse hiç
        çağrılmamasına yol açan birincil nedendi. Merkezleme artık
        go_to_and_center()'ın kendi işi (bkz. centering_controller.py) --
        bir ön koşul değil.

        is_validated() bu aşamada HER ZAMAN False döner çünkü 'hedefe
        gidilmesi' henüz gerçekleşmemiştir (bkz. is_validated docstring)."""
        c1 = self._consecutive_counts.get(shape_type, 0) >= self.min_consecutive_frames
        c4 = self._altitude_ok.get(shape_type, False)
        return c1 and c4

    def is_validated(self, shape_type: str) -> bool:
        """4 koşulun tamamı sağlanmış mı? Yalnızca pursuit BAŞLADIKTAN (set_navigating_to
        ile işaretlendikten) SONRA, hover_and_confirm() tamamlandıktan sonraki son kontrol
        için kullanılmalıdır — is_track_ready() ile karıştırılmamalıdır."""
        c1 = self._consecutive_counts.get(shape_type, 0) >= self.min_consecutive_frames
        c2 = self._is_centered.get(shape_type, False)
        c3 = self._is_navigating_to.get(shape_type, False)
        c4 = self._altitude_ok.get(shape_type, False)
        return c1 and c2 and c3 and c4

    def get_track_state(self, shape_type: str) -> dict:
        """Public, read-only view of the 4 tracking sub-conditions for a
        shape -- used by observability (ADR-004 §9.2: individual sub-
        conditions must be visible, not just is_track_ready()'s AND result)
        instead of reaching into private attributes across the module
        boundary."""
        return {
            "consecutive_frames": self._consecutive_counts.get(shape_type, 0),
            "is_centered": self._is_centered.get(shape_type, False),
            "is_navigating_to": self._is_navigating_to.get(shape_type, False),
            "altitude_ok": self._altitude_ok.get(shape_type, False),
        }

    def reset(self, shape_type: str) -> None:
        """Sayaçları sıfırlar."""
        if shape_type in self._consecutive_counts:
            self._consecutive_counts[shape_type] = 0
        if shape_type in self._is_centered:
            self._is_centered[shape_type] = False
        if shape_type in self._is_navigating_to:
            self._is_navigating_to[shape_type] = False
        if shape_type in self._altitude_ok:
            self._altitude_ok[shape_type] = False
