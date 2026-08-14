import math
from typing import List, Tuple, Optional, Dict
from core.detection.types import Detection

class TargetSelector:
    def __init__(self):
        # Seçilmeyen hedefin kaydedildiği geçici hafıza. Kesin GPS kaydı DEĞİLDİR.
        self.temporary_memory: Dict[str, Detection] = {}

    def select(self, detections: List[Detection], frame_center_px: Tuple[float, float]) -> Tuple[Detection, Optional[Detection]]:
        """Dönüş: (seçilen_hedef, diğer_hedef_ya_da_None).
        Tek hedef varsa diğer_hedef None döner.
        İki hedef varsa, merkeze öklid mesafesi en küçük olan seçilir."""
        
        if not detections:
            raise ValueError("Tespit listesi boş olamaz")
            
        if len(detections) == 1:
            return detections[0], None
            
        def dist_to_center(d: Detection) -> float:
            return math.hypot(d.center_px[0] - frame_center_px[0], d.center_px[1] - frame_center_px[1])
            
        # Merkeze olan uzaklıklarına göre sırala
        sorted_detections = sorted(detections, key=dist_to_center)
        
        selected = sorted_detections[0]
        other = sorted_detections[1]
        
        # Geçici hafızaya al
        self.temporary_memory[other.shape_type] = other
        
        return selected, other
